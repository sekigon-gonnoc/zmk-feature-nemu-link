#!/usr/bin/env python3
"""Generate the Studio semantic schema from the build devicetree."""
import argparse
import json
import pickle
import sys
import zlib
from pathlib import Path


def checksum32(data):
    checksum = 0
    for offset in range(0, len(data), 4):
        checksum ^= int.from_bytes(data[offset:offset + 4], 'little')
    return checksum


def semantic(value):
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, (list, tuple)):
        return [semantic(v) for v in value]
    if hasattr(value, 'controller'):
        return [device_name(value.controller), dict(value.data)]
    if hasattr(value, 'path'):
        return device_name(value)
    raise ValueError(f'Unsupported devicetree value {value!r}')


def device_name(node):
    return node.props['label'].val if 'label' in node.props else node.name


def binding_cells(node):
    prop=node._node.props.get('#binding-cells')
    return prop.to_num() if prop is not None else None


def is_behavior_node(node):
    # ZMK behaviors expose the standard phandle binding-cell metadata. Use that
    # structural contract instead of matching vendor/namespace prefixes so
    # behavior modules can add compatibles without changing this generator.
    return binding_cells(node) is not None


def node_contract(node):
    return {'name': device_name(node), 'status': node.status, 'compatible':node.compats,
            'binding_cells':binding_cells(node),
            'props': {k: semantic(p.val) for k, p in node.props.items()
                      if k not in ('phandle', 'label')},
            'children': [node_contract(c) for c in node.children.values()]}


def layer_mask(node, seen=()):
    if node.path in seen:
        raise ValueError('Recursive behavior binding')
    if any(c in node.compats for c in ('zmk,behavior-momentary-layer',
           'zmk,behavior-toggle-layer', 'zmk,behavior-to-layer')):
        return 1
    if any(c in node.compats for c in ('zmk,behavior-hold-tap', 'zmk,behavior-sticky-key')):
        bindings=node.props['bindings'].val
        return sum((1 << i) for i, b in enumerate(bindings)
                   if layer_mask(getattr(b, 'controller', b), (*seen, node.path)) & 1)
    return 0


def build_contract(edt, app):
    layers=list(edt.compat2nodes['zmk,keymap'][0].children.values())
    layouts=[n for n in edt.compat2nodes['zmk,physical-layout'] if n.status=='okay']
    if not layouts or not 1 <= len(layers) <= 32:
        raise ValueError('Studio requires physical layouts and 1..32 layers')
    behavior_nodes=[n for n in edt.nodes if n.status=='okay' and is_behavior_node(n)]
    behaviors=sorted(behavior_nodes, key=device_name)
    if not behaviors:
        raise ValueError('Studio behavior table cannot be empty')
    if any(binding_cells(n) > 2 for n in behaviors):
        raise ValueError('Studio bindings support at most two parameters')
    contract={'protocol':'nemu-link-general-behaviors-v1', 'layers':[], 'layouts':[],
              'behaviors':[node_contract(n) for n in sorted(behavior_nodes,key=lambda n:n.path)],
              'sensors':[node_contract(n) for n in edt.compat2nodes.get('zmk,keymap-sensors',[])],
              'combos':[node_contract(n) for n in edt.compat2nodes.get('zmk,combos',[]) if n.status=='okay'],
              'position_maps':[node_contract(n) for n in edt.compat2nodes.get('zmk,physical-layout-position-map',[]) if n.status=='okay'],
              'stock_layout':device_name(edt.chosen_node('zmk,physical-layout'))}
    for layer in layers:
        bindings=semantic(layer.props['bindings'].val)
        if not 1 <= len(bindings) <= 128 or (contract['layers'] and len(bindings)!=len(contract['layers'][0]['bindings'])):
            raise ValueError('Studio requires equal layer lengths in 1..128')
        contract['layers'].append({'id':layer.name,'status':layer.status,'bindings':bindings,
                                  'sensor_bindings':semantic(layer.props['sensor-bindings'].val) if 'sensor-bindings' in layer.props else []})
    for layout in layouts:
        transform=layout.props['transform'].val
        contract['layouts'].append({'id':device_name(layout),'name':layout.props['display-name'].val,
            'keys':semantic(layout.props['keys'].val),
            # Scan offsets identify the local half, not shared position semantics.
            'transform':{k:p.val for k,p in transform.props.items() if k in ('rows','columns','map')}})
    return contract, layers, layouts, behaviors


def contract_digest(contract):
    encoded = json.dumps(contract, sort_keys=True, separators=(',', ':')).encode()
    semantics = {k:v for k,v in contract.items() if k != 'layers'}
    semantics['sensor_bindings'] = {i:l['sensor_bindings'] for i,l in enumerate(contract['layers']) if l['sensor_bindings']}
    semantic_bytes = json.dumps(semantics, sort_keys=True, separators=(',', ':')).encode()
    # First CRC protects shared behavior/combo/layout semantics. The second
    # describes stock bindings and dimensions; bounded clipping remains supported.
    return zlib.crc32(semantic_bytes).to_bytes(4,'little') + zlib.crc32(encoded).to_bytes(4,'little') + bytes(20) + b'ESG1'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--app', type=Path, required=True)
    ap.add_argument('--zephyr', type=Path, required=True)
    ap.add_argument('--edt', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    sys.path.insert(0, str(args.zephyr / 'scripts/dts/python-devicetree/src'))
    edt = pickle.loads(args.edt.read_bytes())
    contract, layers, layouts, behaviors = build_contract(edt, args.app)
    checksum = contract_digest(contract)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'studio-contract.json').write_text(json.dumps(contract, sort_keys=True, indent=2) + '\n')
    order = [i for i, l in enumerate(layers) if l.status == 'okay']
    order += [255] * (len(layers) - len(order))
    (args.out / 'studio_schema.h').write_text('#pragma once\nstatic const uint8_t studio_build_contract[32] = {' +
        ','.join(str(b) for b in checksum) + '};\nstatic const uint8_t studio_stock_order[] = {' +
        ','.join(str(b) for b in order) + '};\n')
    table = ',\n'.join('    {' + json.dumps(device_name(n)) + ', ' + str(binding_cells(n)) + '}' for n in behaviors)
    masks = ','.join(str(layer_mask(n)) for n in behaviors)
    with (args.out / 'studio_schema.h').open('a') as header:
        header.write('static const struct nemu_link_studio_behavior studio_behaviors[] = {\n' + table + '\n};\n')
        header.write('static const uint8_t studio_layer_params[] = {' + masks + '};\n')
        header.write('#define STUDIO_LAYOUT_COUNT ' + str(len(layouts)) + '\n')
        stock=edt.chosen_node('zmk,physical-layout')
        header.write('#define STUDIO_STOCK_LAYOUT ' + str(layouts.index(stock)) + '\n')



if __name__ == '__main__':
    main()
