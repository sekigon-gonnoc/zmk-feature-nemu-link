#include <errno.h>
#include <string.h>
#include <zmk/dynamic_settings/schema.h>
#include <zmk/nemu_link/settings_relay.h>

static int finish_response(const struct dynamic_setting_remote_request *request,
                           const struct dynamic_setting_group_descriptor *group,
                           struct dynamic_setting_remote_response *response, int err) {
    if (!err && request->target == DYNAMIC_SETTING_SCOPE_OWNER) {
        err = dynamic_settings_owner_contract_digest(group->id, response->schema_sha256);
        response->schema_known = !err;
    }
    return err;
}

int nemu_link_settings_relay_dispatch(const struct dynamic_setting_remote_request *request,
                                      struct dynamic_setting_remote_response *response) {
    if (!request || !response) {
        return -EINVAL;
    }
    const struct dynamic_setting_group_descriptor *group = NULL;
    const struct dynamic_setting_descriptor *descriptor = NULL;
    if (request->target != DYNAMIC_SETTING_SCOPE_OWNER &&
        request->target != DYNAMIC_SETTING_SCOPE_SHARED) {
        return -EACCES;
    }
    if (((request->operation == 2 || request->operation == 3) && request->setting_id) ||
        (request->operation == 4 && request->setting_id)) {
        descriptor = dynamic_settings_find_setting(request->setting_id);
        group = dynamic_settings_find_setting_group(request->setting_id);
        if (request->group_id && group && request->group_id != group->id) {
            return -ENOENT;
        }
    } else {
        group = dynamic_settings_find_group(request->group_id);
    }
    if (!group || group->scope != request->target ||
        (group->flags & DYNAMIC_SETTING_GROUP_FLAG_PROXY)) {
        return -ENOENT;
    }
    bool record_operation = (group->flags & DYNAMIC_SETTING_GROUP_FLAG_RECORD) &&
                            (request->operation == 2 || request->operation == 3);
    if ((request->operation == 2 || request->operation == 3) && !record_operation && !descriptor) {
        return -ENOENT;
    }
    *response = (struct dynamic_setting_remote_response){0};
    if (record_operation) {
        if (request->value_length > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX) {
            return -EINVAL;
        }
        if (request->operation == 2) {
            size_t total_length = 0;
            enum dynamic_setting_value_origin origin;
            if (request->value_length > 10) {
                return -EINVAL;
            }
            int err = dynamic_settings_record_read(group->id, request->offset, response->value,
                                                   request->value_length, &total_length, &origin);
            if (err) {
                return err;
            }
            response->value_length = request->value_length;
            response->total_length = total_length;
            response->origin = origin;
            response->dirty = dynamic_settings_is_dirty(group->id);
            response->persisted = origin == DYNAMIC_SETTING_ORIGIN_PERSISTED;
            response->generation = dynamic_settings_generation(group->scope);
            return finish_response(request, group, response, 0);
        }
        int err = dynamic_settings_record_write_temp_chunk(
            group->id, request->session_id, request->sequence, request->offset,
            request->total_length, request->value, request->value_length);
        response->total_length = request->total_length;
        response->dirty = dynamic_settings_is_dirty(group->id);
        response->generation = dynamic_settings_generation(group->scope);
        return finish_response(request, group, response, err);
    }
    switch (request->operation) {
    case 2: {
        struct dynamic_setting_value value;
        enum dynamic_setting_value_origin origin;
        int err = dynamic_settings_read(request->setting_id, &value, &origin);
        if (err) {
            return err;
        }
        response->value_type = value.type;
        response->value_length = value.length;
        memcpy(response->value, value.data, value.length);
        response->origin = origin;
        response->dirty = dynamic_settings_is_dirty(group->id);
        response->persisted = origin == DYNAMIC_SETTING_ORIGIN_PERSISTED;
        response->generation = dynamic_settings_generation(group->scope);
        return finish_response(request, group, response, 0);
    }
    case 3: {
        if (request->value_length > DYNAMIC_SETTINGS_VALUE_MAX ||
            request->value_type != descriptor->type ||
            (descriptor->flags & DYNAMIC_SETTING_FLAG_READ_ONLY)) {
            return -EINVAL;
        }
        struct dynamic_setting_value value = {
            .type = request->value_type,
            .length = request->value_length,
        };
        memcpy(value.data, request->value, value.length);
        int err = dynamic_settings_write_temp(request->setting_id, &value);
        if (err) {
            return err;
        }
        response->value_type = value.type;
        response->dirty = dynamic_settings_is_dirty(group->id);
        response->generation = dynamic_settings_generation(group->scope);
        return finish_response(request, group, response, 0);
    }
    case 4:
        if (request->setting_id &&
            (!descriptor || dynamic_settings_find_setting_group(request->setting_id) != group)) {
            return -ENOENT;
        }
        if (request->setting_id && (descriptor->flags & DYNAMIC_SETTING_FLAG_READ_ONLY)) {
            return -EACCES;
        }
        {
            int err = dynamic_settings_reset(group->id, request->setting_id);
            if (err) {
                return err;
            }
        }
        response->dirty = dynamic_settings_is_dirty(group->id);
        response->generation = dynamic_settings_generation(group->scope);
        return finish_response(request, group, response, 0);
    case 5: {
        int err = dynamic_settings_save_scope(group->scope);
        response->dirty = dynamic_settings_scope_is_dirty(group->scope);
        response->persisted = dynamic_settings_scope_has_persisted_value(group->scope);
        response->generation = dynamic_settings_generation(group->scope);
        return finish_response(request, group, response, err);
    }
    case 6: {
        int err = dynamic_settings_discard(group->id);
        response->dirty = dynamic_settings_is_dirty(group->id);
        response->generation = dynamic_settings_generation(group->scope);
        return finish_response(request, group, response, err);
    }
    case 7:
        response->dirty = dynamic_settings_is_dirty(group->id);
        response->persisted = dynamic_settings_group_has_persisted_value(group->id);
        response->generation = dynamic_settings_generation(group->scope);
        return finish_response(request, group, response, 0);
    default:
        return -ENOTSUP;
    }
}
