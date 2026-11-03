#include <errno.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/nemu_link/settings_relay.h>

#define RELAY_REQUEST_SIZE 36
#define RELAY_RESPONSE_SIZE 58
#define RELAY_CHUNK_HEADER 5
#define RELAY_CHUNK_DATA 15
#define RELAY_TIMEOUT K_SECONDS(10)
#define RELAY_SERVER_STACK_SIZE 2048

static struct bt_uuid_128 service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x3fb4f6a1, 0x7806, 0x4df5, 0x9c18, 0x18e14502a5c1));
static struct bt_uuid_128 request_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x3fb4f6a2, 0x7806, 0x4df5, 0x9c18, 0x18e14502a5c1));
static struct bt_uuid_128 response_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x3fb4f6a3, 0x7806, 0x4df5, 0x9c18, 0x18e14502a5c1));

struct relay_rx {
    struct bt_conn *conn;
    uint16_t transaction;
    int64_t updated_ms;
    uint8_t total, used, bytes[RELAY_REQUEST_SIZE];
    bool active;
};
static struct relay_rx server_rx;
static struct bt_conn *server_conn;
static uint8_t server_request[RELAY_REQUEST_SIZE], server_response[RELAY_RESPONSE_SIZE];
static uint16_t server_transaction;
static uint32_t server_session, server_sequence;
static uint8_t server_last_request[RELAY_REQUEST_SIZE];
static bool server_work_pending;
static K_MUTEX_DEFINE(server_lock);
static K_SEM_DEFINE(server_notify_done, 0, 1);
static struct k_work_q server_work_q;
K_THREAD_STACK_DEFINE(server_work_stack, RELAY_SERVER_STACK_SIZE);
static uint8_t client_write_error;

static struct bt_conn *client_conn;
static uint16_t client_request_handle, client_response_handle, client_ccc_handle;
static uint16_t client_transaction;
static bool client_ready;
static struct bt_gatt_discover_params client_discover;
static struct bt_gatt_subscribe_params client_subscribe;
static struct bt_gatt_write_params client_write;
static struct k_sem client_write_done;
static struct k_sem client_response_done;
static struct k_spinlock client_response_lock;
static uint8_t client_response[RELAY_RESPONSE_SIZE];
static uint8_t client_response_used;
static uint16_t client_response_id;
static bool client_response_valid;
static K_MUTEX_DEFINE(client_call_lock);

static void request_work(struct k_work *work);
K_WORK_DEFINE(server_work, request_work);

static void encode_request(uint8_t bytes[RELAY_REQUEST_SIZE],
                           const struct dynamic_setting_remote_request *request) {
    bytes[0] = request->operation;
    bytes[1] = request->target;
    bytes[2] = request->value_type;
    bytes[3] = request->value_length;
    sys_put_le32(request->session_id, bytes + 4);
    sys_put_le32(request->sequence, bytes + 8);
    sys_put_le32(request->group_id, bytes + 12);
    sys_put_le32(request->setting_id, bytes + 16);
    sys_put_le16(request->offset, bytes + 20);
    sys_put_le16(request->total_length, bytes + 22);
    memcpy(bytes + 24, request->value, DYNAMIC_SETTINGS_REMOTE_VALUE_MAX);
}

static bool decode_request(struct dynamic_setting_remote_request *request,
                           const uint8_t bytes[RELAY_REQUEST_SIZE]) {
    if (!request || bytes[1] != DYNAMIC_SETTING_SCOPE_OWNER ||
        bytes[3] > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX) {
        return false;
    }
    *request = (struct dynamic_setting_remote_request){
        .operation = bytes[0],
        .target = bytes[1],
        .value_type = bytes[2],
        .value_length = bytes[3],
        .session_id = sys_get_le32(bytes + 4),
        .sequence = sys_get_le32(bytes + 8),
        .group_id = sys_get_le32(bytes + 12),
        .setting_id = sys_get_le32(bytes + 16),
        .offset = sys_get_le16(bytes + 20),
        .total_length = sys_get_le16(bytes + 22),
    };
    memcpy(request->value, bytes + 24, sizeof(request->value));
    return request->session_id && request->sequence;
}

static void encode_response(uint8_t bytes[RELAY_RESPONSE_SIZE], int error,
                            const struct dynamic_setting_remote_response *response) {
    memset(bytes, 0, RELAY_RESPONSE_SIZE);
    sys_put_le16((uint16_t)(int16_t)error, bytes);
    if (!response) {
        return;
    }
    bytes[2] = response->value_type;
    bytes[3] = response->value_length;
    bytes[4] = response->origin;
    bytes[5] = response->dirty;
    bytes[6] = response->persisted;
    bytes[7] = response->schema_known;
    sys_put_le32(response->generation, bytes + 8);
    sys_put_le16(response->total_length, bytes + 12);
    memcpy(bytes + 14, response->value, DYNAMIC_SETTINGS_REMOTE_VALUE_MAX);
    memcpy(bytes + 26, response->schema_sha256, sizeof(response->schema_sha256));
}

static bool decode_response(struct dynamic_setting_remote_response *response, int *error,
                            const uint8_t bytes[RELAY_RESPONSE_SIZE]) {
    if (!response || !error || bytes[3] > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX ||
        bytes[4] > DYNAMIC_SETTING_ORIGIN_TEMP || bytes[7] > 1) {
        return false;
    }
    *error = (int16_t)sys_get_le16(bytes);
    *response = (struct dynamic_setting_remote_response){
        .value_type = bytes[2],
        .value_length = bytes[3],
        .origin = bytes[4],
        .dirty = bytes[5],
        .persisted = bytes[6],
        .schema_known = bytes[7],
        .generation = sys_get_le32(bytes + 8),
        .total_length = sys_get_le16(bytes + 12),
    };
    memcpy(response->value, bytes + 14, sizeof(response->value));
    memcpy(response->schema_sha256, bytes + 26, sizeof(response->schema_sha256));
    return true;
}

static ssize_t relay_request_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    if (!buf || offset || len < RELAY_CHUNK_HEADER) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    const uint8_t *bytes = buf;
    uint16_t transaction = sys_get_le16(bytes);
    uint8_t total = bytes[2], chunk_offset = bytes[3], chunk_length = bytes[4];
    if (!transaction || total != RELAY_REQUEST_SIZE || len != RELAY_CHUNK_HEADER + chunk_length ||
        !chunk_length || chunk_length > RELAY_CHUNK_DATA ||
        (uint16_t)chunk_offset + chunk_length > total) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    k_mutex_lock(&server_lock, K_FOREVER);
    if (server_rx.active && k_uptime_get() - server_rx.updated_ms >= 10000) {
        bt_conn_unref(server_rx.conn);
        server_rx = (struct relay_rx){0};
    }
    if (server_work_pending) {
        k_mutex_unlock(&server_lock);
        return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
    }
    if (!server_rx.active) {
        if (chunk_offset) {
            k_mutex_unlock(&server_lock);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }
        server_rx = (struct relay_rx){.conn = bt_conn_ref(conn),
                                      .transaction = transaction,
                                      .updated_ms = k_uptime_get(),
                                      .total = total,
                                      .active = true};
    }
    if (server_rx.conn != conn || server_rx.transaction != transaction ||
        server_rx.total != total || server_rx.used != chunk_offset) {
        k_mutex_unlock(&server_lock);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    memcpy(server_rx.bytes + chunk_offset, bytes + RELAY_CHUNK_HEADER, chunk_length);
    server_rx.used += chunk_length;
    server_rx.updated_ms = k_uptime_get();
    server_transaction = transaction;
    if (server_rx.used == server_rx.total) {
        memcpy(server_request, server_rx.bytes, sizeof(server_request));
        server_conn = server_rx.conn;
        server_rx.conn = NULL;
        server_rx.active = false;
        if (!server_work_pending) {
            server_work_pending = true;
            k_work_submit_to_queue(&server_work_q, &server_work);
        }
    }
    k_mutex_unlock(&server_lock);
    return len;
}

static void relay_response_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
}

static void relay_notify_complete(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&server_notify_done);
}

BT_GATT_SERVICE_DEFINE(nemu_link_settings_relay_service, BT_GATT_PRIMARY_SERVICE(&service_uuid),
                       BT_GATT_CHARACTERISTIC(&request_uuid.uuid, BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE_ENCRYPT, NULL, relay_request_write,
                                              NULL),
                       BT_GATT_CHARACTERISTIC(&response_uuid.uuid, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE, NULL, NULL, NULL),
                       BT_GATT_CCC(relay_response_ccc,
                                   BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT));

static void request_work(struct k_work *work) {
    ARG_UNUSED(work);
    struct dynamic_setting_remote_request request;
    struct dynamic_setting_remote_response response = {0};
    int err;
    bool cached = false;
    if (!decode_request(&request, server_request)) {
        err = -EINVAL;
        encode_response(server_response, err, NULL);
    } else {
        k_mutex_lock(&server_lock, K_FOREVER);
        bool replay = request.session_id == server_session && request.sequence == server_sequence &&
                      !memcmp(server_request, server_last_request, sizeof(server_request));
        uint32_t sequence_delta = request.sequence - server_sequence;
        bool stale = request.session_id == server_session &&
                     (sequence_delta == 0 || sequence_delta >= UINT32_C(0x80000000));
        if (request.session_id != server_session) {
            server_session = request.session_id;
            server_sequence = 0;
        }
        k_mutex_unlock(&server_lock);
        if (replay) {
            err = sys_get_le16(server_response);
            cached = true;
        } else if (stale) {
            err = -ESTALE;
        } else {
            err = nemu_link_settings_relay_dispatch(&request, &response);
            encode_response(server_response, err, &response);
            k_mutex_lock(&server_lock, K_FOREVER);
            server_sequence = request.sequence;
            memcpy(server_last_request, server_request, sizeof(server_last_request));
            k_mutex_unlock(&server_lock);
        }
    }
    ARG_UNUSED(cached);
    for (uint8_t offset = 0; offset < RELAY_RESPONSE_SIZE; offset += RELAY_CHUNK_DATA) {
        uint8_t packet[RELAY_CHUNK_HEADER + RELAY_CHUNK_DATA] = {0};
        uint8_t count = MIN(RELAY_CHUNK_DATA, RELAY_RESPONSE_SIZE - offset);
        sys_put_le16(server_transaction, packet);
        packet[2] = RELAY_RESPONSE_SIZE;
        packet[3] = offset;
        packet[4] = count;
        memcpy(packet + RELAY_CHUNK_HEADER, server_response + offset, count);
        if (!server_conn || bt_conn_get_security(server_conn) < BT_SECURITY_L2) {
            break;
        }
        struct bt_gatt_notify_params params = {
            .attr = &nemu_link_settings_relay_service.attrs[4],
            .data = packet,
            .len = RELAY_CHUNK_HEADER + count,
            .func = relay_notify_complete,
        };
        k_sem_reset(&server_notify_done);
        if (bt_gatt_notify_cb(server_conn, &params) ||
            k_sem_take(&server_notify_done, K_SECONDS(2))) {
            break;
        }
    }
    k_mutex_lock(&server_lock, K_FOREVER);
    if (server_conn) {
        bt_conn_unref(server_conn);
        server_conn = NULL;
    }
    server_work_pending = false;
    k_mutex_unlock(&server_lock);
}

static uint8_t client_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                             const void *data, uint16_t length) {
    ARG_UNUSED(conn);
    ARG_UNUSED(params);
    if (!data || length < RELAY_CHUNK_HEADER) {
        return BT_GATT_ITER_CONTINUE;
    }
    const uint8_t *bytes = data;
    uint16_t transaction = sys_get_le16(bytes);
    uint8_t total = bytes[2], offset = bytes[3], count = bytes[4];
    if (total != RELAY_RESPONSE_SIZE || length != RELAY_CHUNK_HEADER + count || !count ||
        count > RELAY_CHUNK_DATA || (uint16_t)offset + count > total) {
        return BT_GATT_ITER_CONTINUE;
    }
    k_spinlock_key_t key = k_spin_lock(&client_response_lock);
    if (transaction != client_response_id || offset != client_response_used) {
        if (!offset && transaction == client_response_id) {
            client_response_used = 0;
            client_response_valid = false;
        } else {
            k_spin_unlock(&client_response_lock, key);
            return BT_GATT_ITER_CONTINUE;
        }
    }
    memcpy(client_response + offset, bytes + RELAY_CHUNK_HEADER, count);
    client_response_used += count;
    if (client_response_used == RELAY_RESPONSE_SIZE) {
        client_response_valid = true;
        k_sem_give(&client_response_done);
    }
    k_spin_unlock(&client_response_lock, key);
    return BT_GATT_ITER_CONTINUE;
}

static void client_write_complete(struct bt_conn *conn, uint8_t err,
                                  struct bt_gatt_write_params *params) {
    ARG_UNUSED(conn);
    ARG_UNUSED(params);
    client_write_error = err;
    k_sem_give(&client_write_done);
}

static uint8_t client_discover_service(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       struct bt_gatt_discover_params *params) {
    if (!attr) {
        return BT_GATT_ITER_STOP;
    }
    client_request_handle = attr->handle + 2;
    client_response_handle = attr->handle + 4;
    client_ccc_handle = attr->handle + 5;
    client_subscribe = (struct bt_gatt_subscribe_params){
        .notify = client_notify,
        .value = BT_GATT_CCC_NOTIFY,
        .ccc_handle = client_ccc_handle,
        .value_handle = client_response_handle,
    };
    int err = bt_gatt_subscribe(conn, &client_subscribe);
    client_ready = !err;
    ARG_UNUSED(params);
    return BT_GATT_ITER_STOP;
}

static void client_connected(struct bt_conn *conn, uint8_t err) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    if (err || client_conn) {
        return;
    }
    client_conn = bt_conn_ref(conn);
    client_discover = (struct bt_gatt_discover_params){
        .uuid = &service_uuid.uuid,
        .func = client_discover_service,
        .start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
        .type = BT_GATT_DISCOVER_PRIMARY,
    };
    if (bt_gatt_discover(conn, &client_discover)) {
        bt_conn_unref(client_conn);
        client_conn = NULL;
    }
#else
    ARG_UNUSED(conn);
    ARG_UNUSED(err);
#endif
}

static void client_disconnected(struct bt_conn *conn, uint8_t reason) {
    k_mutex_lock(&server_lock, K_FOREVER);
    if (server_rx.conn == conn) {
        bt_conn_unref(server_rx.conn);
        server_rx = (struct relay_rx){0};
    }
    k_mutex_unlock(&server_lock);
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    ARG_UNUSED(reason);
    if (conn != client_conn) {
        return;
    }
    client_ready = false;
    client_request_handle = 0;
    client_response_handle = 0;
    client_ccc_handle = 0;
    bt_conn_unref(client_conn);
    client_conn = NULL;
    k_spinlock_key_t key = k_spin_lock(&client_response_lock);
    client_response_valid = false;
    client_response_used = 0;
    k_spin_unlock(&client_response_lock, key);
    client_write_error = 1;
    k_sem_give(&client_write_done);
    k_sem_give(&client_response_done);
#else
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);
#endif
}

BT_CONN_CB_DEFINE(nemu_link_settings_relay_conn_callbacks) = {
    .connected = client_connected,
    .disconnected = client_disconnected,
};

static void client_write_done_callback(struct bt_conn *conn, uint8_t err,
                                       struct bt_gatt_write_params *params) {
    client_write_complete(conn, err, params);
}

int nemu_link_settings_ble_relay_call(const struct dynamic_setting_remote_request *request,
                                      struct dynamic_setting_remote_response *response) {
    if (!request || !response || request->target != DYNAMIC_SETTING_SCOPE_OWNER ||
        request->value_length > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX) {
        return -EINVAL;
    }
    k_mutex_lock(&client_call_lock, K_FOREVER);
    if (!client_conn || !client_ready || !client_request_handle) {
        k_mutex_unlock(&client_call_lock);
        return -EHOSTUNREACH;
    }
    if (bt_conn_get_security(client_conn) < BT_SECURITY_L2) {
        k_mutex_unlock(&client_call_lock);
        return -EACCES;
    }
    uint8_t request_bytes[RELAY_REQUEST_SIZE];
    encode_request(request_bytes, request);
    client_transaction++;
    if (!client_transaction) {
        client_transaction = 1;
    }
    k_spinlock_key_t key = k_spin_lock(&client_response_lock);
    client_response_id = client_transaction;
    client_response_used = 0;
    client_response_valid = false;
    k_spin_unlock(&client_response_lock, key);
    k_sem_reset(&client_response_done);
    client_write_error = 0;
    for (uint8_t offset = 0; offset < RELAY_REQUEST_SIZE; offset += RELAY_CHUNK_DATA) {
        uint8_t packet[RELAY_CHUNK_HEADER + RELAY_CHUNK_DATA] = {0};
        uint8_t count = MIN(RELAY_CHUNK_DATA, RELAY_REQUEST_SIZE - offset);
        sys_put_le16(client_transaction, packet);
        packet[2] = RELAY_REQUEST_SIZE;
        packet[3] = offset;
        packet[4] = count;
        memcpy(packet + RELAY_CHUNK_HEADER, request_bytes + offset, count);
        client_write = (struct bt_gatt_write_params){
            .handle = client_request_handle,
            .offset = 0,
            .data = packet,
            .length = RELAY_CHUNK_HEADER + count,
            .func = client_write_done_callback,
        };
        k_sem_reset(&client_write_done);
        int err = bt_gatt_write(client_conn, &client_write);
        if (err || k_sem_take(&client_write_done, RELAY_TIMEOUT)) {
            k_mutex_unlock(&client_call_lock);
            return err ? err : -ETIMEDOUT;
        }
        if (client_write_error) {
            k_mutex_unlock(&client_call_lock);
            return -ECONNRESET;
        }
    }
    if (k_sem_take(&client_response_done, RELAY_TIMEOUT)) {
        k_mutex_unlock(&client_call_lock);
        return -ETIMEDOUT;
    }
    key = k_spin_lock(&client_response_lock);
    bool valid = client_response_valid;
    uint8_t response_bytes[RELAY_RESPONSE_SIZE];
    memcpy(response_bytes, client_response, sizeof(response_bytes));
    k_spin_unlock(&client_response_lock, key);
    k_mutex_unlock(&client_call_lock);
    int error;
    if (!valid || !decode_response(response, &error, response_bytes)) {
        return -EINVAL;
    }
    return error;
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && !IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
static int split_ble_settings_remote_call(const struct dynamic_setting_remote_request *request,
                                          struct dynamic_setting_remote_response *response,
                                          void *context) {
    ARG_UNUSED(context);
    if (!request || !response || request->target != DYNAMIC_SETTING_SCOPE_OWNER) {
        return -EHOSTUNREACH;
    }
    return nemu_link_settings_ble_relay_call(request, response);
}

static int split_ble_settings_remote_init(void) {
    return dynamic_settings_register_remote(split_ble_settings_remote_call, NULL);
}

SYS_INIT(split_ble_settings_remote_init, APPLICATION, CONFIG_ZMK_DYNAMIC_SETTINGS_INIT_PRIORITY);
#endif

static int client_init(void) {
    k_sem_init(&client_write_done, 0, 1);
    k_sem_init(&client_response_done, 0, 1);
    k_work_queue_start(&server_work_q, server_work_stack, K_THREAD_STACK_SIZEOF(server_work_stack),
                       K_PRIO_PREEMPT(5), NULL);
    return 0;
}
SYS_INIT(client_init, APPLICATION, 99);
