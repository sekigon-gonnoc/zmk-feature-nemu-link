#include <errno.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/dynamic_settings/schema.h>
#include <zmk/nemu_link/settings_relay.h>
#include <zmk/nemu_link/studio_transport.h>

#define ESB_REMOTE_REQUEST_SIZE NEMU_LINK_STUDIO_CHUNK_SIZE
#define ESB_REMOTE_RESPONSE_SIZE 16
#define ESB_REMOTE_RETRY_MS 40
#define ESB_REMOTE_TIMEOUT_MS 10000
#define ESB_SERVER_QUEUE_DEPTH 2

LOG_MODULE_REGISTER(nemu_link_settings_esb, LOG_LEVEL_INF);

static atomic_t client_calls, client_request_packets, client_send_errors;
static atomic_t client_response_packets, client_response_drops, client_timeouts;
static atomic_t client_route_drops, client_expired_drops, client_decode_drops;
static atomic_t server_request_packets, server_request_drops, server_requests_queued;
static atomic_t server_queue_drops, server_dispatch_ok, server_dispatch_errors;
static atomic_t server_response_packets, server_response_send_errors;
static atomic_t server_route_drops, server_expired_drops, server_decode_drops;
static atomic_t client_last_error, server_last_dispatch_error, server_last_send_error;

static K_MUTEX_DEFINE(client_lock);
static K_MUTEX_DEFINE(client_call_lock);
static K_SEM_DEFINE(client_done, 0, 1);
static struct nemu_link_studio_packet client_request;
static uint16_t client_transaction;
static uint8_t client_target_source;
static uint8_t client_response[ESB_REMOTE_RESPONSE_SIZE];
static bool client_response_received, client_schema_expected;
static uint8_t client_schema_received, client_schema_hash[32];
static int client_error;
static bool client_active;

static uint8_t server_last_request[ESB_REMOTE_REQUEST_SIZE];
static struct nemu_link_studio_packet server_last_response;
static struct nemu_link_studio_packet server_last_schema[2];
static uint16_t server_last_transaction;
static uint32_t server_last_session, server_last_sequence;
static struct nemu_link_studio_route server_last_route;
static K_MUTEX_DEFINE(server_lock);

struct server_job {
    struct nemu_link_studio_packet packet;
    struct nemu_link_studio_route route;
};
K_MSGQ_DEFINE(server_jobs, sizeof(struct server_job), ESB_SERVER_QUEUE_DEPTH, 4);
K_THREAD_STACK_DEFINE(server_stack, 2048);
static struct k_thread server_thread;

static bool sequence_newer(uint32_t candidate, uint32_t current) {
    uint32_t delta = candidate - current;
    return delta && delta < UINT32_C(0x80000000);
}

static void encode_request(struct nemu_link_studio_packet *packet,
                           const struct dynamic_setting_remote_request *request) {
    *packet = (struct nemu_link_studio_packet){
        .kind = NEMU_LINK_STUDIO_REMOTE_REQUEST,
        .transaction = client_transaction,
        .length = ESB_REMOTE_REQUEST_SIZE,
    };
    const struct dynamic_setting_group_descriptor *group =
        request->setting_id ? dynamic_settings_find_setting_group(request->setting_id)
                            : dynamic_settings_find_group(request->group_id);
    if (group && (group->flags & DYNAMIC_SETTING_GROUP_FLAG_RECORD) &&
        (request->operation == 2 || request->operation == 3)) {
        /* The current Owner proxy group contains scalar settings. REMOTE_REQUEST's compact
         * record encoding obtains session/sequence from REMOTE_BASE and is shared-scope only. */
        packet->length = 0;
        return;
    }
    packet->data[0] = request->operation;
    packet->data[1] = request->target;
    packet->data[2] = request->value_type;
    packet->data[3] = request->value_length;
    sys_put_le32(request->session_id, packet->data + 4);
    sys_put_le32(request->sequence, packet->data + 8);
    sys_put_le32(request->group_id, packet->data + 12);
    sys_put_le32(request->setting_id, packet->data + 16);
    memcpy(packet->data + 20, request->value, DYNAMIC_SETTINGS_VALUE_MAX);
}

static bool decode_request(struct dynamic_setting_remote_request *request,
                           const struct nemu_link_studio_packet *packet) {
    if (!request || !packet || packet->kind != NEMU_LINK_STUDIO_REMOTE_REQUEST ||
        packet->length != ESB_REMOTE_REQUEST_SIZE ||
        packet->data[1] != DYNAMIC_SETTING_SCOPE_OWNER ||
        packet->data[3] > DYNAMIC_SETTINGS_VALUE_MAX) {
        return false;
    }
    *request = (struct dynamic_setting_remote_request){
        .operation = packet->data[0],
        .target = packet->data[1],
        .value_type = packet->data[2],
        .value_length = packet->data[3],
        .session_id = sys_get_le32(packet->data + 4),
        .sequence = sys_get_le32(packet->data + 8),
        .group_id = sys_get_le32(packet->data + 12),
        .setting_id = sys_get_le32(packet->data + 16),
    };
    memcpy(request->value, packet->data + 20, DYNAMIC_SETTINGS_VALUE_MAX);
    return request->session_id && request->sequence;
}

static void encode_response(struct nemu_link_studio_packet *packet, uint16_t transaction, int err,
                            const struct dynamic_setting_remote_response *response) {
    *packet = (struct nemu_link_studio_packet){
        .kind = NEMU_LINK_STUDIO_REMOTE_RESPONSE,
        .transaction = transaction,
        .length = ESB_REMOTE_RESPONSE_SIZE,
    };
    sys_put_le16((uint16_t)(int16_t)err, packet->data);
    if (!response) {
        return;
    }
    packet->data[2] = response->value_type;
    packet->data[3] = response->value_length;
    packet->data[4] = response->origin;
    packet->data[5] = response->dirty;
    packet->data[6] = response->persisted;
    packet->data[7] = response->schema_known;
    sys_put_le32(response->generation, packet->data + 8);
    memcpy(packet->data + 12, response->value, DYNAMIC_SETTINGS_VALUE_MAX);
}

static void make_schema_packets(uint16_t transaction,
                                const struct dynamic_setting_remote_response *response,
                                struct nemu_link_studio_packet packets[2]) {
    for (uint8_t i = 0; i < 2; i++) {
        uint8_t offset = i ? 24 : 0;
        uint8_t length = i ? 8 : 24;
        packets[i] = (struct nemu_link_studio_packet){
            .kind = NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH,
            .transaction = transaction,
            .offset = offset,
            .length = length,
        };
        memcpy(packets[i].data, response->schema_sha256 + offset, length);
    }
}

static bool decode_response(struct dynamic_setting_remote_response *response,
                            const uint8_t bytes[ESB_REMOTE_RESPONSE_SIZE]) {
    if (!response || bytes[3] > DYNAMIC_SETTINGS_VALUE_MAX ||
        bytes[4] > DYNAMIC_SETTING_ORIGIN_TEMP || bytes[7] > 1) {
        return false;
    }
    *response = (struct dynamic_setting_remote_response){
        .value_type = bytes[2],
        .value_length = bytes[3],
        .origin = bytes[4],
        .dirty = bytes[5],
        .persisted = bytes[6],
        .schema_known = bytes[7],
        .generation = sys_get_le32(bytes + 8),
    };
    memcpy(response->value, bytes + 12, DYNAMIC_SETTINGS_VALUE_MAX);
    if (response->schema_known) {
        memcpy(response->schema_sha256, client_schema_hash, sizeof(response->schema_sha256));
    }
    return true;
}

static void client_finish_locked(int err) {
    client_error = err;
    atomic_set(&client_last_error, err < 0 ? -err : 0);
    k_sem_give(&client_done);
}

static int server_send(const struct nemu_link_studio_route *route,
                       const struct nemu_link_studio_packet *packet) {
    int err = -EIO;
    for (uint8_t attempt = 0; attempt < 8; attempt++) {
        err = nemu_link_link_settings_esb_reply(route, packet);
        if (!err || (err != -EAGAIN && err != -EBUSY && err != -ENOMSG)) {
            break;
        }
        k_sleep(K_MSEC(5));
    }
    if (err) {
        atomic_inc(&server_response_send_errors);
        atomic_set(&server_last_send_error, -err);
    } else {
        atomic_inc(&server_response_packets);
    }
    return err;
}

static void server_process_job(const struct server_job *job) {
    uint16_t transaction = job->packet.transaction;
    uint8_t request_bytes[ESB_REMOTE_REQUEST_SIZE];
    memcpy(request_bytes, job->packet.data, sizeof(request_bytes));
    struct nemu_link_studio_packet response_packet;
    struct nemu_link_studio_packet schema_packets[2] = {0};
    struct dynamic_setting_remote_request request;
    struct dynamic_setting_remote_response response = {0};
    int err = -EINVAL;

    k_mutex_lock(&server_lock, K_FOREVER);
    if (transaction == server_last_transaction &&
        nemu_link_studio_route_same_session(&job->route, &server_last_route) &&
        !memcmp(request_bytes, server_last_request, sizeof(request_bytes))) {
        response_packet = server_last_response;
        memcpy(schema_packets, server_last_schema, sizeof(schema_packets));
        k_mutex_unlock(&server_lock);
        (void)server_send(&job->route, &response_packet);
        if (schema_packets[0].transaction) {
            (void)server_send(&job->route, &schema_packets[0]);
            (void)server_send(&job->route, &schema_packets[1]);
        }
        return;
    }

    bool new_session = false;
    if (decode_request(&request, &job->packet)) {
        new_session = request.session_id != server_last_session ||
                      !nemu_link_studio_route_same_session(&job->route, &server_last_route);
        if ((!new_session && !sequence_newer(request.sequence, server_last_sequence)) ||
            (!new_session && server_last_transaction &&
             !nemu_link_studio_transaction_newer(transaction, server_last_transaction))) {
            err = -ESTALE;
        } else {
            err = nemu_link_settings_relay_dispatch(&request, &response);
            if (!err) {
                server_last_session = request.session_id;
                server_last_sequence = request.sequence;
            }
        }
    }
    encode_response(&response_packet, transaction, err, &response);
    if (!err && response.schema_known) {
        make_schema_packets(transaction, &response, schema_packets);
    }
    memcpy(server_last_request, request_bytes, sizeof(server_last_request));
    server_last_response = response_packet;
    memcpy(server_last_schema, schema_packets, sizeof(server_last_schema));
    server_last_transaction = transaction;
    server_last_route = job->route;
    k_mutex_unlock(&server_lock);

    if (err) {
        atomic_inc(&server_dispatch_errors);
        atomic_set(&server_last_dispatch_error, -err);
    } else {
        atomic_inc(&server_dispatch_ok);
        atomic_set(&server_last_dispatch_error, 0);
    }
    (void)server_send(&job->route, &response_packet);
    if (schema_packets[0].transaction) {
        (void)server_send(&job->route, &schema_packets[0]);
        (void)server_send(&job->route, &schema_packets[1]);
    }
}

static void server_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    while (true) {
        struct server_job job;
        if (!k_msgq_get(&server_jobs, &job, K_FOREVER)) {
            server_process_job(&job);
        }
    }
}

int nemu_link_settings_esb_receive_request(const struct nemu_link_studio_packet *packet,
                                           const struct nemu_link_studio_route *route) {
    if (!packet || !route || route->local_side != NEMU_LINK_STUDIO_KEYBOARD || route->source >= 2 ||
        packet->kind != NEMU_LINK_STUDIO_REMOTE_REQUEST || !nemu_link_studio_packet_valid(packet)) {
        atomic_inc(&server_request_drops);
        return -EINVAL;
    }
    struct dynamic_setting_remote_request request;
    if (!decode_request(&request, packet)) {
        atomic_inc(&server_request_drops);
        return -EINVAL;
    }
    const struct dynamic_setting_group_descriptor *group =
        request.setting_id ? dynamic_settings_find_setting_group(request.setting_id)
                           : dynamic_settings_find_group(request.group_id);
    if (!group || group->scope != DYNAMIC_SETTING_SCOPE_OWNER ||
        (group->flags & DYNAMIC_SETTING_GROUP_FLAG_PROXY)) {
        /* A REMOTE_REQUEST addressed here is explicitly an Owner operation. */
        atomic_inc(&server_request_drops);
        return -EACCES;
    }
    atomic_inc(&server_request_packets);
    struct server_job job = {.packet = *packet, .route = *route};
    int err = k_msgq_put(&server_jobs, &job, K_NO_WAIT);
    if (err) {
        atomic_inc(&server_queue_drops);
        return -ENOSPC;
    }
    atomic_inc(&server_requests_queued);
    return 0;
}

int nemu_link_settings_esb_receive_response(const struct nemu_link_studio_packet *packet,
                                            uint8_t source) {
    if (!packet || !nemu_link_studio_packet_valid(packet)) {
        atomic_inc(&client_response_drops);
        return -EINVAL;
    }
    k_mutex_lock(&client_lock, K_FOREVER);
    if (!client_active || source != client_target_source ||
        packet->transaction != client_transaction) {
        atomic_inc(&client_response_drops);
        k_mutex_unlock(&client_lock);
        return -ESTALE;
    }
    if (packet->kind == NEMU_LINK_STUDIO_REMOTE_RESPONSE) {
        if (packet->length != ESB_REMOTE_RESPONSE_SIZE) {
            atomic_inc(&client_response_drops);
            k_mutex_unlock(&client_lock);
            return -EINVAL;
        }
        memcpy(client_response, packet->data, sizeof(client_response));
        client_response_received = true;
        client_schema_expected = packet->data[7] != 0;
        int err = (int16_t)sys_get_le16(client_response);
        if (err || !client_schema_expected || client_schema_received == 0x03) {
            client_finish_locked(err);
        }
    } else if (packet->kind == NEMU_LINK_STUDIO_OWNER_SCHEMA_HASH) {
        uint8_t part = packet->offset ? 1 : 0;
        uint8_t bit = BIT(part);
        if ((client_schema_received & bit) &&
            memcmp(client_schema_hash + packet->offset, packet->data, packet->length)) {
            client_schema_received = 0;
        }
        memcpy(client_schema_hash + packet->offset, packet->data, packet->length);
        client_schema_received |= bit;
        if (client_response_received && client_schema_expected && client_schema_received == 0x03) {
            client_finish_locked((int16_t)sys_get_le16(client_response));
        }
    } else {
        atomic_inc(&client_response_drops);
        k_mutex_unlock(&client_lock);
        return -EINVAL;
    }
    atomic_inc(&client_response_packets);
    k_mutex_unlock(&client_lock);
    return 0;
}

void nemu_link_settings_esb_note_link_rx_drop(bool response,
                                              enum nemu_link_settings_esb_rx_drop_reason reason) {
    atomic_t *counter = NULL;
    if (response) {
        switch (reason) {
        case NEMU_LINK_SETTINGS_ESB_RX_DROP_ROUTE:
            counter = &client_route_drops;
            break;
        case NEMU_LINK_SETTINGS_ESB_RX_DROP_EXPIRED:
            counter = &client_expired_drops;
            break;
        case NEMU_LINK_SETTINGS_ESB_RX_DROP_DECODE:
            counter = &client_decode_drops;
            break;
        }
    } else {
        switch (reason) {
        case NEMU_LINK_SETTINGS_ESB_RX_DROP_ROUTE:
            counter = &server_route_drops;
            break;
        case NEMU_LINK_SETTINGS_ESB_RX_DROP_EXPIRED:
            counter = &server_expired_drops;
            break;
        case NEMU_LINK_SETTINGS_ESB_RX_DROP_DECODE:
            counter = &server_decode_drops;
            break;
        }
    }
    if (counter) {
        atomic_inc(counter);
    }
}

int nemu_link_settings_esb_relay_call(const struct dynamic_setting_remote_request *request,
                                      struct dynamic_setting_remote_response *response) {
    if (!request || !response || request->target != DYNAMIC_SETTING_SCOPE_OWNER ||
        request->value_length > DYNAMIC_SETTINGS_VALUE_MAX || !request->session_id ||
        !request->sequence || request->target_source >= 2) {
        return -EINVAL;
    }
    atomic_inc(&client_calls);
    k_mutex_lock(&client_call_lock, K_FOREVER);
    k_mutex_lock(&client_lock, K_FOREVER);
    client_transaction++;
    if (!client_transaction) {
        client_transaction = 1;
    }
    encode_request(&client_request, request);
    if (!client_request.length) {
        k_mutex_unlock(&client_lock);
        k_mutex_unlock(&client_call_lock);
        return -ENOTSUP;
    }
    client_target_source = request->target_source;
    client_response_received = false;
    client_schema_expected = false;
    client_schema_received = 0;
    memset(client_schema_hash, 0, sizeof(client_schema_hash));
    memset(client_response, 0, sizeof(client_response));
    client_active = true;
    client_error = -ETIMEDOUT;
    k_sem_reset(&client_done);
    k_mutex_unlock(&client_lock);

    uint64_t deadline = k_uptime_get() + ESB_REMOTE_TIMEOUT_MS;
    int result = -ETIMEDOUT;
    do {
        int err = nemu_link_link_settings_esb_send(&client_request, client_target_source);
        if (err) {
            atomic_inc(&client_send_errors);
        } else {
            atomic_inc(&client_request_packets);
        }
        if (!k_sem_take(&client_done, K_MSEC(ESB_REMOTE_RETRY_MS))) {
            k_mutex_lock(&client_lock, K_FOREVER);
            result = client_error;
            if (!result && !decode_response(response, client_response)) {
                result = -EINVAL;
            }
            client_active = false;
            k_mutex_unlock(&client_lock);
            k_mutex_unlock(&client_call_lock);
            return result;
        }
    } while (k_uptime_get() < deadline);

    atomic_inc(&client_timeouts);
    atomic_set(&client_last_error, ETIMEDOUT);
    k_mutex_lock(&client_lock, K_FOREVER);
    client_active = false;
    k_mutex_unlock(&client_lock);
    k_mutex_unlock(&client_call_lock);
    return -ETIMEDOUT;
}

void nemu_link_settings_esb_diag_log(void) {
    LOG_INF("settings ESB client calls=%ld req_packets=%ld send_err=%ld rx_packets=%ld rx_drop=%ld "
            "timeout=%ld last_err=%ld route_drop=%ld expired=%ld decode=%ld",
            (long)atomic_get(&client_calls), (long)atomic_get(&client_request_packets),
            (long)atomic_get(&client_send_errors), (long)atomic_get(&client_response_packets),
            (long)atomic_get(&client_response_drops), (long)atomic_get(&client_timeouts),
            (long)atomic_get(&client_last_error), (long)atomic_get(&client_route_drops),
            (long)atomic_get(&client_expired_drops), (long)atomic_get(&client_decode_drops));
    LOG_INF("settings ESB server req_packets=%ld req_drop=%ld queued=%ld queue_drop=%ld "
            "dispatch_ok=%ld dispatch_err=%ld last_dispatch=%ld reply_packets=%ld reply_err=%ld "
            "last_reply=%ld route_drop=%ld expired=%ld decode=%ld",
            (long)atomic_get(&server_request_packets), (long)atomic_get(&server_request_drops),
            (long)atomic_get(&server_requests_queued), (long)atomic_get(&server_queue_drops),
            (long)atomic_get(&server_dispatch_ok), (long)atomic_get(&server_dispatch_errors),
            (long)atomic_get(&server_last_dispatch_error),
            (long)atomic_get(&server_response_packets),
            (long)atomic_get(&server_response_send_errors),
            (long)atomic_get(&server_last_send_error), (long)atomic_get(&server_route_drops),
            (long)atomic_get(&server_expired_drops), (long)atomic_get(&server_decode_drops));
}

static int server_init(void) {
    k_thread_create(&server_thread, server_stack, K_THREAD_STACK_SIZEOF(server_stack),
                    server_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
    k_thread_name_set(&server_thread, "settings_esb");
    return 0;
}
SYS_INIT(server_init, APPLICATION, 99);
