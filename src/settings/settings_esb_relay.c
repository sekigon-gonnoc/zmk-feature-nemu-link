#include <errno.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/nemu_link/settings_relay.h>
#include <zmk/nemu_link/studio_transport.h>

#define ESB_REQUEST_SIZE 36
#define ESB_RESPONSE_SIZE 58
#define ESB_FRAGMENT_SIZE NEMU_LINK_STUDIO_CHUNK_SIZE
#define ESB_RETRY_MS 500
#define ESB_TIMEOUT_MS 10000
#define ESB_SERVER_QUEUE_DEPTH 2

LOG_MODULE_REGISTER(nemu_link_settings_esb, LOG_LEVEL_INF);

static atomic_t client_calls, client_request_fragments, client_send_errors;
static atomic_t client_response_fragments, client_response_drops, client_responses_complete;
static atomic_t client_response_restarts;
static atomic_t client_timeouts, client_route_drops, client_expired_drops, client_decode_drops;
static atomic_t server_request_fragments, server_request_drops, server_requests_complete;
static atomic_t server_request_head, server_request_tail, server_request_transaction_reset;
static atomic_t server_request_session_reset;
static atomic_t server_queue_drops, server_dispatch_ok, server_dispatch_errors;
static atomic_t server_response_fragments, server_response_send_errors;
static atomic_t server_route_drops, server_expired_drops, server_decode_drops;
static atomic_t client_last_error, server_last_dispatch_error, server_last_send_error;

static K_MUTEX_DEFINE(client_lock);
static K_MUTEX_DEFINE(client_call_lock);
static K_SEM_DEFINE(client_done, 0, 1);
static struct nemu_link_studio_packet client_request[2];
static uint16_t client_transaction;
static uint8_t client_response[ESB_RESPONSE_SIZE];
static uint8_t client_response_mask;
static uint8_t client_target_source;
static int client_error;
static bool client_active;

static struct nemu_link_studio_owner_request_assembly server_rx;
static uint8_t server_last_request[ESB_REQUEST_SIZE];
static uint8_t server_last_response[ESB_RESPONSE_SIZE];
static uint16_t server_last_transaction;
static uint32_t server_last_session, server_last_sequence;
static K_MUTEX_DEFINE(server_lock);

struct server_job {
    uint16_t transaction;
    uint8_t bytes[ESB_REQUEST_SIZE];
    struct nemu_link_studio_route route;
};
K_MSGQ_DEFINE(server_jobs, sizeof(struct server_job), ESB_SERVER_QUEUE_DEPTH, 4);
K_THREAD_STACK_DEFINE(server_stack, 2048);
static struct k_thread server_thread;

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

void nemu_link_settings_esb_diag_log(void) {
    LOG_INF("settings ESB client calls=%ld req_queued=%ld send_err=%ld rx_frag=%ld rx_drop=%ld "
            "rx_restart=%ld complete=%ld timeout=%ld last_err=%ld route_drop=%ld expired=%ld "
            "decode=%ld",
            (long)atomic_get(&client_calls), (long)atomic_get(&client_request_fragments),
            (long)atomic_get(&client_send_errors), (long)atomic_get(&client_response_fragments),
            (long)atomic_get(&client_response_drops), (long)atomic_get(&client_response_restarts),
            (long)atomic_get(&client_responses_complete), (long)atomic_get(&client_timeouts),
            (long)atomic_get(&client_last_error), (long)atomic_get(&client_route_drops),
            (long)atomic_get(&client_expired_drops), (long)atomic_get(&client_decode_drops));
    LOG_INF("settings ESB server req_frag=%ld req_drop=%ld complete=%ld queue_drop=%ld "
            "head=%ld tail=%ld transaction_reset=%ld session_reset=%ld "
            "dispatch_ok=%ld dispatch_err=%ld "
            "last_dispatch=%ld reply_queued=%ld reply_err=%ld last_reply=%ld route_drop=%ld "
            "expired=%ld decode=%ld",
            (long)atomic_get(&server_request_fragments), (long)atomic_get(&server_request_drops),
            (long)atomic_get(&server_requests_complete), (long)atomic_get(&server_queue_drops),
            (long)atomic_get(&server_request_head), (long)atomic_get(&server_request_tail),
            (long)atomic_get(&server_request_transaction_reset),
            (long)atomic_get(&server_request_session_reset), (long)atomic_get(&server_dispatch_ok),
            (long)atomic_get(&server_dispatch_errors),
            (long)atomic_get(&server_last_dispatch_error),
            (long)atomic_get(&server_response_fragments),
            (long)atomic_get(&server_response_send_errors),
            (long)atomic_get(&server_last_send_error), (long)atomic_get(&server_route_drops),
            (long)atomic_get(&server_expired_drops), (long)atomic_get(&server_decode_drops));
}

static void encode_request(uint8_t bytes[ESB_REQUEST_SIZE],
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
                           const uint8_t bytes[ESB_REQUEST_SIZE]) {
    if (bytes[1] != DYNAMIC_SETTING_SCOPE_OWNER || bytes[3] > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX) {
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

static void encode_response(uint8_t bytes[ESB_RESPONSE_SIZE], int err,
                            const struct dynamic_setting_remote_response *response) {
    memset(bytes, 0, ESB_RESPONSE_SIZE);
    sys_put_le16((uint16_t)(int16_t)err, bytes);
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

static bool decode_response(struct dynamic_setting_remote_response *response,
                            const uint8_t bytes[ESB_RESPONSE_SIZE]) {
    if (bytes[3] > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX || bytes[4] > DYNAMIC_SETTING_ORIGIN_TEMP ||
        bytes[7] > 1) {
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
        .total_length = sys_get_le16(bytes + 12),
    };
    memcpy(response->value, bytes + 14, DYNAMIC_SETTINGS_REMOTE_VALUE_MAX);
    memcpy(response->schema_sha256, bytes + 26, sizeof(response->schema_sha256));
    return true;
}

static bool newer(uint32_t candidate, uint32_t current) {
    uint32_t delta = candidate - current;
    return delta && delta < UINT32_C(0x80000000);
}

static void make_response_packets(uint16_t transaction, const uint8_t bytes[ESB_RESPONSE_SIZE],
                                  struct nemu_link_studio_packet packets[3]) {
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t offset = (uint8_t)(i * ESB_FRAGMENT_SIZE);
        uint8_t amount = (uint8_t)MIN(ESB_FRAGMENT_SIZE, ESB_RESPONSE_SIZE - offset);
        packets[i] = (struct nemu_link_studio_packet){
            .kind = NEMU_LINK_STUDIO_OWNER_RPC_RESPONSE,
            .transaction = transaction,
            .offset = offset,
            .length = amount,
        };
        memcpy(packets[i].data, bytes + offset, amount);
    }
}

static void server_process_job(const struct server_job *job) {
    uint16_t transaction = job->transaction;
    const uint8_t *request_bytes = job->bytes;
    struct nemu_link_studio_packet replies[3];
    uint8_t reply_count = 0;
    uint8_t response_bytes[ESB_RESPONSE_SIZE];
    if (transaction == server_last_transaction &&
        !memcmp(request_bytes, server_last_request, ESB_REQUEST_SIZE)) {
        memcpy(response_bytes, server_last_response, sizeof(response_bytes));
    } else {
        struct dynamic_setting_remote_request request;
        struct dynamic_setting_remote_response response = {0};
        int err = -EINVAL;
        if (decode_request(&request, request_bytes)) {
            bool new_session = request.session_id != server_last_session;
            if ((!new_session && !newer(request.sequence, server_last_sequence)) ||
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
        if (err) {
            atomic_inc(&server_dispatch_errors);
            atomic_set(&server_last_dispatch_error, -err);
        } else {
            atomic_inc(&server_dispatch_ok);
            atomic_set(&server_last_dispatch_error, 0);
        }
        encode_response(server_last_response, err, &response);
        memcpy(server_last_request, request_bytes, ESB_REQUEST_SIZE);
        server_last_transaction = transaction;
        memcpy(response_bytes, server_last_response, sizeof(response_bytes));
    }
    make_response_packets(transaction, response_bytes, replies);
    reply_count = 3;
    for (uint8_t i = 0; i < reply_count; i++) {
        int err = -EIO;
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            err = nemu_link_link_settings_esb_reply(&job->route, &replies[i]);
            if (!err || (err != -EIO && err != -ETIMEDOUT)) {
                break;
            }
        }
        if (err) {
            atomic_inc(&server_response_send_errors);
            atomic_set(&server_last_send_error, -err);
        } else {
            atomic_inc(&server_response_fragments);
        }
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
        packet->kind != NEMU_LINK_STUDIO_OWNER_RPC_REQUEST ||
        !nemu_link_studio_packet_valid(packet)) {
        atomic_inc(&server_request_drops);
        return -EINVAL;
    }
    atomic_inc(&server_request_fragments);
    k_mutex_lock(&server_lock, K_FOREVER);
    if (packet->offset == 0) {
        atomic_inc(&server_request_head);
    } else {
        atomic_inc(&server_request_tail);
    }
    if (server_rx.received_mask) {
        if (packet->transaction != server_rx.transaction) {
            atomic_inc(&server_request_transaction_reset);
        }
        if (!nemu_link_studio_route_same_session(route, &server_rx.route)) {
            atomic_inc(&server_request_session_reset);
        }
    }
    uint8_t request_bytes[ESB_REQUEST_SIZE];
    if (!nemu_link_studio_owner_request_assemble(&server_rx, packet, route, request_bytes)) {
        k_mutex_unlock(&server_lock);
        return 0;
    }

    struct server_job job = {
        .transaction = packet->transaction,
        .route = server_rx.route,
    };
    atomic_inc(&server_requests_complete);
    memcpy(job.bytes, request_bytes, sizeof(job.bytes));
    int err = k_msgq_put(&server_jobs, &job, K_NO_WAIT);
    if (err) {
        atomic_inc(&server_queue_drops);
    }
    k_mutex_unlock(&server_lock);
    return err ? -ENOSPC : 0;
}

int nemu_link_settings_esb_receive_response(const struct nemu_link_studio_packet *packet,
                                            uint8_t source) {
    if (!packet || packet->kind != NEMU_LINK_STUDIO_OWNER_RPC_RESPONSE ||
        !nemu_link_studio_packet_valid(packet)) {
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
    memcpy(client_response + packet->offset, packet->data, packet->length);
    client_response_mask |= packet->offset == 0 ? 0x01 : packet->offset == 24 ? 0x02 : 0x04;
    atomic_inc(&client_response_fragments);
    if (client_response_mask == 0x07) {
        atomic_inc(&client_responses_complete);
        k_sem_give(&client_done);
    }
    k_mutex_unlock(&client_lock);
    return 0;
}

int nemu_link_settings_esb_relay_call(const struct dynamic_setting_remote_request *request,
                                      struct dynamic_setting_remote_response *response) {
    if (!request || !response || request->target != DYNAMIC_SETTING_SCOPE_OWNER ||
        request->value_length > DYNAMIC_SETTINGS_REMOTE_VALUE_MAX || !request->session_id ||
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
    uint8_t request_bytes[ESB_REQUEST_SIZE];
    encode_request(request_bytes, request);
    for (uint8_t i = 0; i < 2; i++) {
        uint8_t offset = (uint8_t)(i * ESB_FRAGMENT_SIZE);
        uint8_t amount = (uint8_t)(i ? ESB_REQUEST_SIZE - ESB_FRAGMENT_SIZE : ESB_FRAGMENT_SIZE);
        client_request[i] = (struct nemu_link_studio_packet){
            .kind = NEMU_LINK_STUDIO_OWNER_RPC_REQUEST,
            .transaction = client_transaction,
            .offset = offset,
            .length = amount,
        };
        memcpy(client_request[i].data, request_bytes + offset, amount);
    }
    client_response_mask = 0;
    client_target_source = request->target_source;
    client_active = true;
    client_error = -ETIMEDOUT;
    k_sem_reset(&client_done);
    k_mutex_unlock(&client_lock);

    uint64_t deadline = k_uptime_get() + ESB_TIMEOUT_MS;
    do {
        /* Repeat both chunks together until a reply arrives. The peer may lose
         * its partial assembly during an ESB reconnect, so retrying only the
         * tail can leave it waiting forever. The receiver treats duplicates
         * as idempotent within this transaction. */
        for (uint8_t i = 0; i < ARRAY_SIZE(client_request); i++) {
            int err = nemu_link_link_settings_esb_send(&client_request[i], client_target_source);
            if (err) {
                atomic_inc(&client_send_errors);
                /* Never enqueue the tail when the head was not accepted. */
                break;
            } else {
                atomic_inc(&client_request_fragments);
            }
        }
        if (!k_sem_take(&client_done, K_MSEC(ESB_RETRY_MS))) {
            k_mutex_lock(&client_lock, K_FOREVER);
            bool complete = client_response_mask == 0x07;
            if (complete) {
                client_error = (int16_t)sys_get_le16(client_response);
                if (!client_error && !decode_response(response, client_response)) {
                    client_error = -EINVAL;
                }
            }
            atomic_set(&client_last_error, -client_error);
            client_active = false;
            int result = client_error;
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

static int server_init(void) {
    k_thread_create(&server_thread, server_stack, K_THREAD_STACK_SIZEOF(server_stack),
                    server_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
    k_thread_name_set(&server_thread, "settings_esb");
    return 0;
}
SYS_INIT(server_init, APPLICATION, 99);
