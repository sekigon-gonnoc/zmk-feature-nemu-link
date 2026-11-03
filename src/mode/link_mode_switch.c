 // Runtime selector for the BLE and runtime ESB split transports.
#if defined(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
#include <zmk/nemu_link/studio_zmk.h>
#endif

#include <errno.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#endif
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
#include <zmk/nemu_link/link_timing.h>
#endif
#include <zmk/nemu_link/control.h>
#include <zmk/nemu_link/internal.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/split/bluetooth/service.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/endpoints.h>
#include <zmk/split/central.h>
#endif

LOG_MODULE_REGISTER(nemu_link_switch, LOG_LEVEL_INF);

#define MODE_SETTLE_MS 350
#define MODE_PEER_SETTLE_MS 1000

#define MODE_WORK_STACK_SIZE 2048
#define MODE_WORK_PRIORITY K_PRIO_PREEMPT(5)

static bool esb_active;
static bool ble_suspended;
static bool mode_pending;
static bool target_esb;
static K_MUTEX_DEFINE(mode_lock);
K_THREAD_STACK_DEFINE(mode_work_stack, MODE_WORK_STACK_SIZE);
static struct k_work_q mode_work_q;

static int mode_workqueue_init(void) {
    static const struct k_work_queue_config queue_config = {
#if IS_ENABLED(CONFIG_THREAD_NAME)
        .name = "dongle mode switch",
#endif
    };
    k_work_queue_start(&mode_work_q, mode_work_stack, K_THREAD_STACK_SIZEOF(mode_work_stack),
                       MODE_WORK_PRIORITY, &queue_config);
    return 0;
}
SYS_INIT(mode_workqueue_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

extern const struct zmk_split_transport_peripheral *zmk_nemu_link_peripheral_transport(void);

bool zmk_nemu_link_owns_radio(void) { return esb_active || ble_suspended; }

static int resume_ble(void) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    LOG_INF("BLE enable starting");
    int err = bt_enable(NULL);
    LOG_INF("BLE enable returned %d", err);
    if (err) {
        return err;
    }
    LOG_INF("BLE bond reload starting");
    err = settings_load_subtree("bt");
    LOG_INF("BLE bond reload returned %d", err);
    if (err) {
        LOG_ERR("BLE bond reload failed: %d", err);
    }
    ble_suspended = false;
    esb_active = false;
    LOG_INF("BLE split resume starting");
    err = zmk_nemu_link_ble_split_resume();
    LOG_INF("BLE split resume returned %d", err);
    if (err) {
        LOG_ERR("BLE split resume failed: %d", err);
    }
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    err = zmk_nemu_link_ble_host_resume();
    if (err) {
        LOG_ERR("BLE host advertising failed: %d", err);
    }
#endif
    LOG_INF("BLE peripheral transport selection starting");
    err = zmk_split_peripheral_select_first_available_transport();
    LOG_INF("BLE peripheral transport selection returned %d", err);
    if (err != -ENODEV && err) {
        LOG_ERR("BLE peripheral transport resume failed: %d", err);
        return err;
    }
    return 0;
#else
    return -ENOTSUP;
#endif
}

static void mode_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    int err = 0;
    bool target = target_esb;
    LOG_INF("Mode work started target_esb=%d esb_active=%d ble_suspended=%d", target, esb_active,
            ble_suspended);

    if (target) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK_STUDIO_SYNC)
        nemu_link_studio_release_all();
#endif
        zmk_endpoint_clear_reports();
#endif
        ble_suspended = true;
        LOG_INF("Stopping current peripheral transport");
        err = zmk_split_peripheral_select_transport(NULL);
        LOG_INF("Current peripheral transport stop returned %d", err);
        if (err) {
            LOG_ERR("BLE peripheral transport stop failed: %d", err);
            ble_suspended = false;
            goto done;
        }
#if IS_ENABLED(CONFIG_ZMK_BLE)
        LOG_INF("BLE disable starting");
        err = bt_disable();
        LOG_INF("BLE disable returned %d", err);
        if (err) {
            LOG_ERR("BLE disable failed: %d", err);
            resume_ble();
            goto done;
        }
#endif
        LOG_INF("Selecting ESB peripheral transport");
        err = zmk_split_peripheral_select_transport(zmk_nemu_link_peripheral_transport());
        LOG_INF("ESB peripheral transport selection returned %d", err);
        if (err) {
            LOG_ERR("ESB peripheral transport start failed: %d", err);
#if IS_ENABLED(CONFIG_ZMK_BLE)
            if (resume_ble()) {
                LOG_ERR("BLE recovery failed after ESB start failure");
            }
#endif
            goto done;
        }
        esb_active = true;
        ble_suspended = false;
        LOG_INF("ESB split peripheral active, side %u peripheral_id %u",
                CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID, CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID);
    } else {
        LOG_INF("Stopping ESB peripheral transport");
        /* set_enabled(false) stops the runtime ESB radio and drops its TX queue. */
        err = zmk_split_peripheral_select_transport(NULL);
        if (err) {
            LOG_ERR("ESB peripheral transport stop failed: %d", err);
            goto done;
        }
#if IS_ENABLED(CONFIG_ZMK_BLE)
        err = resume_ble();
        if (err) {
            LOG_ERR("BLE resume failed: %d", err);
            /* Keep the BLE controller down if its initialization failed. */
            goto done;
        }
        LOG_INF("BLE split active, side %u", CONFIG_ZMK_NEMU_LINK_PERIPHERAL_ID);
#else
        LOG_ERR("Cannot return to BLE: this image has no BLE role");
#endif
    }

done:
    k_mutex_lock(&mode_lock, K_FOREVER);
    mode_pending = false;
    k_mutex_unlock(&mode_lock);
    nemu_link_notify_mode_changed(zmk_nemu_link_owns_radio(), err);
}
K_WORK_DELAYABLE_DEFINE(mode_work, mode_work_handler);

static int request_mode(bool target, uint32_t settle_ms) {
    k_mutex_lock(&mode_lock, K_FOREVER);
    if (mode_pending || esb_active == target) {
        k_mutex_unlock(&mode_lock);
        return 0;
    }
    mode_pending = true;
    target_esb = target;
    LOG_INF("Mode switch requested target_esb=%d", target);
    k_mutex_unlock(&mode_lock);

    nemu_link_notify_mode_requested(target);
    /* Bluetooth shutdown/startup can block. Keep key scanning and ZMK event
     * processing on the system work queue responsive during the transition. */
    int err = k_work_reschedule_for_queue(&mode_work_q, &mode_work, K_MSEC(settle_ms));
    if (err < 0) {
        k_mutex_lock(&mode_lock, K_FOREVER);
        mode_pending = false;
        k_mutex_unlock(&mode_lock);
    }
    return err < 0 ? err : 0;
}

int zmk_nemu_link_request_mode(bool target) {
    uint32_t settle_ms = target ? MODE_SETTLE_MS : NEMU_LINK_LINK_MODE_SETTLE_MS;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    /* The central sends the peer's BLE-to-ESB request before this local
     * request. Let the peer stop BLE and start ESB before we tear down BLE. */
    if (target) {
        settle_ms += MODE_PEER_SETTLE_MS;
    }
#endif
    return request_mode(target, settle_ms);
}
int nemu_link_recover_ble(void) { return request_mode(false, 0); }

static int mode_boot(void) {
    if (IS_ENABLED(CONFIG_ZMK_NEMU_LINK_BOOT_ESB)) {
        return zmk_nemu_link_request_mode(true);
    }
    return 0;
}
SYS_INIT(mode_boot, APPLICATION, 99);
