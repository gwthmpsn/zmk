#ifdef CONFIG_PMW3360
#define DT_DRV_COMPAT pixart_pmw3360
#include <pixart/pmw3360/pmw3360.h>
#elif defined(CONFIG_PAW3395)
#define DT_DRV_COMPAT pixart_paw3395
#include <pixart/paw3395/paw3395.h>
#endif

#include <logging/log.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/mouse.h>
#include <dt-bindings/zmk/mouse.h>
#include <math.h>

#include <zmk/event_manager.h>
#include <zmk/events/endpoint_selection_changed.h>
#include <sys/atomic.h>

#define SCROLL_DIV_FACTOR 5
/* #define SCROLL_LAYER_INDEX 4 */
#define SCROLL_LAYER_INDEX                                                                         \
    COND_CODE_0(DT_INST_NODE_HAS_PROP(0, scroll_layer), (255), (DT_INST_PROP(0, scroll_layer)))

#define abs(n) ((n < 0) ? (n)*(-1) : (n))

static int polling_count = 0;
static int max_poll_count = 0;
static int polling_interval = 0;
#define BLE_POLL_COUNT 20
#define BLE_POLL_INTERVAL 15 // in ms
#define USB_POLL_COUNT 20
#define USB_POLL_INTERVAL 15 // in ms
#define TRACKBALL_LAYER_THRESHOLD 3
#define SENSOR_VALUE_MULTIPLIER 0.9

LOG_MODULE_REGISTER(trackball, CONFIG_SENSOR_LOG_LEVEL);

ATOMIC_DEFINE(timer_set_bit, 1);

void deactivate_mouse_layer(struct k_timer *timer) {
    zmk_keymap_layer_deactivate(CONFIG_MOUSE_LAYER_INDEX);
    atomic_clear_bit(timer_set_bit, 1);
}
K_TIMER_DEFINE(mouse_layer_timer, deactivate_mouse_layer, NULL);

// polling work
static void trackball_poll_handler(struct k_work *work) {
    // get the device pointer
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, poll_work);
    const struct device *dev = data->dev;

    // fetch dx and dy from sensor and save them into pixart_data structure
    int ret = sensor_sample_fetch(dev);
    if (ret < 0) {
        LOG_ERR("fetch: %d", ret);
        return;
    }

    // remaining scroll from last update
    static int16_t scroll_ver_rem = 0, scroll_hor_rem = 0;
    if (data->x != 0 || data->y != 0) {
        // process the updated position and send to host
        zmk_hid_mouse_movement_set(0, 0);
        zmk_hid_mouse_scroll_set(0, 0);

        data->x *= SENSOR_VALUE_MULTIPLIER;
        data->y = -(SENSOR_VALUE_MULTIPLIER*data->y);
        if (abs(data->x) > TRACKBALL_LAYER_THRESHOLD || abs(data->y) > TRACKBALL_LAYER_THRESHOLD) {
            if (!zmk_keymap_layer_active(CONFIG_MOUSE_LAYER_INDEX)) { // If the MOUSE_LAYER is NOT currently active
				if (!atomic_test_and_set_bit(timer_set_bit, 1)) {	
					zmk_keymap_layer_activate(CONFIG_MOUSE_LAYER_INDEX); // Activate MOUSE_LAYER 
					k_timer_start(&mouse_layer_timer, K_MSEC(CONFIG_MOUSE_LAYER_ACTIVE_MS), j); // for MOUSE_LAYER_ACTIVE_MS
				}
			}
        }

        if (zmk_keymap_layer_active(CONFIG_SCROLL_LAYER_INDEX)) { // lower
            const int16_t total_hor = data->x + scroll_hor_rem,
                          total_ver = -(data->y + scroll_ver_rem);
            scroll_hor_rem = total_hor % SCROLL_DIV_FACTOR;
            scroll_ver_rem = total_ver % SCROLL_DIV_FACTOR;
            zmk_hid_mouse_scroll_update(total_hor / SCROLL_DIV_FACTOR,
                                        total_ver / SCROLL_DIV_FACTOR);
        } else {
            zmk_hid_mouse_movement_update(CLAMP(data->x, INT8_MIN, INT8_MAX),
                                          CLAMP(data->y, INT8_MIN, INT8_MAX));
        }

        // send the report to host
        zmk_endpoints_send_mouse_report();
    }
}

// trigger handler
static void trackball_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    struct pixart_data *data = dev->data;

    // do not resume motion interrupt by passing-in null handler
    struct sensor_trigger trigger = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    if (sensor_trigger_set(dev, &trigger, NULL) < 0) {
        LOG_ERR("can't stop motion interrupt line");
    };

    // start the polling timer (the real work now is dispatched to a timer-based polling)
    k_timer_start(&data->poll_timer, K_NO_WAIT, K_MSEC(polling_interval));
}

// timer expiry function
void trackball_timer_expiry(struct k_timer *timer) {
    struct pixart_data *data = CONTAINER_OF(timer, struct pixart_data, poll_timer);

    // check whether reaching the polling count limit
    if (polling_count < max_poll_count) {
        // submit polling work to mouse work queue
        k_work_submit_to_queue(zmk_mouse_work_q(), &data->poll_work);

        // update status
        polling_count++;
    } else {
        // stop timer
        k_timer_stop(&data->poll_timer);
    }
}

// timer stop function
void trackball_timer_stop(struct k_timer *timer) {
    struct pixart_data *data = CONTAINER_OF(timer, struct pixart_data, poll_timer);
    const struct device *dev = data->dev;

    // reset polling count
    polling_count = 0;

    // resume motion interrupt line by setting the handler to a real object
    struct sensor_trigger trigger = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    if (sensor_trigger_set(dev, &trigger, trackball_trigger_handler) < 0) {
        LOG_ERR("can't resume motion interrupt line");
    };
}

/* Setup the trigger handler at system powerup */
// The device instance should be determined in this step. All other functions should only
// use the device poiter passed as an argument.
// In this applaication, the devcie instance 'trackball' is hard-coded as the first
// pixart_pmw3360 node in dts file and used implicitly here.
static int trackball_init() {
    LOG_INF("Init trackball_new");

    // get the sensor device instance
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
    struct pixart_data *data = dev->data;

    // setup the timer and handler function of the polling work
    k_timer_init(&data->poll_timer, trackball_timer_expiry, trackball_timer_stop);
    k_work_init(&data->poll_work, trackball_poll_handler);

    // set up the trigger handler (i.e. the entry point to other code in this file)
    struct sensor_trigger trigger = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };

    int err;
    do {
        err = sensor_trigger_set(dev, &trigger, trackball_trigger_handler);
        if (err == -EBUSY) {
            k_sleep(K_MSEC(1));
        }
    } while (err == -EBUSY);

    if (err) {
        LOG_ERR("Cannot enable trigger");
        return err;
    }

    // init polling parameters based on current endpoint
    switch (zmk_endpoints_selected()) {
#if IS_ENABLED(CONFIG_ZMK_USB)
    case ZMK_ENDPOINT_USB: {
        max_poll_count = USB_POLL_COUNT;
        polling_interval = USB_POLL_INTERVAL;
        return 0;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_USB) */

#if IS_ENABLED(CONFIG_ZMK_BLE)
    case ZMK_ENDPOINT_BLE: {
        max_poll_count = BLE_POLL_COUNT;
        polling_interval = BLE_POLL_INTERVAL;
        return 0;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_BLE) */
    default:
        LOG_ERR("Unsupported endpoint");
        return -ENOTSUP;
    }
}

SYS_INIT(trackball_init, APPLICATION, CONFIG_SENSOR_INIT_PRIORITY);

/* the following code dynamically changes poll rate based on endpoint changing */
// The code execution is driven by the zmk event system instead of zephyr interrupt routine
int trackball_endpoint_listener(const zmk_event_t *eh) {
    // update polling parameters
    switch (zmk_endpoints_selected()) {
#if IS_ENABLED(CONFIG_ZMK_USB)
    case ZMK_ENDPOINT_USB: {
        max_poll_count = USB_POLL_COUNT;
        polling_interval = USB_POLL_INTERVAL;
        break;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_USB) */

#if IS_ENABLED(CONFIG_ZMK_BLE)
    case ZMK_ENDPOINT_BLE: {
        max_poll_count = BLE_POLL_COUNT;
        polling_interval = BLE_POLL_INTERVAL;
        break;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_BLE) */
    default:
        LOG_ERR("Unsupported endpoint");
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(trackball, trackball_endpoint_listener)
ZMK_SUBSCRIPTION(trackball, zmk_endpoint_selection_changed)
