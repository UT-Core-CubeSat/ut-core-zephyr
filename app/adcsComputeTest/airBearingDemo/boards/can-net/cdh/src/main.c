//standard includes, proto.h is our self written CAN header file used for abstraction
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "can_proto.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/time_units.h>  // for k_uptime_get() for the latency test

//global variables for latency test
static int64_t ping_sent_time = 0;
static bool latency_test_active = false;


//init the LED as a an object you can interact with later from the deivcetree
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

//used for logging
LOG_MODULE_REGISTER(cdh, LOG_LEVEL_INF);

/* === App config === */
#define NODE_ID     0x1 //used as an identifier on the CAN BUS
#define DST_ME      NODE_ID // lets us use DST_ME instead of NODE_ID later in the code
#define PRIO_LOW    3 //our CAN priority 
#define CLS_CORE    0 //CAN message class

//our CAN BUS queue, holds incoming messages
CAN_MSGQ_DEFINE(rxq, 16);

//pointer to our CAN driver?
static const struct device *can_dev;

//CAN initialization stops CAN (if running), sets it to normal mode, and restarts it.
static int can_init_normal(void) {
  (void)can_stop(can_dev);
  (void)can_set_mode(can_dev, CAN_MODE_NORMAL);
  return can_start(can_dev);
}

//takes in dst, op code, and 6 bytes of data , sends CAN messages
static void send_simple(uint8_t dst, uint8_t op,
                        uint8_t b2,uint8_t b3,uint8_t b4,
                        uint8_t b5,uint8_t b6,uint8_t b7)
{
  struct can_frame f = {0};
  f.id = CAN_ID(PRIO_LOW, dst, CLS_CORE); //builds a CAN ID (CAN_ID macro from proto.h)
  can_fill_payload(&f, NODE_ID, op, b2,b3,b4,b5,b6,b7); //Fill CAN data bytes (from proto.h)
  (void)can_send(can_dev, &f, K_MSEC(200), NULL, NULL); // send CAN msg 
}

void main(void)
{
  can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)); //retrieves the CAN device defined in the board’s Device Tree
  //check if CAN is ready
  if (!device_is_ready(can_dev) || can_init_normal()) {
    LOG_ERR("CAN not ready");
    return;
  }

  /* Filters for CAN messages: to me, broadcast */
  struct can_filter to_me = { .id = CAN_DST(DST_ME), .mask = CAN_DST_MASK, .flags = 0 };
  struct can_filter bcast = { .id = CAN_DST(CAN_BROADCAST), .mask = CAN_DST_MASK, .flags = 0 };
  //if messages get through filters they end up in the rx queue
  can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
  can_add_rx_filter_msgq(can_dev, &rxq, &bcast);


//verifies LED GPIO port is valid.
  if (!device_is_ready(led.port)) { 
    LOG_ERR("LED not ready"); 
    return; 
  }
//configures it as an output, initially off.
  gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);


  int led_state = 0; //tracks if LED is on or off (unused)

  /* Heartbeat */
  int64_t hb_t0 = 0; //stores last heartbeat timestamp (for 1-second intervals).

  while (1) {
    /* periodic CDH heartbeat on broadcast */
    int64_t now = k_uptime_get();
    if (now - hb_t0 >= 1000) {
      hb_t0 = now;
      send_simple(CAN_BROADCAST, OP_HEARTBEAT, 0,0,0,0,0,0);
    }

    /* handle incoming */
    struct can_frame rx;
    if (k_msgq_get(&rxq, &rx, K_MSEC(100)) == 0) { //if a message is detected within 100ms process it
      uint8_t src = rx.data[0]; //extract src from CAN MSG
      uint8_t op  = rx.data[1]; //extract OP code from CAN MSG

      //based on OP code recevied respond appropriately
      switch (op) {

        //if ping recieved, respond pong
        case OP_PING:
          LOG_INF("PING from 0x%x", src);
          send_simple(src, OP_PONG, 0,0,0,0,0,0);
          break;

         //if pong received (response to our latency test)
        case OP_PONG:
          if (latency_test_active) {
            int64_t rtt = k_uptime_get() - ping_sent_time;
            LOG_INF("Latency test result: round-trip = %lld ms", rtt);
            latency_test_active = false;
          } else {
            LOG_INF("Unexpected PONG from 0x%x", src);
          }
          break;


        //if button press received, toggle LED and run latency test
        case OP_BUTTON: {
            /* payload[2] = edge: 1=pressed, 0=released (from EPS) */
            uint8_t edge = rx.data[2];
            static bool led_on;
            if (edge == 1) {                     // toggle only on press
                led_on = !led_on;
                gpio_pin_set_dt(&led, led_on ? 1 : 0);
                LOG_INF("CDH LED -> %s", led_on ? "ON" : "OFF");
                /* tell the sender (EPS) to mirror our LED state */
                send_simple(src, OP_SET_LED, led_on ? 1 : 0, 0,0,0,0,0);

                /* ---- Start latency test ---- */
                latency_test_active = true;
                ping_sent_time = k_uptime_get();
                send_simple(src, OP_PING, 0,0,0,0,0,0);
                LOG_INF("Latency test started: sent PING to 0x%x", src);

            } else {
                LOG_INF("Button release from 0x%x (ignored for toggle)", src);
            }
            break;
        }

        //by default just log that a heartbeat was recieved
        default:
          LOG_INF("RX op=0x%02x from 0x%x", op, src);
          break;
      }
    }
  }
}
