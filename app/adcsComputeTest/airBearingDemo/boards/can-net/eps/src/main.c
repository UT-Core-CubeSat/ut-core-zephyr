//standard includes, proto.h is our self written CAN header file used for abstraction
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "can_proto.h"

/* Devicetree aliases for user button & LED */
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

//used for logging
LOG_MODULE_REGISTER(eps, LOG_LEVEL_INF);

/* App config */
#define NODE_ID     0x2//used as an identifier on the CAN BUS
#define CDH_ID      0x2 //redundant 
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

/* Button ISR */
static struct gpio_callback btn_cb;
static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
  /* Read level and push event (edge: 1 press, 0 release) */
  int level = gpio_pin_get_dt(&btn);
  send_simple(CDH_ID, OP_BUTTON, (uint8_t)(level ? 1 : 0), 0,0,0,0,0);
}

void main(void)
{
  can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)); //retrieves the CAN device defined in the board’s Device Tree
    //check if CAN is ready
  if (!device_is_ready(can_dev) || can_init_normal()) {
    LOG_ERR("CAN not ready");
    return;
  }

 //check if GPIO(Button) is ready
  if (!device_is_ready(btn.port) || !device_is_ready(led.port)) {
    LOG_ERR("GPIO not ready");
    return;
  }

   /* GPIO init */
  gpio_pin_configure_dt(&btn, GPIO_INPUT);
  gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
  gpio_init_callback(&btn_cb, btn_isr, BIT(btn.pin));
  gpio_add_callback(btn.port, &btn_cb);
  //configures it as an output, initially off.
  gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

  /* Filters for CAN messages: to me, broadcast */
  struct can_filter to_me = { .id = CAN_DST(DST_ME), .mask = CAN_DST_MASK, .flags = 0 };
  struct can_filter bcast = { .id = CAN_DST(CAN_BROADCAST), .mask = CAN_DST_MASK, .flags = 0 };
    //if messages get through filters they end up in the rx queue
  can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
  can_add_rx_filter_msgq(can_dev, &rxq, &bcast);

   /* Heartbeat */
  int64_t hb_t0 = 0; //stores last heartbeat timestamp (for 1-second intervals).

  while (1) {
    /* periodic heartbeat to CDH */
    int64_t now = k_uptime_get();
    if (now - hb_t0 >= 1000) {
      hb_t0 = now;
      send_simple(CDH_ID, OP_HEARTBEAT, 0,0,0,0,0,0);
    }

    /* handle incoming */
    struct can_frame rx;
    if (k_msgq_get(&rxq, &rx, K_MSEC(100)) == 0) { //if a message is detected within 100ms process it
      uint8_t op = rx.data[1]; //extract OP code from CAN MSG

      //based on OP code recevied respond appropriately
      switch (op) {

        //If CDH sends OP_SET_LED turn on lED and log it
        case OP_SET_LED: {
          uint8_t on = rx.data[2];
          gpio_pin_set_dt(&led, on ? 1 : 0);
          LOG_INF("LED -> %s", on ? "ON" : "OFF");
          break;
        }

        //if ping recieved, respond pong
        case OP_PING:
          send_simple(rx.data[0], OP_PONG, 0,0,0,0,0,0);
          break;

        //by default just log that a heartbeat was recieved
        default:
          LOG_INF("RX op=0x%02x", op);
          break;
      }
    }
  }
}
