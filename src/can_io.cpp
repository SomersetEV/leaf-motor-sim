#include "can_io.h"

#include <Arduino.h>
#include <driver/twai.h>
#include <string.h>

#include "cmd_latch.h"
#include "config.h"
#include "pins.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static simcore::CmdLatch s_latch(RX_TIMEOUT_MS);
static CanStats s_stats = {};
static bool s_bus_error_seen = false;

static uint32_t now_ms() {
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
}

bool can_init() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);  // transceiver supply on
    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);  // transceiver out of standby
    delay(10);                      // let the 5 V rail settle

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    g.tx_queue_len = CAN_TX_QUEUE_LEN;
    g.rx_queue_len = CAN_RX_QUEUE_LEN;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                       TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    return twai_start() == ESP_OK;
}

static void handle_frame(const twai_message_t &m) {
    uint32_t t = now_ms();
    portENTER_CRITICAL(&s_mux);
    if (m.extd || m.rtr) {
        s_stats.rx_other++;
    } else if (m.identifier == ID_LEAF_TORQUE_CMD) {
        s_latch.accept(m.data, m.data_length_code, t, K_T_DEFAULT);
    } else if (m.identifier == ID_LEAF_VCM_GEAR) {
        s_stats.rx_11a++;
        if (m.data_length_code >= 2) {
            s_stats.gear_11a  = m.data[0];
            s_stats.onoff_11a = m.data[1];
        }
    } else if (m.identifier == ID_LEAF_VCM_ALIVE) {
        s_stats.rx_50b++;
    } else {
        s_stats.rx_other++;
    }
    portEXIT_CRITICAL(&s_mux);
}

static void handle_alerts(uint32_t alerts) {
    if (alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS)) {
        uint32_t t = now_ms();
        portENTER_CRITICAL(&s_mux);
        s_bus_error_seen = true;
        s_stats.last_bus_error_ms = t;
        if (alerts & TWAI_ALERT_BUS_OFF) s_stats.bus_off_count++;
        portEXIT_CRITICAL(&s_mux);
    }
    if (alerts & TWAI_ALERT_BUS_OFF) twai_initiate_recovery();
    if (alerts & TWAI_ALERT_BUS_RECOVERED) twai_start();
}

static void can_rx_task(void *) {
    twai_message_t m;
    uint32_t alerts;
    for (;;) {
        // Short timeout so alerts are checked even on a silent bus.
        if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) handle_frame(m);
        if (twai_read_alerts(&alerts, 0) == ESP_OK) handle_alerts(alerts);
    }
}

void can_start_rx_task() {
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", TASK_STACK_BYTES, nullptr,
                            CAN_RX_TASK_PRIO, nullptr, CAN_RX_TASK_CORE);
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t dlc) {
    twai_message_t m = {};
    m.identifier = id;
    m.data_length_code = dlc;
    memcpy(m.data, data, dlc);
    bool ok = twai_transmit(&m, 0) == ESP_OK;
    portENTER_CRITICAL(&s_mux);
    if (ok) s_stats.tx_queued++;
    else    s_stats.tx_failed++;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

bool can_get_cmd(uint32_t now, leafcodec::Cmd1D4 &out) {
    portENTER_CRITICAL(&s_mux);
    bool fresh = s_latch.current(now, out);
    portEXIT_CRITICAL(&s_mux);
    return fresh;
}

uint32_t can_rejected_1d4() {
    portENTER_CRITICAL(&s_mux);
    uint32_t n = s_latch.rejected();
    portEXIT_CRITICAL(&s_mux);
    return n;
}

bool can_bus_error_within(uint32_t now, uint32_t window_ms) {
    portENTER_CRITICAL(&s_mux);
    bool recent = s_bus_error_seen && (uint32_t)(now - s_stats.last_bus_error_ms) < window_ms;
    portEXIT_CRITICAL(&s_mux);
    return recent;
}

CanStats can_get_stats() {
    portENTER_CRITICAL(&s_mux);
    CanStats s = s_stats;
    s.rx_1d4  = s_latch.accepted();
    s.rej_1d4 = s_latch.rejected();
    if (!s_bus_error_seen) s.last_bus_error_ms = 0;
    portEXIT_CRITICAL(&s_mux);

    twai_status_info_t info;
    if (twai_get_status_info(&info) == ESP_OK) {
        s.tec   = info.tx_error_counter;
        s.rec   = info.rx_error_counter;
        s.state = (uint8_t)info.state;
    }
    return s;
}
