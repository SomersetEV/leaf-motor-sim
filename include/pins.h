// LilyGo T-CAN485 board pins. See docs/SIMULATOR_PLAN.md section 4.
#pragma once

// Drive high to power the CAN transceiver supply.
#define PIN_5V_EN 16

// Plan section 12, V4: settled by Ben (6 Oct 2026). Copied from his working
// telematics firmware, LilyGo-Zombie-telematics/main/can_handler.h.
// The plan and LilyGO's example config.h list them the other way round
// (TX 26 / RX 27); the telematics firmware is proven on this hardware.
#define CAN_TX_PIN 27
#define CAN_RX_PIN 26

// Drive low to take the transceiver out of standby (TX is disabled when high).
#define CAN_SE_PIN 23

// SPI microSD
#define SD_MISO_PIN 2
#define SD_MOSI_PIN 15
#define SD_SCLK_PIN 14
#define SD_CS_PIN 13

// Status LED
#define WS2812_PIN 4
