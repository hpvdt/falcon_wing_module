#ifndef ONEWIRE_CONFIG_HEADER_H
#define ONEWIRE_CONFIG_HEADER_H

#include <stdint.h>

extern const uint_fast8_t OW_BIT_PERIOD;        // Period for each bit in us (needs to be at least thrice `OW_PULSE_PERIOD`)
extern const uint_fast8_t OW_PULSE_PERIOD;      // Minimum period the pulse is held for a bit on one wire bus
extern const uint_fast8_t OW_ADDRESS_WIDTH;     // Number of bits for device addresses on one wire bus
extern const uint_fast8_t OW_DATA_WIDTH;        // Width of data response on one wire bus

extern const uint_fast8_t OW_NUM_ATTEMPTS;      // Maximum number of automatic retries in one wire exchange
extern const uint_fast16_t OW_TIMEOUT_COMMS;    // Timeout after sending a request in us for one wire protocol

extern const uint_fast8_t OW_ADDR_TEST_ENABLE;  // Address/command to switch into testing mode
extern const uint_fast8_t OW_ADDR_TEST_DISABLE; // Address command to switch into normal operation

#endif