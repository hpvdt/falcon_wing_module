#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "onewire.hpp"
#include "onewireConfig.hpp"

/**
 * \brief Handles potential requests from the one wire bus
 * 
 * \note Meant to be an interrupt
 * \warning Blocks for the entirety of a transmission to host if responding
 */
void ow_input_handler(); // Since it is only meant to be used as an interrupt it is locally scoped

/**
 * \brief Add the checksu to a payload 
 * 
 * \param data Data to base the checksumed payload (lowest 24 bits)
 *
 * \return Payload with checksum affixed 
 */
int32_t ow_affix_checksum(int32_t data);

unsigned long ow_led_timeout_mark = 0;
const unsigned long LED_PERIOD = 30; // Period to illumniate following a scan

// Hardcode as constants to compile more optimized code
const uint8_t OW_RX = PIN_PA1;
const uint8_t OW_TX = PIN_PA2;
volatile uint8_t ow_address;
volatile int32_t ow_payload_out;
volatile int32_t ow_payload_in;
volatile int32_t ow_payload_test;
volatile bool ow_message_received_flag;
volatile bool ow_listener_flag;

volatile bool ow_testing_mode = false;

bool ow_get_testing_mode_active() {
    return ow_testing_mode;
}

int ow_setup(uint8_t address, bool isListener) {
    pinMode(OW_RX, INPUT);
    pinMode(OW_TX, OUTPUT);
    digitalWrite(OW_TX, LOW);

    // Protect special addresses
    if ((address == OW_ADDR_TEST_DISABLE) || (address == OW_ADDR_TEST_ENABLE)) return 1;

    ow_address = address;
    ow_listener_flag = isListener;

    attachInterrupt(digitalPinToInterrupt(OW_RX), ow_input_handler, CHANGE);
    CPUINT.LVL1VEC = PORTA_PORT_vect_num; // Make the OneWire interrupt top priority

    return 0;
}

void ow_input_handler() {
    static unsigned long last_edge = 0; // Store previous edge timestamp
    unsigned long present = micros();

    // Read directly from registers for max speed
    bool reading = (VPORTA.IN & digital_pin_to_bit_mask[OW_RX]) != 0;

    unsigned long delta = present - last_edge;

    static uint8_t bit_count = 0;
    static uint8_t ignore_count = 0;
    static uint32_t temp_data = 0;

    // Too short since last edge, ignore. Probably setting up the next actual edge
    if (delta < (3 * OW_PULSE_PERIOD)) return;
    last_edge = present;

    // See if the edge is late (new message or timeout)
    if (delta > (2 * OW_BIT_PERIOD)) {
        bit_count = 0;
        ignore_count = 0;
        temp_data = 0;
        return;
    }

    // Are we ignoring data edges? (From other responders)
    if (ignore_count != 0) {
        ignore_count--;
        return;
    }
    
    // Process the edge
    temp_data = (temp_data << 1) + reading;
    bit_count++;

    // Potential states to react based on node role
    bool check_address      = ow_listener_flag && (bit_count == OW_ADDRESS_WIDTH);
    bool received_response  = (!ow_listener_flag) && (bit_count == OW_DATA_WIDTH);

    if (check_address) {
        if (temp_data == ow_address) {
            ow_led_timeout_mark = millis() + LED_PERIOD;

            if (ow_testing_mode) ow_send_data(ow_payload_test, OW_DATA_WIDTH);
            else ow_send_data(ow_payload_out, OW_DATA_WIDTH);            
        }
        else if (temp_data == OW_ADDR_TEST_ENABLE) ow_testing_mode = true;
        else if (temp_data == OW_ADDR_TEST_DISABLE) ow_testing_mode = false;
        else ignore_count = OW_DATA_WIDTH; // Ignore the other device's response

        // Reset for next message
        bit_count = 0;
        temp_data = 0;
    }
    if (received_response) {
        ow_message_received_flag = true;

        // Extend sign by prefixing ones as needed prior to recording it
        if (temp_data & (1L << (OW_DATA_WIDTH - 1))) {
            ow_payload_in = temp_data | (0xFFFFFFFF << (OW_DATA_WIDTH - 1));
        }
        else ow_payload_in = temp_data; 

        // Reset for next message
        bit_count = 0;
        ignore_count = 0;
        temp_data = 0;
    }
}

bool ow_request(uint8_t target, int32_t *destination) {

    uint8_t attempts = 0;

    do {
        noInterrupts(); // Don't want it catching it's own messages
        ow_message_received_flag = false; // Clear received flag

        // Pull line down for a half period to get attention of all devices
        digitalWrite(OW_TX, HIGH);
        delayMicroseconds(OW_PULSE_PERIOD);

        // Send out address
        ow_send_data(target, OW_ADDRESS_WIDTH);

        // Read data in from line
        unsigned long ow_timeout_mark = micros() + OW_TIMEOUT_COMMS;
        while (!ow_message_received_flag && (micros() < ow_timeout_mark)) {
            //delayMicroseconds(1000);
        }

        attempts++;
    } while (!ow_message_received_flag && (attempts < OW_NUM_ATTEMPTS));

    if (ow_message_received_flag) *(destination) = ow_payload_in; 
    //else *(destination) = 0;

    return ow_message_received_flag;
}

void ow_send_data(uint32_t data, uint8_t width) {
    noInterrupts(); // Don't want interrupts to catch outgoing message

    // Port values to set TX without changing other pins
    uint8_t pin_mask = digital_pin_to_bit_mask[OW_TX];

    uint32_t mask = 1L << (width - 1); // Needs the `1L` otherwise mask will be 16 bits wide

    for (uint8_t i = width; i > 0; i--) {

        bool current_bit = ((mask & data) != 0);
        mask = mask >> 1;

        if (current_bit == true) {
            // Falling edge for 1 bit
            PORTA.OUTSET = pin_mask;
            delayMicroseconds(OW_BIT_PERIOD - OW_PULSE_PERIOD);
            PORTA.OUTCLR = pin_mask;
            delayMicroseconds(OW_PULSE_PERIOD);
        }
        else {
            // Rising edge for 0 bit
            PORTA.OUTCLR = pin_mask;
            delayMicroseconds(OW_BIT_PERIOD - OW_PULSE_PERIOD);
            PORTA.OUTSET = pin_mask;
            delayMicroseconds(OW_PULSE_PERIOD);
        }
    }

    digitalWriteFast(OW_TX, LOW); // Release line
    interrupts();
}

void ow_set_payload(int32_t new_payload) {

    new_payload = ow_affix_checksum(new_payload);

    noInterrupts();
    ow_payload_out = new_payload;
    interrupts();
}

void ow_update_test_data() {
    uint32_t half_0 = rand();
    uint32_t half_1 = rand();

    int32_t result = half_0 + (half_1 << 15);

    result = result & 0x00FFFFF0;
    result |= ow_address; // Add explicit address for better tracking

    ow_payload_test = ow_affix_checksum(result);
}

int32_t ow_affix_checksum(int32_t data) {
    uint8_t checksum = ow_address; // Always start checksum with address to ensure origin
    checksum += data & 0xFF;
    checksum += (data >> 8) & 0xFF;
    checksum += (data >> 16) & 0xFF;

    data = data & 0x00FFFFFFL;

    uint32_t checksum_placement = (uint32_t)(checksum) << 24;

    data = data | checksum_placement;

    return data;
}