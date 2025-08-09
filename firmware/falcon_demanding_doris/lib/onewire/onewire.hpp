#ifndef ONEWIRE_HEADER_H
#define ONEWIRE_HEADER_H

#include <Arduino.h>

extern unsigned long ow_led_timeout_mark; // Mark for if the activity LED should be on

/**
 * \brief Setups up a one wire interface
 * 
 * \param address Address for device
 * \param isListener Set to true if device is listener. If a listener, it will set the handler interrupt routine up.
 * 
 * \return - `0` if everything went well
 * \return - `1` if the address provided is invalid
 */
int ow_setup(uint8_t address, bool isListener);

/**
 * \brief Requests and receives data from device on the one wire bus
 * 
 * \warning Leaves interrupts enabled once completed
 * 
 * \param target Address of the unit of interest
 * \param destination Pointer to location to store response from target
 * 
 * \return True if message was received
 */
bool ow_request(uint8_t targetAdd, int32_t *destination);

/**
 * \brief Send data over one wire interface
 * 
 * \note Shifts data out MSB first. Positive dominant edges are for 1.
 * 
 * \warning Leaves interrupts enabled on completion
 * 
 * \param data Payload to send
 * \param width The width of the data to send in bits
 */
void ow_send_data(uint32_t data, uint8_t width);

/**
 * \brief Set the one wire response payload
 * 
 * \param new_payload What to repsond with next one wire query
 */
void ow_set_payload(int32_t newPayload);

/**
 * \brief Generates a new test value for the one wire protocol
 * 
 * \note The number is a random number with a deterministic checksum
 * 
 * \warning This seems to take a lot of time and messes up interrupts
 */
void ow_update_test_data();

/**
 * \brief Returns if the device is in testing mode or not for one wire system
 * 
 * \return `true` if in testing mode, `false` for normal operation
 */
bool ow_get_testing_mode_active();

#endif