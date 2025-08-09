#ifndef HX711_HEADER_H
#define HX711_HEADER_H

#include <stdint.h>

/**
 * \brief Sets up the HX711 interface
 * 
 */
void setupHX();

/**
 * \brief Checks if there is data available from HX711
 * 
 * \return True if HX711 has data ready.
 */
bool readyHX();

/**
 * \brief Reads value from HX711
 * 
 * \param clocks Number of clocks to read (25 for next reading to be 128 gain on channel A, 26 for 32 gain channel B)
 * 
 * \warning Disables interrupts during read
 * 
 * \return Reading, not sign extended
 */
int32_t readHX(uint8_t clocks);

#endif