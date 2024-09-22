#pragma once
#include "stdint.h"


namespace Crc
{
unsigned char getCRC8CheckSum(unsigned char *msg, unsigned int len, unsigned char ucCRC8);

unsigned int verifyCRC8CheckSum(unsigned char *msg, unsigned int len);

void appendCRC8CheckSum(unsigned char *msg, unsigned int len);

/**
 * @brief calcutes the CRC16 checksoum for the ENTIRE message
 *
 * @param msg the given message
 * @param len length of the message
 * @param wCRC should be CRC_INIT_REFEREE
 * @return uint16_t
 */
uint16_t getCRC16CheckSum(uint8_t *msg, uint32_t len, uint16_t wCRC);

/**
 * @brief verifies crc16 checksum of the given message
 *
 * @param msg given message, THE LAST TWO BYTES OF THE ARRAY ARE NOT
 * CALCULATED FOR CRC
 * @param len length of the ENTIRE message
 */
uint32_t verifyCRC16CheckSum(uint8_t *msg, uint32_t len);
/**
 * @brief appends crc16 checksum to the given message
 *
 * @param msg given message, THE LAST TWO BYTES OF THE ARRAY ARE NOT
 * CALCULATED INTO CRC
 * @param len length of the ENTIRE message
 */

void appendCRC16CheckSum(uint8_t *msg, uint32_t len);

/**
 * @brief verifies crc16 checksum of the given message
 *
 * @param msg
 * @param len
 */
uint32_t verifyCRC16CCITTCheckSum(uint8_t *msg, uint32_t len);
/**
 * @brief appends crc16 checksum to the given message
 *
 * @param msg
 * @param len
 */
void appendCRC16CCITTCheckSum(uint8_t *msg, const uint32_t &len);

}  // namespace Crc

