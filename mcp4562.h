/**
 * ADDR is 0101 1[A1][A0]
 *
 *  COMMAND char    : AAAA CC DD
 *  AAAA            : Adress in memory to do things
 *  CC              : What to do at AAAA ( R,W,Inc,Decr )
 *  00              : Write
 *  01              : increment (NV only)
 *  10              : Decrement (NV only)
 *  11              : Read
 *  DD              : D9 and D8 ( MSB of the value we are sending)
 *
 *  DATA char       : DDDD DDDD *only read and write commands have data
 */
#ifndef _MCP4562_H
#define _MCP4562_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "mcp4562_defines.h"

// Defines depending on user device choice in mcp4562_defines.h

#if (defined(MCP4531) || defined(MCP4532) || defined(MCP4541) || defined(MCP4542) || defined(MCP4631) || defined(MCP4632) || defined(MCP4641) || defined(MCP4642))
#define MCP4562_WIPER_STEPS ((float)129.0) // Number of steps for the wiper
#define MCP4562_MAX_WIPER_VALUE 0x80       // Max value for wiper is 0x80 because we are in a 7-bit device
#else
#define MCP4562_WIPER_STEPS ((float)257.0) // Number of steps for the wiper
#define MCP4562_MAX_WIPER_VALUE 0x100      // Max value for wiper is 0x100 because we are in a 8-bit device
#endif

// Max number of wiper in the device : only two (Wiper 0 and Wiper 1)
#if defined(MCP4531) || defined(MCP4532) || defined(MCP4541) || defined(MCP4542) || defined(MCP4551) || defined(MCP4552) || defined(MCP4561) || defined(MCP4562)
#define MCP4562_MAX_NBR_WIPER 1
#else
#define MCP4562_MAX_NBR_WIPER 2
#endif

#define MCP4562_WIPER_INTERNAL_RESISTANCE 75 // Internal resistance of the wiper in Ohm

#if defined(MCP4562_OPTION_502)
#define MCP4562_MAX_RESISTANCE (5000 + MCP4562_WIPER_INTERNAL_RESISTANCE) // Maximum resistance in Ohm 5k + wiper resistance (75 Ohm)
#elif defined(MCP4562_OPTION_103)
#define MCP4562_MAX_RESISTANCE (10000 + MCP4562_WIPER_INTERNAL_RESISTANCE) // Maximum resistance in Ohm 10k + wiper resistance (75 Ohm)
#elif defined(MCP4562_OPTION_503)
#define MCP4562_MAX_RESISTANCE (50000 + MCP4562_WIPER_INTERNAL_RESISTANCE) // Maximum resistance in Ohm 50k + wiper resistance (75 Ohm)
#elif defined(MCP4562_OPTION_104)
#define MCP4562_MAX_RESISTANCE (100000 + MCP4562_WIPER_INTERNAL_RESISTANCE) // Maximum resistance in Ohm 100k + wiper resistance (75 Ohm)
#endif

// I2C ADDRESSES
#define MCP4562_BASE_ADDRESS 0x2C // 0b0010 11 + A1:A0

// CONFIG DEFINES
#define MCP4562_MIN_WIPER_VALUE 0x00                                                                              // Min value for wiper
#define MCP4562_REGISTER_ADDRESS_POS 4                                                                            // Position of the register address in the data
#define MCP4562_COMMAND_POS 2                                                                                     // Position of the command in the data
#define MCP4562_MSB_DATA_POS 8                                                                                    // Position of the MSB in the data
#define MCP4562_EEWA_TIMEOUT 100                                                                                  // EEPROM Write Active timeout in ms
#define MCP4562_EEWA_READ_STEPS 10                                                                                // Time to wait before reading again the status of the EEPROM Write Active
#define MCP4562_MIN_RESISTANCE MCP4562_WIPER_INTERNAL_RESISTANCE                                                  // Minimum resistance is the internal resistance of the wiper (75 Ohm)
#define MCP4562_STEP_VALUE (float)((MCP4562_MAX_RESISTANCE - MCP4562_MIN_RESISTANCE) / (MCP4562_WIPER_STEPS - 1)) // Step value for the wiper

// REGISTERS ADDRESSES
#define MCP4562_WIPER0_ADDR 0x00    // Volatile Wiper 0
#define MCP4562_WIPER1_ADDR 0x01    // Volatile Wiper 1
#define MCP4562_NVWIPER0_ADDR 0x02  // Non-Volatile Wiper 0, also Wiper Lock 0 Enable/Disable in High voltage
#define MCP4562_NVWIPER1_ADDR 0x03  // Non-Volatile Wiper 1, also Wiper Lock 1 Enable/Disable in High voltage
#define MCP4562_TCON_ADDR 0x04      // Volatile TCON
#define MCP4562_STATUS_ADDR 0x05    // Status
#define MCP4562_DATA_BASE_ADDR 0x06 // Non-Volatile Data Memory
#define MCP4562_DATA_END_ADDR 0x0F  // Non-Volatile Data Memory

// REGISTERS VALUES
#define MCP4562_TCON_GCEN (1 << 8)   // General Call Enable
#define MCP4562_TCON_R1HW (1 << 7)   // Resistor 1 Hardware
#define MCP4562_TCON_R1A (1 << 6)    // Resistor 1 A
#define MCP4562_TCON_R1W (1 << 5)    // Resistor 1 W
#define MCP4562_TCON_R1B (1 << 4)    // Resistor 1 B
#define MCP4562_TCON_R0HW (1 << 3)   // Resistor 0 Hardware
#define MCP4562_TCON_R0A (1 << 2)    // Resistor 0 A
#define MCP4562_TCON_R0W (1 << 1)    // Resistor 0 W
#define MCP4562_TCON_R0B (1 << 0)    // Resistor 0 B
#define MCP4562_STATUS_EEWA (1 << 3) // EEPROM Write Active

// ERROR CODES
typedef enum
{
    MCP4562_OK = 0,
    MCP4562_ERROR,
    MCP4562_ERROR_EEWA,                   // EEPROM Write is in progress longer than timeout
    MCP4562_ERROR_HANDLE_NULL,            // Handle is NULL
    MCP4562_ERROR_I2C_HANDLE_NULL,        // I2C handle is NULL
    MCP4562_ERROR_I2C_TRANSMIT_FAIL,      // I2C transmit failed
    MCP4562_ERROR_I2C_RECEIVE_FAIL,       // I2C receive failed
    MCP4562_ERROR_DATA_ADDR_OUT_OF_RANGE, // Data address is out of range
    MCP4562_ERROR_BAD_COMMAND,            // Command is not valid
    MCP4562_ERROR_WIPER_LOCKED,           // Wiper is locked
    MCP4562_ERROR_WIPER_DISABLED,         // Wiper is disabled
    MCP4562_ERROR_RESISTANCE_OUT_OF_RANGE // Resistance is out of range
} MCP4562_ErrorType;

/**
 * @brief Handle for the MCP4562.
 * The `address` is the 7bits address of the device on the I2C bus.
 */
typedef struct
{
    uint8_t address;

    /* User defined functions */
    MCP4562_ErrorType (*write)(uint8_t address, uint8_t *data, uint8_t bytes_to_send);
    MCP4562_ErrorType (*read)(uint8_t address, uint8_t *data, uint8_t bytes_to_read);
    MCP4562_ErrorType (*delay)(uint32_t ms);
    MCP4562_ErrorType (*software_reset)(); // Facultative, only necessary if we want to implement the software function
} MCP4562_HandleTypeDef;

/**
 * @brief Commands allowed to be sent to the MCP4562
 */
typedef enum
{
    MCP4562_WRITE = 0x00,     // Write to the device
    MCP4562_INCREMENT = 0x01, // Increment the wiper value by 1
    MCP4562_DECREMENT = 0x02, // Decrement the wiper value by 1
    MCP4562_READ = 0x03       // Read from the device
} MCP4562_Command;

/**
 * @brief Wiper to control.
 * There is a maximum of two wipers in the MCP4xxx family. But not all devices have two wipers.
 */
typedef enum
{
    MCP4562_WIPER0 = 0,
    MCP4562_WIPER1 = 1
} MCP4562_Wiper;

// Those functions MUST be implemented in the user code

/**
 * @brief This function writes `bytes_to_send` bytes from `data` to the I2C platform dependent bus.
 *
 * @note This function MUST be implemented by the user as I2C writing is platform dependent
 *
 * @param mcp4562 The MCP4562 handle with the address of the device
 * @param data The data to send
 * @param bytes_to_send The number of bytes to send
 * @return `MCP4562_ErrorType` 0 if no error occurred
 */
// extern MCP4562_ErrorType MCP4562_Write_I2C(MCP4562_HandleTypeDef *mcp4562, uint8_t *data, uint8_t bytes_to_send);

/**
 * @brief This function reads `bytes_to_read` bytes from the I2C platform dependent bus to buffer `data`.
 * `data` must be allocated before calling this function.
 *
 * @note This function MUST be implemented by the user as I2C reading is platform dependent.
 *
 * @param mcp4562 The MCP4562 handle with the address of the device
 * @param data The buffer to store the data
 * @param bytes_to_read The number of bytes to read
 * @return `MCP4562_ErrorType` 0 if no error occurred
 */
// extern MCP4562_ErrorType MCP4562_Read_I2C(MCP4562_HandleTypeDef *mcp4562, uint8_t *data, uint8_t bytes_to_read);

/**
 * @brief This function delays the program for `ms` milliseconds.
 *
 * @note This function MUST be implemented by the user as delay is platform dependent.
 *
 * @param ms The number of milliseconds to wait
 * @return `MCP4562_ErrorType` 0 if no error occurred
 */
// extern MCP4562_ErrorType MCP4562_Delay(uint32_t ms);

MCP4562_ErrorType MCP4562_Init(MCP4562_HandleTypeDef *mcp4562, uint8_t i2c_address_offset);
MCP4562_ErrorType MCP4562_Increment(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper);
MCP4562_ErrorType MCP4562_Decrement(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper);
MCP4562_ErrorType MCP4562_SetWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t value);
MCP4562_ErrorType MCP4562_GetWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t *value);
MCP4562_ErrorType MCP4562_SetNVWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t value);
MCP4562_ErrorType MCP4562_GetNVWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t *value);
MCP4562_ErrorType MCP4562_SetTCON(MCP4562_HandleTypeDef *mcp4562, uint16_t value);
MCP4562_ErrorType MCP4562_GetTCON(MCP4562_HandleTypeDef *mcp4562, uint16_t *value);
MCP4562_ErrorType MCP4562_GetStatus(MCP4562_HandleTypeDef *mcp4562, uint16_t *value);
MCP4562_ErrorType MCP4562_GetData(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, uint16_t *value);
MCP4562_ErrorType MCP4562_SetData(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, uint16_t value);
MCP4562_ErrorType MCP4562_ShutdownAll(MCP4562_HandleTypeDef *mcp4562);
MCP4562_ErrorType MCP4562_WakeUpAll(MCP4562_HandleTypeDef *mcp4562);
MCP4562_ErrorType MCP4562_ShutdownWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper);
MCP4562_ErrorType MCP4562_WakeUpWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper);
MCP4562_ErrorType MCP4562_EnableGCEN(MCP4562_HandleTypeDef *mcp4562);
MCP4562_ErrorType MCP4562_DisableGCEN(MCP4562_HandleTypeDef *mcp4562);
MCP4562_ErrorType MCP4562_SetResistance(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, float resistance);
MCP4562_ErrorType MCP4562_GetResistance(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, float *resistance);
MCP4562_ErrorType MCP4562_SoftwareReset(MCP4562_HandleTypeDef *mcp4562);
MCP4562_ErrorType MCP4562_TestLibrary(MCP4562_HandleTypeDef *mcp4562);

// TODO: general call commands
// TODO: High voltage commands

#ifdef __cplusplus
}
#endif

#endif // _MCP4562_H
