#include "mcp4562.h"

// Static functions declaration

static uint8_t MCP4562_GetWiperAddress(MCP4562_Wiper wiper);
static uint8_t MCP4562_GetNVWiperAddress(MCP4562_Wiper wiper);
static MCP4562_ErrorType MCP4562_CheckIfOpToNV(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, int retry);
static int MCP4562_IsEEPROMBusy(MCP4562_HandleTypeDef *mcp4562);
static MCP4562_ErrorType MCP4562_SendCommand(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, MCP4562_Command command);
static MCP4562_ErrorType MCP4562_WriteData(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, uint16_t value);
static MCP4562_ErrorType MCP4562_Read(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, uint16_t *value);

/**
 * @brief Return the register address of the wiper 0 or 1
 *
 * @param wiper Which wiper to control
 * @return `uint8_t` address of the wiper
 */
static uint8_t MCP4562_GetWiperAddress(MCP4562_Wiper wiper)
{
    switch (wiper)
    {
    default:
    case 0:
        return MCP4562_WIPER0_ADDR;
    case 1:
        return MCP4562_WIPER1_ADDR;
    }
}

/**
 * @brief Return the register address of the non volatile wiper 0 or 1
 *
 * @param wiper Which wiper to control
 * @return `uint8_t` address of the wiper
 */
static uint8_t MCP4562_GetNVWiperAddress(MCP4562_Wiper wiper)
{
    switch (wiper)
    {
    default:
    case 0:
        return MCP4562_NVWIPER0_ADDR;
    case 1:
        return MCP4562_NVWIPER1_ADDR;
    }
}

/**
 * @brief Check if the EEPROM is busy with a write operation.
 * Indeed, if we sent a write command before, we can't read or write to the EEPROM until the write operation is finished.
 *
 * @param mcp4562 The device handle
 * @return `int` 0 if the EEPROM is not busy, 1 if the EEPROM is busy
 */
static int MCP4562_IsEEPROMBusy(MCP4562_HandleTypeDef *mcp4562)
{
    uint16_t status = 0;
    int err = MCP4562_GetStatus(mcp4562, &status);
    if (err != MCP4562_OK)
    {
        return err;
    }
    return (status & MCP4562_STATUS_EEWA) ? 1 : 0;
}

/**
 * @brief Send an increment or decrement command to wiper 0 or 1.
 * Only volatile wipers can be incremented or decremented.
 *
 * @param mcp4562 The device handle
 * @param wiper Which wiper to increment or decrement
 * @param command Increment or decrement command
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
static MCP4562_ErrorType MCP4562_SendCommand(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, MCP4562_Command command)
{
    /*
     * Command explanation
     *  COMMAND char    : AAAA CC DD (8 bits)
     *      AAAA            : Adress in memory to do things
     *      CC              : What to do at AAAA ( R,W,Inc,Decr )
     *      00              : Write
     *      01              : increment (NV only)
     *      10              : Decrement (NV only)
     *      11              : Read
     *      DD              : D9 and D8 ( MSB of the value we are sending)
     *
     *  DATA char       : DDDD DDDD (8 bits) *only read and write commands have data
     */
    if (mcp4562 == NULL || mcp4562->write == NULL)
    {
        return MCP4562_ERROR_HANDLE_NULL;
    }

    if (command != MCP4562_INCREMENT && command != MCP4562_DECREMENT)
    {
        return MCP4562_ERROR_BAD_COMMAND;
    }

    uint8_t data_to_send = MCP4562_GetWiperAddress(wiper) << MCP4562_REGISTER_ADDRESS_POS; // AAAA = wiper number
    data_to_send |= command << MCP4562_COMMAND_POS;                                        // CC = command

    if (mcp4562->write(mcp4562->address, &data_to_send, 1))
    {
        return MCP4562_ERROR_I2C_TRANSMIT_FAIL;
    }

    return MCP4562_OK;
}

/**
 * @brief Check if there is an operation to a non volatile register
 *
 * @param mcp4562 The device handle
 * @param reg_address The register address to write to
 * @param retry If set to 0, the function will return MCP4562_ERROR_EEWA directly if the EEPROM is busy.
 * If set to 1, the function will wait until the EEPROM is not busy or until the timeout is reached
 * @return `MCP4562_ErrorType` MCP4562_OK if not writing to a NV register or if the EEPROM is not busy. MCP4562_ERROR_EEWA if the EEPROM is busy
 */
static MCP4562_ErrorType MCP4562_CheckIfOpToNV(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, int retry)
{
    if (mcp4562 == NULL || mcp4562->delay == NULL){
        return MCP4562_ERROR_HANDLE_NULL;
    }
    // Test if we are writing to a non volatile register
    uint8_t reg = (reg_address >> (MCP4562_REGISTER_ADDRESS_POS + MCP4562_MSB_DATA_POS)) & 0x0F;
    if (!(reg == MCP4562_STATUS_ADDR || reg == MCP4562_TCON_ADDR || reg == MCP4562_WIPER0_ADDR || reg == MCP4562_WIPER1_ADDR))
    {
        // We are writing to a non volatile register so we must ensure that there is no previous write operation in progress
        // by reading the EEWA (EEPROM Write Active Status bit) in the status register
        int time = 0;

        while (MCP4562_IsEEPROMBusy(mcp4562))
        {
            if (retry == 0)
            {
                return MCP4562_ERROR_EEWA;
            }
            // wait 10ms before checking again
            mcp4562->delay(MCP4562_EEWA_READ_STEPS);
            time += (MCP4562_EEWA_READ_STEPS);
            if (time >= MCP4562_EEWA_TIMEOUT)
            {
                // We have waited too long, the EEPROM is still busy
                return MCP4562_ERROR_EEWA;
            }
        }
    }
    return MCP4562_OK;
}

/**
 * @brief Write `value` to the MCP4562 at a specific `reg_address` register address
 *
 * @param mcp4562 The device handle
 * @param reg_address The register address to write to
 * @param value The value to write. The value can be at maximum 10 bits long
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
static MCP4562_ErrorType MCP4562_WriteData(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, uint16_t value)
{
    if (mcp4562 == NULL || mcp4562->write == NULL)
    {
        return MCP4562_ERROR_HANDLE_NULL;
    }

    int err = MCP4562_CheckIfOpToNV(mcp4562, reg_address, 1);
    if (err != MCP4562_OK)
    {
        return err;
    }
    /** As reminder
     * COMMAND char     : AAAACCDD DDDDDDDD
     *  AAAA            : Adress in memory to do things
     *  CC              : What to do at AAAA ( R,W,Inc,Decr )
     *  00              : Write
     *  01              : increment (NV only)
     *  10              : Decrement (NV only)
     *  11              : Read
     *  DD DDDD DDDD    : D9 and D0 *only read and write commands have data
     */
    // Make value Endianess agnostic
    uint8_t sending_buffer[2];
    sending_buffer[0] = reg_address << MCP4562_REGISTER_ADDRESS_POS | MCP4562_WRITE << MCP4562_COMMAND_POS | ((value >> 8) && 0x3);
    sending_buffer[1] = (value & 0xff);

    if (mcp4562->write(mcp4562->address, sending_buffer, 2))
    {
        return MCP4562_ERROR_I2C_TRANSMIT_FAIL;
    }

    return MCP4562_OK;
}

/**
 * @brief Read the value at a specific `reg_address` register address
 *
 * @param mcp4562 The device handle
 * @param reg_address The register address to read from
 * @param value The value read from the register. Only one value is read, 10 bits long
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
static MCP4562_ErrorType MCP4562_Read(MCP4562_HandleTypeDef *mcp4562, uint8_t reg_address, uint16_t *value)
{
    if (mcp4562 == NULL || mcp4562->read == NULL || mcp4562->write == NULL || value == NULL)
    {
        return MCP4562_ERROR_HANDLE_NULL;
    }

    // Here we must ensure that we are not reading from a non volatile register while a write operation is in progress
    // by reading the EEWA (EEPROM Write Active Status bit) in the status register
    int err = MCP4562_CheckIfOpToNV(mcp4562, reg_address, 1);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // See MCP4562_WriteData for the explanation of the command
    reg_address <<= MCP4562_REGISTER_ADDRESS_POS;         // AAAA = register address
    reg_address |= (MCP4562_READ << MCP4562_COMMAND_POS); // CC = Read

    if (mcp4562->write(mcp4562->address, &reg_address, 1))
    {
        return MCP4562_ERROR_I2C_TRANSMIT_FAIL;
    }

    // We always read 2 bytes
    uint8_t receiving_buffer[2];
    if (mcp4562->read(mcp4562->address, receiving_buffer, 2))
    {
        return MCP4562_ERROR_I2C_RECEIVE_FAIL;
    }
    *value = (receiving_buffer[0] << 8) | receiving_buffer[1];

    return MCP4562_OK;
}

/**
 * @brief Initialize the MCP4562 device
 *
 * @param mcp4562 The device handle
 * @param i2c_address_offset The I2C address offset of the device
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_Init(MCP4562_HandleTypeDef *mcp4562, uint8_t i2c_address_offset)
{
    if (mcp4562 == NULL)
    {
        return MCP4562_ERROR_HANDLE_NULL;
    }
    mcp4562->address = (MCP4562_BASE_ADDRESS | i2c_address_offset) << 1;
    mcp4562->write = NULL;
    mcp4562->read = NULL;
    mcp4562->delay = NULL;
    mcp4562->software_reset = NULL;

    return MCP4562_OK;
}

/**
 * @brief Set the wiper value of the MCP4562 to volatile memory
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to set
 * @param value if wiper value exceeds MCP4562_MAX_WIPER_VALUE, it will be set to MCP4562_MAX_WIPER_VALUE
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_SetWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t value)
{
    if (value > MCP4562_MAX_WIPER_VALUE)
    {
        value = MCP4562_MAX_WIPER_VALUE;
    }
    return MCP4562_WriteData(mcp4562, MCP4562_GetWiperAddress(wiper), value);
}

/**
 * @brief Get the wiper value of the MCP4562 from volatile memory
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to get
 * @param value The value of the wiper, 10 bits long
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_GetWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t *value)
{
    return MCP4562_Read(mcp4562, MCP4562_GetWiperAddress(wiper), value);
}

/**
 * @brief Set the wiper value of the MCP4562 to non volatile memory
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to set
 * @param value if wiper value exceeds MCP4562_MAX_WIPER_VALUE, it will be set to MCP4562_MAX_WIPER_VALUE
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_SetNVWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t value)
{
    if (value > MCP4562_MAX_WIPER_VALUE)
    {
        value = MCP4562_MAX_WIPER_VALUE;
    }
    return MCP4562_WriteData(mcp4562, MCP4562_GetNVWiperAddress(wiper), value);
}

/**
 * @brief Get the wiper value of the MCP4562 from non volatile memory
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to get
 * @param value The value of the wiper, 10 bits long
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_GetNVWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, uint16_t *value)
{
    return MCP4562_Read(mcp4562, MCP4562_GetNVWiperAddress(wiper), value);
}

/**
 * @brief Returns the status of the device
 *
 * @note D3 = EEPROM Write active status bit.
 *      This bit indicates if a write operation is in progress in the EEPROM.
 *      1 = write operation in progress, 0 = no write operation in progress.
 * @note D2 = WiperLock status for wiper 1.
 *      This bit indicates if the Wiper Lock feature is enabled or disabled for wiper 1.
 *      1 = Wiper Lock feature enabled, 0 = Wiper Lock feature disabled.
 *      This feature can only change with a High Voltage operation. read datasheet for more information.
 * @note D1 = WiperLock status for wiper 0.
 *      same a D2 but for wiper 0.
 * @note D0 = EEPROM write Protect status bit.
 *      This bit indicates if the EEPROM write protection is enabled or disabled.
 *      1 = EEPROM write protection enabled, 0 = EEPROM write protection disabled.
 *      read datasheet for more information.
 * @note D8:D4 = forced to 1 (reserved). So we mask them to 0
 *
 * @param mcp4562 The device handle
 * @param value The status value of the device, 9 bits long
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_GetStatus(MCP4562_HandleTypeDef *mcp4562, uint16_t *value)
{
    uint16_t status = 0;
    int err = MCP4562_Read(mcp4562, MCP4562_STATUS_ADDR, &status);
    if (err != MCP4562_OK)
    {
        return err;
    }
    *value = status & 0b1111; // Only the 4 last bits are useful
    return MCP4562_OK;
}

/**
 * @brief This function sets the TCON register
 *
 * @note D8 = GCEN (General Call Enable). This bit specifies if I2C General Call commands are accepted
 *     1 = General Call commands are accepted (default). 0 = General Call commands are not accepted.
 * @note D7 = R1HW Resistor 1 Hardware Configuration Control bit.
 *      This bit forces Resistor 1 into the “shutdown” configuration of the Hardware pin
 *      1 = Resistor 1 is NOT forced to the hardware pin “shutdown” configuration.
 *      0 = Resistor 1 is forced to the hardware pin “shutdown” configuration
 * @note D6 = R1A Resistor 1 Terminal A (P1A pin) Connection Control bit.
 *      This bit connects/disconnects the Resistor 1 Terminal A to the Resistor 1 Network
 *      1 = P1A pin is connected to the Resistor 1 Network
 *      0 = P1A pin is disconnected from the Resistor 1 Network
 * @note D5 = R1W: Resistor 1 Wiper (P1W pin) Connect Control bit
 *      This bit connects/disconnects the Resistor 1 Wiper to the Resistor 1 Network
 *      1 = P1W pin is connected to the Resistor 1 Network
 *      0 = P1W pin is disconnected from the Resistor 1 Network
 * @note D4 = R1B: Resistor 1 Terminal B (P1B pin) Connect Control bit
 *      This bit connects/disconnects the Resistor 1 Terminal B to the Resistor 1 Network
 *      1 = P1B pin is connected to the Resistor 1 Network
 *      0 = P1B pin is disconnected from the Resistor 1 Network
 * @note D3 = R0HW Resistor 0 Hardware Configuration Control bit.
 *      Same as R1HW but for Resistor 0
 * @note D2 = R0A Resistor 0 Terminal A (P0A pin) Connection Control bit.
 *      Same as R1A but for Resistor 0
 * @note D1 = R0W Resistor 0 Wiper (P0W pin) Connect Control bit.
 *      Same as R1W but for Resistor 0
 * @note D0 = R0B Resistor 0 Terminal B (P0B pin) Connect Control bit.
 *      Same as R1B but for Resistor 0
 *
 * @param mcp4562 The device handle
 * @param value 9 bits value to set the TCON register
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_SetTCON(MCP4562_HandleTypeDef *mcp4562, uint16_t value)
{
    return MCP4562_WriteData(mcp4562, MCP4562_TCON_ADDR, value);
}

/**
 * @brief This function gets the TCON register. See `MCP4562_SetTCON` for the meaning of the bits
 *
 * @param mcp4562 The device handle
 * @param value The value of the TCON register, 9 bits long
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_GetTCON(MCP4562_HandleTypeDef *mcp4562, uint16_t *value)
{
    return MCP4562_Read(mcp4562, MCP4562_TCON_ADDR, value);
}

/**
 * @brief This function shutdown the device by disconnecting all terminals of the resistors
 *
 * @param mcp4562 The device handle
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_ShutdownAll(MCP4562_HandleTypeDef *mcp4562)
{
    int err = MCP4562_OK;
    err = MCP4562_ShutdownWiper(mcp4562, MCP4562_WIPER0);
    if (err != MCP4562_OK)
    {
        return err;
    }
    err = MCP4562_ShutdownWiper(mcp4562, MCP4562_WIPER1);
    if (err != MCP4562_OK)
    {
        return err;
    }
    return err;
}

/**
 * @brief This function wake up the device by connecting all terminals of the resistors
 *
 * @param mcp4562 The device handle
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_WakeUpAll(MCP4562_HandleTypeDef *mcp4562)
{
    int err = MCP4562_OK;
    err = MCP4562_WakeUpWiper(mcp4562, MCP4562_WIPER0);
    if (err != MCP4562_OK)
    {
        return err;
    }
    err = MCP4562_WakeUpWiper(mcp4562, MCP4562_WIPER1);
    if (err != MCP4562_OK)
    {
        return err;
    }
    return err;
}

/**
 * @brief This function shutdown a specific wiper by disconnecting all of it's terminals of the resistors
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to shutdown
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_ShutdownWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper)
{
    uint16_t tcon_data;
    int err = MCP4562_OK;
    err = MCP4562_GetTCON(mcp4562, &tcon_data); // Get the current TCON value
    if (err != MCP4562_OK)
    {
        return err;
    }

    switch (wiper)
    {
    default:
    case MCP4562_WIPER0:
        tcon_data &= ~MCP4562_TCON_R0HW; // Set the R0HW bit to 0
        break;
    case MCP4562_WIPER1:
        tcon_data &= ~MCP4562_TCON_R1HW; // Set the R1HW bit to 0
        break;
    }
    return MCP4562_SetTCON(mcp4562, tcon_data);
}

/**
 * @brief This function wake up a specific wiper by connecting all of it's terminals of the resistors
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to wake up
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_WakeUpWiper(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper)
{
    uint16_t tcon_data;
    int err = MCP4562_OK;
    err = MCP4562_GetTCON(mcp4562, &tcon_data); // Get the current TCON value
    if (err != MCP4562_OK)
    {
        return err;
    }

    switch (wiper)
    {
    default:
    case MCP4562_WIPER0:
        tcon_data |= MCP4562_TCON_R0HW; // Set the R0HW bit to 1
        break;
    case MCP4562_WIPER1:
        tcon_data |= MCP4562_TCON_R1HW; // Set the R1HW bit to 1
        break;
    }
    return MCP4562_SetTCON(mcp4562, tcon_data);
}

/**
 * @brief This function increments the wiper by 1
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to increment
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_Increment(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper)
{
    return MCP4562_SendCommand(mcp4562, wiper, MCP4562_INCREMENT);
}

/**
 * @brief This function decrements the wiper by 1
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to decrement
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_Decrement(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper)
{
    return MCP4562_SendCommand(mcp4562, wiper, MCP4562_DECREMENT);
}

/**
 * @brief This function resets the I2C state machine of the device
 *
 * It is the responsability to the user to implement it following the format below:
 * 
 * In order to soft reset the I2C communication when an error occurs we should send the following bits:
 * | S | 1 1 1 1 1 1 1 1 1 (9 ones) | S | P |
 * S = start bit, P = stop bit
 * This will reset the I2C communication and the device will be ready to receive new commands
 *
 * @param mcp4562 The device handle
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_SoftwareReset(MCP4562_HandleTypeDef *mcp4562)
{
    if (mcp4562 == NULL || mcp4562->software_reset == NULL)
    {
        return MCP4562_ERROR_HANDLE_NULL;
    }

    return mcp4562->software_reset();
}

/**
 * @brief This function reads the EEPROM data memory of the device
 *
 * @param mcp4562 The device handle
 * @param address The address of the data memory to read from. The address must be between `MCP4562_DATA_BASE_ADDR` and `MCP4562_DATA_END_ADDR`
 * @param value The value of the data memory, 10 bits long. Value must be allocated by the caller
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_GetData(MCP4562_HandleTypeDef *mcp4562, uint8_t address, uint16_t *value)
{
    // test if the address is in the range of the data memory
    if (address < MCP4562_DATA_BASE_ADDR || address > MCP4562_DATA_END_ADDR)
    {
        return MCP4562_ERROR_DATA_ADDR_OUT_OF_RANGE;
    }
    return MCP4562_Read(mcp4562, address, value);
}

/**
 * @brief This function writes the EEPROM data memory of the device
 *
 * @param mcp4562 The device handle
 * @param address The address of the data memory to write to. The address must be between `MCP4562_DATA_BASE_ADDR` and `MCP4562_DATA_END_ADDR`
 * @param value The value to write to the data memory, 10 bits long
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_SetData(MCP4562_HandleTypeDef *mcp4562, uint8_t address, uint16_t value)
{
    // test if the address is in the range of the data memory
    if (address < MCP4562_DATA_BASE_ADDR || address > MCP4562_DATA_END_ADDR)
    {
        return MCP4562_ERROR_DATA_ADDR_OUT_OF_RANGE;
    }

    return MCP4562_WriteData(mcp4562, address, value);
}

/**
 * @brief This function enables the General Call Enable bit in the TCON register
 *
 * @param mcp4562 The device handle
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_EnableGCEN(MCP4562_HandleTypeDef *mcp4562)
{
    uint16_t tcon_data;
    int err = MCP4562_OK;
    err = MCP4562_GetTCON(mcp4562, &tcon_data); // Get the current TCON value
    if (err != MCP4562_OK)
    {
        return err;
    }

    tcon_data |= MCP4562_TCON_GCEN; // Set the GCEN bit to 1

    return MCP4562_SetTCON(mcp4562, tcon_data);
}

/**
 * @brief This function disables the General Call Enable bit in the TCON register
 *
 * @param mcp4562 The device handle
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_DisableGCEN(MCP4562_HandleTypeDef *mcp4562)
{
    uint16_t tcon_data;
    int err = MCP4562_OK;
    err = MCP4562_GetTCON(mcp4562, &tcon_data); // Get the current TCON value
    if (err != MCP4562_OK)
    {
        return err;
    }

    tcon_data &= ~MCP4562_TCON_GCEN; // Set the GCEN bit to 0

    return MCP4562_SetTCON(mcp4562, tcon_data);
}

// TODO: To test
/**
 * @brief This function tries to set the resistance of the wiper to the desired value.
 *
 * @note The wiper value will be set to the closest value possible due to the numeric resistance having a limited number of steps.
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to set
 * @param resistance The resistance to set. The resistance must be between `MCP4562_MIN_RESISTANCE` and `MCP4562_MAX_RESISTANCE`
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_SetResistance(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, float resistance)
{
    if (resistance < MCP4562_MIN_RESISTANCE || resistance > MCP4562_MAX_RESISTANCE)
    {
        return MCP4562_ERROR_RESISTANCE_OUT_OF_RANGE;
    }
    // The formula to found the step we need for the numeric rhesotat is (target_res) = (R internal wiper) + (step value) * n where n is the number of steps
    // So n = ((target_res) - Rmin) / steps and then we round the result (the round can be changed later)
    uint16_t R_to_set = roundf((resistance - MCP4562_MIN_RESISTANCE) / MCP4562_STEP_VALUE);
    return MCP4562_SetWiper(mcp4562, wiper, R_to_set);
}

/**
 * @brief This function gets the resistance programmed in the wiper
 *
 * @param mcp4562 The device handle
 * @param wiper The wiper to get
 * @param resistance The resistance of the wiper
 * @return MCP4562_ErrorType MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_GetResistance(MCP4562_HandleTypeDef *mcp4562, MCP4562_Wiper wiper, float *resistance)
{
    if (resistance == NULL)
    {
        return MCP4562_ERROR_HANDLE_NULL;
    }

    uint16_t wiper_value = 0;
    MCP4562_ErrorType err = MCP4562_GetWiper(mcp4562, wiper, &wiper_value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    *resistance = MCP4562_MIN_RESISTANCE + wiper_value * MCP4562_STEP_VALUE;
    return MCP4562_OK;
}

/**
 * @brief This function tries to test a maximum of functions. This is not a complete test of the library.
 *
 * @param mcp4562 The device handle
 * @return `MCP4562_ErrorType` MCP4562_OK if no error occurred
 */
MCP4562_ErrorType MCP4562_TestLibrary(MCP4562_HandleTypeDef *mcp4562)
{
    if (mcp4562 == NULL)
    {
        return MCP4562_ERROR_HANDLE_NULL;
    }

    int err = 0;

    // Test the get wiper function
    uint16_t value = 0;
    err = MCP4562_GetWiper(mcp4562, MCP4562_WIPER0, &value);
    if (err != MCP4562_OK)
    {
        return err;
    }
    printf("Wiper 0 value: %d\n", value);

    uint16_t NVvalue = 0;
    err = MCP4562_GetNVWiper(mcp4562, MCP4562_WIPER0, &NVvalue);
    if (err != MCP4562_OK)
    {
        return err;
    }
    printf("Non-volatile Wiper 0 value: %d\n", value);

    // assert that value and NVvalue are equal (This is only value after a POR)
    //    if (value != NVvalue) {
    //        printf("Error: Wiper 0 and Non-volatile Wiper 0 values are different\n");
    //        return MCP4562_ERROR;
    //    }

    // Test the increment function
    err = MCP4562_Increment(mcp4562, MCP4562_WIPER0);
    if (err != MCP4562_OK)
    {
        return err;
    }

    uint16_t new_value = 0;
    err = MCP4562_GetWiper(mcp4562, MCP4562_WIPER0, &new_value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that the new value is the old value + 1
    if (new_value != value + 1)
    {
        printf("Error: Wiper 0 value is not incremented\n");
        return MCP4562_ERROR;
    }

    // Test the decrement function
    err = MCP4562_Decrement(mcp4562, MCP4562_WIPER0);
    if (err != MCP4562_OK)
    {
        return err;
    }

    err = MCP4562_GetWiper(mcp4562, MCP4562_WIPER0, &new_value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that the new value is the old value
    if (new_value != value)
    {
        printf("Error: Wiper 0 value is not decremented\n");
        return MCP4562_ERROR;
    }

    // Test the set wiper function
    err = MCP4562_SetWiper(mcp4562, MCP4562_WIPER0, 2);
    if (err != MCP4562_OK)
    {
        return err;
    }

    err = MCP4562_GetWiper(mcp4562, MCP4562_WIPER0, &new_value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that the new value is the set value
    if (new_value != 2)
    {
        printf("Error: Wiper 0 value is not set\n");
        return MCP4562_ERROR;
    }

    // Test the set non-volatile wiper function
    err = MCP4562_SetNVWiper(mcp4562, MCP4562_WIPER0, 2);
    if (err != MCP4562_OK)
    {
        return err;
    }

    uint16_t new_NVvalue = 0;
    err = MCP4562_GetNVWiper(mcp4562, MCP4562_WIPER0, &new_NVvalue);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that the new value is the set value
    if (new_NVvalue != 2)
    {
        printf("Error: Non-volatile Wiper 0 value is not set\n");
        return MCP4562_ERROR;
    }

    // Reset original NV value
    err = MCP4562_SetNVWiper(mcp4562, MCP4562_WIPER0, NVvalue);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Test the set TCON function
    // err = MCP4562_SetTCON(mcp4562, 0x100);
    // if (err != MCP4562_OK) {
    //     return err;
    // }

    // Test the get TCON function
    uint16_t tcon = 0;
    err = MCP4562_GetTCON(mcp4562, &tcon);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Test the shutdown all function
    err = MCP4562_ShutdownAll(mcp4562);
    if (err != MCP4562_OK)
    {
        return err;
    }

    uint16_t new_tcon = 0;
    err = MCP4562_GetTCON(mcp4562, &new_tcon);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that tcon has now the R0HW and R1HW bits set to 0
    if ((new_tcon & MCP4562_TCON_R0HW) || (new_tcon & MCP4562_TCON_R1HW))
    {
        printf("Error: TCON register is not shutdown\n");
        return MCP4562_ERROR;
    }

    // Test the wake up all function
    err = MCP4562_WakeUpAll(mcp4562);
    if (err != MCP4562_OK)
    {
        return err;
    }

    err = MCP4562_GetTCON(mcp4562, &new_tcon);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that tcon has now the R0HW and R1HW bits set to 1
    if (!(new_tcon & MCP4562_TCON_R0HW) || !(new_tcon & MCP4562_TCON_R1HW))
    {
        printf("Error: TCON register is not woken up\n");
        return MCP4562_ERROR;
    }

    // Assert that the new value of TCON is the same as initial
    if (new_tcon != tcon)
    {
        printf("Error: TCON register is not the same as initial\n");
        return MCP4562_ERROR;
    }
    // Test the get status function
    err = MCP4562_GetStatus(mcp4562, &value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that bit 0, 1, and 2 are set to 0 (EEPROM write protection disabled, Wiper Lock feature disabled)
    if ((value & 0x7) != 0)
    {
        printf("Error: Status register is not as expected\n");
        return MCP4562_ERROR;
    }

    // Test the get data function
    err = MCP4562_GetData(mcp4562, MCP4562_DATA_BASE_ADDR, &value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Test the set data function
    err = MCP4562_SetData(mcp4562, MCP4562_DATA_BASE_ADDR, 0x123);
    if (err != MCP4562_OK)
    {
        return err;
    }

    err = MCP4562_GetData(mcp4562, MCP4562_DATA_BASE_ADDR, &new_value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    // Assert that the new value is the set value
    if (new_value != 0x123)
    {
        printf("Error: Data memory value is not set\n");
        return MCP4562_ERROR;
    }

    // Reset original value
    err = MCP4562_SetData(mcp4562, MCP4562_DATA_BASE_ADDR, value);
    if (err != MCP4562_OK)
    {
        return err;
    }

    printf("All tests passed\n");

    return MCP4562_OK;
}
