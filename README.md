# MCP4562 library

This readme explains some of the functionalities of the `7/8-Bit Single/Dual I2C Digital POT with Nonvolatile Memory` library from Microchip and how to use it. This digital potentiometer has the following characteristics:

- Single or Dual Resistor Network Options
- Potentiometer or Rheostat Configuration Options
- Resistor Network Resolution
    - 7-bit: 128 Resistors (129 Steps)
    - 8-bit: 256 Resistors (257 Steps)
- RAB Resistances Options of:
    - 5kOhm
    - 10 kOhm
    - 50 kOhm
    - 100 kOhm
- Zero-Scale to Full-Scale Wiper Operation
- Low Wiper Resistance: 75 (typical)
- Low Tempco:
    - Absolute (Rheostat): 50 ppm typical
(0°C to 70°C)
    - Ratiometric (Potentiometer): 15 ppm typical
- Nonvolatile Memory
    - Automatic Recall of Saved Wiper Setting
    - WiperLock™ Technology
    - 10 General Purpose Memory Locations
- I2C Serial Interface
    - 100 kHz, 400 kHz and 3.4 MHz Support
- Serial Protocol Allows:
    - High-Speed Read/Write to Wiper
    - Read/Write to EEPROM
    - Write Protect to be Enabled/Disabled
    - WiperLock to be Enabled/Disabled
- Resistor Network Terminal Disconnect Feature
via the Terminal Control (TCON) Register
- Write Protect Feature:
    - Hardware Write Protect (WP) Control Pin
    - Software Write Protect (WP) Configuration Bit
- Brown-out Reset Protection (1.5V typical)
- Serial Interface Inactive Current (2.5 uA typical)
- High-Voltage Tolerant Digital Inputs: Up to 12.5V
- Wide Operating Voltage:
    - 2.7V to 5.5V - Device Characteristics Specified
    - 1.8V to 5.5V - Device Operation
- Wide Bandwidth (-3dB) Operation:
    - 2 MHz (typ.) for 5.0 k Device
- Extended Temperature Range (-40°C to +125°C)

# Implementation methodology

This library is implemented in C and is platform independent. It means that we can easily add it to any project for any platform that has an I2C port and uses C/C++ language by implementing only the interfacing functions. Examples: STM32, ESP32, NXP, etc.

Implementation is done in a manner to bring high level functions to user in order to make it easy to use and abstract him of all hardware specificities. Also, implementation is robust with error handling and data validation.

The best way to understand the program is to read function headers. The file `mcp4562.h` contains all `enumeration`,`structure` and `function prototypes` with understandable names and functions headers are in the file `mcp4562.c`.

# How to use

To use the library, just add the files:
- `mcp4562.c`
- `mcp4562.h`
- `mcp4562_defines.h`

to your project and include the file `mcp4562.h` in your main file.

Then, declare a variable of type `MCP4562_HandleTypeDef` and init it with the function `MCP4562_ErrorType MCP4562_Init(MCP4562_HandleTypeDef *mcp4562, uint8_t i2c_address_offset);`. For that you must give the `address_offset` with the I2C address you set on your hardware. **Caution**, you must give the **offset** and not the address of the device.

You must then implement the following functions and add them to the handle:

- `MCP4562_ErrorType (*write)(uint8_t address, uint8_t *data, uint8_t bytes_to_send);`
- `MCP4562_ErrorType (*read)(uint8_t address, uint8_t *data, uint8_t bytes_to_read);`
- `MCP4562_ErrorType (*delay)(uint32_t ms);`
- `MCP4562_ErrorType (*software_reset)(); // Facultative, only necessary if you want to implement the software reset`

```C
// Example:

MCP4562_ErrorType platform_write(uint8_t address, uint8_t *data, uint8_t bytes_to_send){
    // Your platform dependent I2C implementation

}

// ...

void main(){

    MCP4562_HandleTypeDef myHandle;

    MCP4562_Init(&myHandle, 0);

    myHandle.write = platform_write;
    myHandle.read = //...
}

``` 

Those functions are the interface between the library and the hardware. Information about their implementation can be found in the file `mcp4562.h`.

Also, you must choose the model device and option you have in the file `mcp4562_defines.h` by commenting and uncommenting the one you have. This will allow you to use only the function that works for your device.

After that, you should be good to go and use the MCP4562 functions.

# Actual state

Implementation that needs to be done:
- General call commands
- High voltage commands
- manage correctly the discrimination between different devices of the same family