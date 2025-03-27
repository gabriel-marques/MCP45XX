#ifndef _MCP4562_DEFINES_H
#define _MCP4562_DEFINES_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>

// MCP45xx is for 1 potentiometer, MCP46xx is for 2 potentiometers
// MCP4xx1 is for potentiometer, MCP4xx2 is for rheostat
// MCP4x3x and MCP4x5x are for RAM memory, MCP4x4x and MCP4x6x are for EEPROM memory
// MCP4x3x and MCP4x4x have 129 steps, MCP4x5x and MCP4x6x have 257 steps

// Choose the device you want to use

// DEVICES      //  | # of Pots | wiper config  | MEM type | WiperLock | POR Wiper setting | # of steps
// #define MCP4531  // | 1         | Potentiometer | RAM      | No        | Mid-scale         | 129
// #define MCP4532  // | 1         | Rheostat      | RAM      | No        | Mid-scale         | 129
// #define MCP4541  // | 1         | Potentiometer | EE       | Yes       | NV Wiper          | 129
// #define MCP4542  // | 1         | Rheostat      | EE       | Yes       | NV Wiper          | 129
// #define MCP4551  // | 1         | Potentiometer | RAM      | No        | Mid-scale         | 257
// #define MCP4552  // | 1         | Rheostat      | RAM      | No        | Mid-scale         | 257
// #define MCP4561  // | 1         | Potentiometer | EE       | Yes       | NV Wiper          | 257
#define MCP4562 // | 1         | Rheostat      | EE       | Yes       | NV Wiper          | 257
// #define MCP4631  // | 2         | Potentiometer | RAM      | No        | Mid-scale         | 129
// #define MCP4632  // | 2         | Rheostat      | RAM      | No        | Mid-scale         | 129
// #define MCP4641  // | 2         | Potentiometer | EE       | Yes       | NV Wiper          | 129
// #define MCP4642  // | 2         | Rheostat      | EE       | Yes       | NV Wiper          | 129
// #define MCP4651  // | 2         | Potentiometer | RAM      | No        | Mid-scale         | 257
// #define MCP4652  // | 2         | Rheostat      | RAM      | No        | Mid-scale         | 257
// #define MCP4661  // | 2         | Potentiometer | EE       | Yes       | NV Wiper          | 257
// #define MCP4662  // | 2         | Rheostat      | EE       | Yes       | NV Wiper          | 257

// Resistance options -> choose the one you have
#define MCP4562_OPTION_502 // 5k Ohm option 20 % tolerance
// #define MCP4562_OPTION_103  // 10k Ohm option 20 % tolerance
// #define MCP4562_OPTION_503  // 50k Ohm option 20 % tolerance
// #define MCP4562_OPTION_104  // 100k Ohm option 20 % tolerance

#endif // _MCP4562_DEFINES_H
