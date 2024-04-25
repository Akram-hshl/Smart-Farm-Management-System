////////////////////////////////////////////////////////////////////////////////
//// Name of Student :       
//// Developer:   Group J  
//
//// Description:
//      Smart Farm management system
//
//// Dependencies:
//      avr/io.h
//
//      File-ID: utility.h
//      Created on: 2023
//////////////////////////////////////////////////////////////////////////////// 
#ifndef UTILITY_H_
#define UTILITY_H_

#include <avr/io.h>

//#define ERROR 0xEE
#define MASK(x) ((uint8_t)(1 << x))
//#define MASK(x) (1 << x)
// Bit manipulation macros
#define SET_BIT(reg, bit)          ((reg) |= MASK(bit))
#define CLEAR_BIT(reg, bit)        ((reg) &= ~MASK(bit))




#endif