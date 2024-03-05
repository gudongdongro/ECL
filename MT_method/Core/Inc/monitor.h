//-------------------------------------------------------------------------
// File name : monitor.h
// <Abstract>
//
// Programmed by Prof. Lee on 12 Sep 2015
//-------------------------------------------------------------------------
#ifndef _MONITOR_H
#define _MONITOR_H

#include <stdint.h>

#define	BIT_MASK(x)			(1 << (x))
#define	ClearBit(val, bit)	((val) &= ~BIT_MASK(bit))
#define	ToggleBit(val, bit)	((val) ^=BIT_MASK(bit))
#define GetBit(val, bit)	(((val) & BIT_MASK(bit))>>(bit))
#define	SetBit(val, bit)	((val) |= BIT_MASK(bit))

#define SS_DOUBLE  0   //    64-bit double Çü data
#define SS_SINGLE  1   //    32-bit float Çü data
#define SS_INT8    2   // int8_T
#define SS_UINT8   3   // uint8_T
#define SS_INT16   4   // int16_T
#define SS_UINT16  5   // uint16_T
#define SS_INT32   6   // int32_T
#define SS_UINT32  7   // uint32_T
#define SS_BOOLEAN 8   // boolean_T

#define MAX_NUM_OF_TO_HOST_BLOCK 16

typedef struct {
    unsigned int ModNumber;
    unsigned int ModCntr;
    unsigned int NumOfToHostBlock;
    unsigned int NumOfDoubleData; 
    unsigned int UpdateDoneFlag;
    int SendHoldFlag; 
    char TypeAndChannel[MAX_NUM_OF_TO_HOST_BLOCK];
    void * DataPtr[MAX_NUM_OF_TO_HOST_BLOCK];
    uint8_t data[512];
    unsigned long indx;	
} ToHost;



void connectData(ToHost *toHostVar, char type, char channel, void *ptr);
void putcToBuffer(ToHost *toHostVar, char c);
void outhex8_Buffer(ToHost *toHostVar, char n);
void outhex32_Buffer(ToHost *toHostVar, unsigned long n);
void ProcessToHostBlock();

#endif 