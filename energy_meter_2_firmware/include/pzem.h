#pragma once
// ================================================================
// pzem.h 
// ================================================================
#include <Arduino.h>

struct PzemReading {
    float   voltage;      
    float   current;     
    float   power;         
    float   energy;        
    float   frequency;     
    float   power_factor;  
    bool    alarm;         
    bool    valid;         
};


void pzem_init();
PzemReading pzem_read(uint8_t slaveAddr, uint8_t channel);
void pzem_read_all(PzemReading readings[3]);
bool pzem_set_address(uint8_t channel, uint8_t oldAddr, uint8_t newAddr);
