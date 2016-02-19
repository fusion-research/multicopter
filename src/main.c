#include "SI_C8051F330_Defs.h"
#include <stdint.h>
#include <stdbool.h>

// port 0
#define Mux_B 6
#define Rcp_In 5
#define Tx_Out 4
#define Comp_Com 3
#define Mux_A 2
#define Mux_C 0

// port 1
#define AnFET 7
#define ApFET 6
#define CnFET 5
#define CpFET 4
#define BnFET 3
#define BpFET 2
#define Adc_Ip 0

#define CAT(a, ...) PRIMITIVE_CAT(a, __VA_ARGS__)
#define PRIMITIVE_CAT(a, ...) a ## __VA_ARGS__

#define AnFET_on CAT(P1_B, AnFET) = 0
#define BnFET_on CAT(P1_B, BnFET) = 0
#define CnFET_on CAT(P1_B, CnFET) = 0
#define ApFET_on CAT(P1_B, ApFET) = 1
#define BpFET_on CAT(P1_B, BpFET) = 1
#define CpFET_on CAT(P1_B, CpFET) = 1

#define AnFET_off CAT(P1_B, AnFET) = 1
#define BnFET_off CAT(P1_B, BnFET) = 1
#define CnFET_off CAT(P1_B, CnFET) = 1
#define ApFET_off CAT(P1_B, ApFET) = 0
#define BpFET_off CAT(P1_B, BpFET) = 0
#define CpFET_off CAT(P1_B, CpFET) = 0

void switch_power_off() {
    AnFET_off;
    CnFET_off;
    BnFET_off;
    ApFET_off;
    BpFET_off;
    CpFET_off;
}

void delay(uint8_t ticks) {
_asm
    delayloop: djnz dpl, delayloop
_endasm;
}

void longdelay(uint8_t ticks) {
_asm
    clr acc
delayloop2:
    djnz acc, delayloop2
    djnz dpl, delayloop2
_endasm;
}

uint8_t on_time = 100;
uint8_t off_time_long = 3;

void main() {
    uint16_t speed = 400;
    
    PCA0MD &= ~0x40; // disable watchdog
    
    OSCICN |= 0x03; // set clock divider to 1
    
    P0 = 0xFF;
    P0MDOUT = (1 << Tx_Out);
    P0MDIN = ~((1 << Mux_A)+(1 << Mux_B)+(1 << Mux_C)+(1 << Comp_Com));
    P0SKIP = ~((1 << Rcp_In)+(1 << Tx_Out));
    
    P1 = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << Adc_Ip);
    P1MDOUT = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << ApFET)+(1 << BpFET)+(1 << CpFET);
    P1MDIN = ~(1 << Adc_Ip);
    P1SKIP = (1 << Adc_Ip);
    
    XBR0 = 0x01; // enable uart
    XBR1 = 0x40; // enable crossbar
    
    // configure uart
    TCON = 0x00;
    TMOD = 0x20;
    CKCON = 0x08;
    TL1 = 0;
    TH1 = 0x96;
    TCON = 0x40;
    SCON0 = 0x00;
    
    switch_power_off();
    
    while(true) {
        uint8_t count = (uint16_t)40000/speed;
        uint8_t f;
        for(f = 0; f < count; f++) {
            ApFET_on;
            BnFET_on;
            delay(on_time);
            ApFET_off;
            BnFET_off;
            longdelay(off_time_long);
        }
        for(f = 0; f < count; f++) {
            BnFET_on;
            CpFET_on;
            delay(on_time);
            BnFET_off;
            CpFET_off;
            longdelay(off_time_long);
        }
        for(f = 0; f < count; f++) {
            AnFET_on;
            CpFET_on;
            delay(on_time);
            AnFET_off;
            CpFET_off;
            longdelay(off_time_long);
        }
        for(f = 0; f < count; f++) {
            AnFET_on;
            BpFET_on;
            delay(on_time);
            AnFET_off;
            BpFET_off;
            longdelay(off_time_long);
        }
        for(f = 0; f < count; f++) {
            BpFET_on;
            CnFET_on;
            delay(on_time);
            BpFET_off;
            CnFET_off;
            longdelay(off_time_long);
        }
        for(f = 0; f < count; f++) {
            ApFET_on;
            CnFET_on;
            delay(on_time);
            ApFET_off;
            CnFET_off;
            longdelay(off_time_long);
        }
        if(speed <= 5000) speed += (count+3)/4;
        
        SBUF0 = 42;
        while(!SCON0_TI);
        SCON0_TI = 0;
        SBUF0 = speed >> 8;
        while(!SCON0_TI);
        SCON0_TI = 0;
        SBUF0 = speed & 255;
        while(!SCON0_TI);
        SCON0_TI = 0;
    }
}
