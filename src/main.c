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
    delayloop2: djnz acc, delayloop2
    djnz dpl, delayloop2
_endasm;
}

void main() {
    PCA0MD &= ~0x40; // disable watchdog
    
    OSCICN |= 0x03; // set clock divider to 1
    
    if(!(OSCLCN & 1)) {
        if(OSCICL + 4 < 128) {
            OSCICL = OSCICL + 4;
            
            OSCLCN |= 1;
        }
    }
    
    P0 = 0xFF;
    P0MDOUT = 0;
    P0MDIN = ~((1 << Mux_A)+(1 << Mux_B)+(1 << Mux_C)+(1 << Comp_Com));
    P0SKIP = ~((1 << Rcp_In)+(1 << Tx_Out));
    
    P1 = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << Adc_Ip);
    P1MDOUT = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << ApFET)+(1 << BpFET)+(1 << CpFET);
    P1MDIN = ~(1 << Adc_Ip);
    P1SKIP = (1 << Adc_Ip);
    
    XBR0 = 0x01;
    XBR1 = 0x40;
    
    switch_power_off();
    
    while(true) {
        uint8_t f;
        for(f = 1; f < 255; f++) {
            AnFET_on;
            BpFET_on;
            delay(f);
            AnFET_off;
            BpFET_off;
            
            ApFET_on;
            BnFET_on;
            delay(f);
            ApFET_off;
            BnFET_off;
            
            longdelay(f);
        }
    }
    
    TCON = 0x00;
    TMOD = 0x20;
    CKCON = 0x08;
    TL1 = 0;
    TH1 = 0x98;
    TCON = 0x40;
    SCON0 = 0x00;
    
    while(true) {
        uint8_t b;
        
        SCON0_TI = 0;
        SBUF0 = 170;
        while(!SCON0_TI);
        SCON0_TI = 0;
        continue;
        
        
        SCON0_RI = 0;
        SCON0_REN = 1;
        while(!SCON0_RI);
        b = SBUF0;
        SCON0_REN = 0;
        SCON0_RI = 0;
        
        b++;
        
        SCON0_TI = 0;
        SBUF0 = b;
        while(!SCON0_TI);
        SCON0_TI = 0;
    }
}
