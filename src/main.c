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

#define Set_Comp_Phase_A CPT0MX = 0x11
#define Set_Comp_Phase_B CPT0MX = 0x13
#define Set_Comp_Phase_C CPT0MX = 0x10

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

uint8_t on_time = 200;
uint8_t off_time_long = 6;

void main() {
    PCA0MD &= ~0x40; // disable watchdog
    
    OSCICN |= 0x03; // set clock divider to 1
    
    P0 = 0xFF;
    P0MDOUT = 0;//(1 << Tx_Out);
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
    
    // wait for byte
    SCON0_RI = 0;
    SCON0_REN = 1;
    while(!SCON0_RI);
    SCON0_REN = 0;
    SCON0_RI = 0;
    
    P0MDOUT = (1 << Tx_Out);
    
    CPT0CN = 0x80; // Comparator enabled, no hysteresis
    CPT0MD = 0x00; // Comparator response time 100ns
    
    SBUF0 = 42;
    
    while(true) {
        uint8_t set_count;
        uint8_t count;
        uint8_t f;
        uint8_t cmps;
        cmps = 0;
        Set_Comp_Phase_C;
        count = 100; set_count = 0;
        for(f = 0; f < count; f++) {
            uint8_t res;
            ApFET_on;
            BnFET_on;
            delay(on_time);
            res = CPT0CN;
            ApFET_off;
            BnFET_off;
            cmps <<= 1;
            if(res & 0x40) cmps |= 1;
            if(!(res & 0x40) && !set_count) { count = 0; set_count = 1; }
            longdelay(off_time_long);
        }
        SBUF0 = cmps;
        cmps = 0;
        Set_Comp_Phase_A;
        count = 100; set_count = 0;
        for(f = 0; f < count; f++) {
            uint8_t res;
            BnFET_on;
            CpFET_on;
            delay(on_time);
            res = CPT0CN;
            BnFET_off;
            CpFET_off;
            cmps <<= 1;
            if(res & 0x40) cmps |= 1;
            if(!(res & 0x40) && !set_count) { count = 0; set_count = 1; }
            longdelay(off_time_long);
        }
        SBUF0 = cmps;
        cmps = 0;
        Set_Comp_Phase_B;
        count = 100; set_count = 0;
        for(f = 0; f < count; f++) {
            uint8_t res;
            AnFET_on;
            CpFET_on;
            delay(on_time);
            res = CPT0CN;
            AnFET_off;
            CpFET_off;
            cmps <<= 1;
            if(res & 0x40) cmps |= 1;
            if(!(res & 0x40) && !set_count) { count = 0; set_count = 1; }
            longdelay(off_time_long);
        }
        SBUF0 = cmps;
        cmps = 0;
        Set_Comp_Phase_C;
        count = 100; set_count = 0;
        for(f = 0; f < count; f++) {
            uint8_t res;
            AnFET_on;
            BpFET_on;
            delay(on_time);
            res = CPT0CN;
            AnFET_off;
            BpFET_off;
            cmps <<= 1;
            if(res & 0x40) cmps |= 1;
            if(!(res & 0x40) && !set_count) { count = 0; set_count = 1; }
            longdelay(off_time_long);
        }
        SBUF0 = cmps;
        cmps = 0;
        Set_Comp_Phase_A;
        count = 100; set_count = 0;
        for(f = 0; f < count; f++) {
            uint8_t res;
            BpFET_on;
            CnFET_on;
            delay(on_time);
            res = CPT0CN;
            BpFET_off;
            CnFET_off;
            cmps <<= 1;
            if(res & 0x40) cmps |= 1;
            if(!(res & 0x40) && !set_count) { count = 0; set_count = 1; }
            longdelay(off_time_long);
        }
        SBUF0 = cmps;
        cmps = 0;
        Set_Comp_Phase_B;
        count = 100; set_count = 0;
        for(f = 0; f < count; f++) {
            uint8_t res;
            ApFET_on;
            CnFET_on;
            delay(on_time);
            res = CPT0CN;
            ApFET_off;
            CnFET_off;
            cmps <<= 1;
            if(res & 0x40) cmps |= 1;
            if(!(res & 0x40) && !set_count) { count = 0; set_count = 1; }
            longdelay(off_time_long);
        }
        SBUF0 = cmps;
    }
}
