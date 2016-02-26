#include "SI_C8051F330_Defs.h"
#include <stdint.h>
#include <stdbool.h>

#include "crc.h"

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

void enable_interrupts() {
_asm
    setb _IE_EA
_endasm;
}
void disable_interrupts() {
_asm
     clr _IE_EA
     clr _IE_EA
_endasm;
}

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


volatile uint8_t on_time = 100;
uint8_t off_time_long = 3;
volatile uint8_t revs = 0;

uint8_t rx_state = 0;

volatile __xdata uint8_t tx_buf[256];
volatile uint8_t tx_buf_write_pos = 0;
volatile uint8_t tx_buf_read_pos = 0;

volatile bit sending = 0;

void send_byte(uint8_t x) {
    if(tx_buf_write_pos + 1 == tx_buf_read_pos) return; // buffer full
    
    tx_buf[tx_buf_write_pos++] = x;
    
    if(!sending && tx_buf_read_pos != tx_buf_write_pos) {
        SBUF0 = tx_buf[tx_buf_read_pos++];
        sending = 1;
    }
}

void uart0_isr() __interrupt UART0_IRQn {
    if(SCON0_RI) {
        uint8_t c = SBUF0;
        SCON0_RI = 0;
        switch(rx_state) {
            case 0: if(c == 0x37) rx_state = 1; break;
            case 1: if(c == 0x19) rx_state = 2; else if(c == 0x37) rx_state = 1; else rx_state = 0; break;
            case 2: if(c == 0x15) rx_state = 3; else if(c == 0x37) rx_state = 1; else rx_state = 0; break;
            case 3: if(c == 0x3c) rx_state = 4; else if(c == 0x37) rx_state = 1; else rx_state = 0; break;
            case 4:
                if(c == 0) {
                    /*__critical {
                        send_byte(0xa4);
                        send_byte(0x76);
                        send_byte(0x6a);
                        send_byte(0x7f);
                        send_byte(revs);
                    }*/
                } else if(c == 255) {
                    RSTSRC = 0x10; // reset into bootloader
                    while(1);
                } else {
                    on_time = c;
                }
                
                rx_state = 0;
                break;
        }
    }
    if(SCON0_TI) {
        sending = 0;
        SCON0_TI = 0;
        if(!sending && tx_buf_read_pos != tx_buf_write_pos) {
            SBUF0 = tx_buf[tx_buf_read_pos++];
            sending = 1;
        }
    }
}

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
    
    CPT0CN = 0x80; // Comparator enabled, no hysteresis
    CPT0MD = 0x00; // Comparator response time 100ns
    
    SCON0_RI = 0;
    SCON0_REN = 1;
    IE_ES0 = 1;
    enable_interrupts();
    
    //while(true);
    
    while(true) {
        uint8_t count;
        uint8_t f;
        if(on_time == 1) continue;
                        //send_byte(0xa4);
                        //send_byte(0x76);
                        //send_byte(0x6a);
                        //send_byte(0x7f);
        Set_Comp_Phase_C;
        count = 255;
        for(f = 0; f < count; f++) {
            uint8_t res;
            ApFET_on;
            BnFET_on;
            delay(on_time);
            res = CPT0CN;
            ApFET_off;
            BnFET_off;
            longdelay(off_time_long);
            if((res & 0x40)) { break; }
        }
        //send_byte(f);
        Set_Comp_Phase_A;
        count = 255;
        for(f = 0; f < count; f++) {
            uint8_t res;
            BnFET_on;
            CpFET_on;
            delay(on_time);
            res = CPT0CN;
            BnFET_off;
            CpFET_off;
            longdelay(off_time_long);
            if(!(res & 0x40)) { break; }
        }
        //send_byte(f);
        Set_Comp_Phase_B;
        count = 255;
        for(f = 0; f < count; f++) {
            uint8_t res;
            AnFET_on;
            CpFET_on;
            delay(on_time);
            res = CPT0CN;
            AnFET_off;
            CpFET_off;
            longdelay(off_time_long);
            if((res & 0x40)) { break; }
        }
        //send_byte(f);
        Set_Comp_Phase_C;
        count = 255;
        for(f = 0; f < count; f++) {
            uint8_t res;
            AnFET_on;
            BpFET_on;
            delay(on_time);
            res = CPT0CN;
            AnFET_off;
            BpFET_off;
            longdelay(off_time_long);
            if(!(res & 0x40)) { break; }
        }
        //send_byte(f);
        Set_Comp_Phase_A;
        count = 255;
        for(f = 0; f < count; f++) {
            uint8_t res;
            BpFET_on;
            CnFET_on;
            delay(on_time);
            res = CPT0CN;
            BpFET_off;
            CnFET_off;
            longdelay(off_time_long);
            if((res & 0x40)) { break; }
        }
        //send_byte(f);
        Set_Comp_Phase_B;
        count = 255;
        for(f = 0; f < count; f++) {
            uint8_t res;
            ApFET_on;
            CnFET_on;
            delay(on_time);
            res = CPT0CN;
            ApFET_off;
            CnFET_off;
            longdelay(off_time_long);
            if(!(res & 0x40)) { break; }
        }
        //send_byte(f);
        
        revs++;
    }
}
