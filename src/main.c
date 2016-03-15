#include "SI_C8051F330_Defs.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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


volatile uint8_t on_time = 1;
uint8_t off_time_long = 3;
volatile uint8_t revs = 0;

volatile __xdata uint8_t tx_buf[256];
volatile uint8_t tx_buf_write_pos = 0;
volatile uint8_t tx_buf_read_pos = 0;

volatile bit sending = 0;

void send_byte(uint8_t x) {
    if(tx_buf_write_pos + 1 == tx_buf_read_pos) return; // buffer full
    
    tx_buf[tx_buf_write_pos++] = x;
    
    if(!sending && tx_buf_read_pos != tx_buf_write_pos) {
        P0MDOUT = (1 << Tx_Out);
        SBUF0 = tx_buf[tx_buf_read_pos++];
        sending = 1;
    }
}

#define ESCAPE        0x34
#define ESCAPE_START  0x01
#define ESCAPE_END    0x02
#define ESCAPE_ESCAPE 0x03

void send_escaped_byte(uint8_t x) {
    if(x == ESCAPE) {
        send_byte(ESCAPE);
        send_byte(ESCAPE_ESCAPE);
    } else {
        send_byte(x);
    }
}

volatile bit in_escape = 0, in_message = 0;
volatile __xdata uint8_t rx_buf[255];
volatile uint8_t rx_buf_pos;

uint8_t __code *id_pointer = (uint8_t __code *)0x1c00;

void uart0_isr() __interrupt UART0_IRQn {
    if(SCON0_RI) {
        uint8_t c = SBUF0;
        SCON0_RI = 0;
        if(c == ESCAPE) {
            if(in_escape) {
                in_message = 0; // shouldn't happen, reset
            }
            in_escape = 1;
        } else if(in_escape) {
            in_escape = 0;
            if(c == ESCAPE_START) {
                in_message = 1;
                rx_buf_pos = 0;
                crc_init();
            } else if(c == ESCAPE_ESCAPE && in_message) {
                if(rx_buf_pos == sizeof(rx_buf)) { // overrun
                    in_message = 0;
                }
                rx_buf[rx_buf_pos++] = ESCAPE;
                crc_update(ESCAPE);
            } else if(c == ESCAPE_END && in_message) {
                // do something with rx_buf and rx_buf_pos
                crc_finalize();
                if(rx_buf_pos >= 4 && crc_good()) {
                    rx_buf_pos -= 4;
                    if(rx_buf_pos == 3 && rx_buf[0] == 2 && rx_buf[1] == *id_pointer) {
                        if(rx_buf[2] == 0) {
                            RSTSRC = 0x10; // reset into bootloader
                            while(1);
                        } else if(rx_buf[2] == 1) {
                            __critical {
                                uint8_t i;
                                send_byte(0xff);
                                crc_init();
                                send_byte(ESCAPE);
                                send_byte(ESCAPE_START);
                                send_escaped_byte(3); crc_update(3);
                                send_escaped_byte(*id_pointer); crc_update(*id_pointer);
                                send_escaped_byte(revs); crc_update(revs);
                                send_escaped_byte(rx_buf_pos); crc_update(rx_buf_pos);
                                crc_finalize();
                                for(i = 0; i < 4; i++) send_escaped_byte(crc.as_4_uint8[i]);
                                send_byte(ESCAPE);
                                send_byte(ESCAPE_END);
                            }
                        } else if(rx_buf[2] == 2) {
                            on_time = rx_buf[3];
                        }
                    }
                }
                in_message = 0;
            } else {
                in_message = 0; // shouldn't happen, reset
            }
        } else {
            if(in_message) {
                if(rx_buf_pos == sizeof(rx_buf)) { // overrun
                    in_message = 0;
                }
                rx_buf[rx_buf_pos++] = c;
                crc_update(c);
            } else {
                // shouldn't happen, ignore
            }
        }
    }
    if(SCON0_TI) {
        SCON0_TI = 0;
        // sending is 1
        if(tx_buf_read_pos != tx_buf_write_pos) {
            SBUF0 = tx_buf[tx_buf_read_pos++];
            sending = 1;
        } else {
            sending = 0;
            P0MDOUT = 0;
        }
    }
}

void main() {
    PCA0MD &= ~0x40; // disable watchdog
    
    OSCICN |= 0x03; // set clock divider to 1
    
    P0 = 0xFF;
    P0MDOUT = 0;
    P0MDIN = ~((1 << Mux_A)+(1 << Mux_B)+(1 << Mux_C)+(1 << Comp_Com));
    P0SKIP = ~((1 << Rcp_In)+(1 << Tx_Out));
    
    P1 = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << Adc_Ip);
    P1MDOUT = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << ApFET)+(1 << BpFET)+(1 << CpFET);
    P1MDIN = ~(1 << Adc_Ip);
    P1SKIP = (1 << Adc_Ip);
    
    XBR0 = 0x01; // enable uart
    XBR1 = 0xc0; // disable pullups, enable crossbar
    
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
        uint8_t count = 255;
        uint8_t f;
        if(on_time == 1) continue;
        Set_Comp_Phase_C;
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
        Set_Comp_Phase_A;
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
        Set_Comp_Phase_B;
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
        Set_Comp_Phase_C;
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
        Set_Comp_Phase_A;
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
        Set_Comp_Phase_B;
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
        
        revs++;
    }
}
