#include "SI_C8051F330_Defs.h"
#include <stdint.h>
#include <stdbool.h>

#include "crc.h"

#ifndef UPGRADER
void ext_reset();
void ext_isr0() __interrupt 0;
void ext_isr1() __interrupt 1;
void ext_isr2() __interrupt 2;
void ext_isr3() __interrupt 3;
void ext_isr4() __interrupt 4;
void ext_isr5() __interrupt 5;
void ext_isr6() __interrupt 6;
void ext_isr7() __interrupt 7;
void ext_isr8() __interrupt 8;
void ext_isr9() __interrupt 9;
void ext_isr10() __interrupt 10;
void ext_isr11() __interrupt 11;
void ext_isr12() __interrupt 12;
void ext_isr13() __interrupt 13;
void ext_isr14() __interrupt 14;
#endif

// port 0
#define Rcp_In 5
#define Tx_Out 4

// port 1
#define AnFET 7
#define ApFET 6
#define CnFET 5
#define CpFET 4
#define BnFET 3
#define BpFET 2
#define Adc_Ip 0

uint8_t read_byte() {
    uint8_t res;
    while(!SCON0_RI);
    res = SBUF0;
    SCON0_RI = 0;
    return res;
}
void send_byte(uint8_t x) {
    while(!SCON0_TI);
    SCON0_TI = 0;
    SBUF0 = x;
}

uint8_t __code *id_pointer = (uint8_t __code *)0x1c00;
const uint8_t type_byte = 
#ifndef UPGRADER
    0
#else
    4
#endif
;

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
void send_escaped_byte_and_crc(uint8_t byte) {
    send_escaped_byte(byte);
    crc_update(byte);
}
void start_tx() {
    crc_init();
    P0MDOUT = (1 << Tx_Out);
    send_byte(0xff);
    send_byte(ESCAPE);
    send_byte(ESCAPE_START);
    send_escaped_byte_and_crc(type_byte + 1);
    send_escaped_byte_and_crc(*id_pointer);
}
void end_tx() {
    uint8_t i;
    crc_finalize();
    for(i = 0; i < 4; i++) send_escaped_byte(crc.as_4_uint8[i]);
    send_byte(ESCAPE);
    send_byte(ESCAPE_END);
    while(!SCON0_TI);
    P0MDOUT = 0;
}

volatile __xdata uint8_t rx_buf[255];
volatile uint8_t rx_buf_pos;

void handle_message() {
    if(rx_buf[0] != type_byte) return;
    if(rx_buf[1] != *id_pointer) return;
    
    if(rx_buf[2] == 0) { // read page
        uint8_t __code *ptr = (uint8_t __code *)(((uint16_t)rx_buf[3]) << 9);
        start_tx();
        {
            uint16_t i;
            for(i = 0; i < 512; i++) {
                send_escaped_byte_and_crc(*ptr++);
            }
        }
        end_tx();
        return;
    } else if(rx_buf[2] == 1) { // write string
        uint8_t i;
        uint8_t length = rx_buf[3];
        uint16_t addr = ((((uint16_t)rx_buf[4]) << 8) | rx_buf[5]);
        if(length > 16) return;
        #ifndef UPGRADER
            if(addr < 0x800 || addr >= 0x2000 || addr + length > 0x1c00) return;
        #else
            if(addr >= 0x2000 || addr + length > 0x800) return;
        #endif
        for(i = 0; i < length; i++) {
            PSCTL = 0x01; // PSWE = 1, PSEE = 0
            FLKEY = 0xA5;
            FLKEY = 0xF1;
            *(uint8_t __xdata *)addr = rx_buf[6+i];
            PSCTL = 0x00; // PSWE = 0
            addr++;
        }
    } else if(rx_buf[2] == 2) { // erase page
        #ifndef UPGRADER
            if(rx_buf[3] < 4 || rx_buf[3] >= 14) return;
        #else
            if(rx_buf[3] >= 4) return;
        #endif
        PSCTL = 0x03; // PSWE = 1, PSEE = 1
        FLKEY = 0xA5;
        FLKEY = 0xF1;
        *(uint8_t __xdata *)(((uint16_t)rx_buf[3]) << 9) = 0;
        PSCTL = 0x00;
    } else if(rx_buf[2] == 3) { // run program
    } else {
        return;
    }
    
    start_tx();
    {
        uint8_t i;
        for(i = 2; i < rx_buf_pos; i++) {
            send_escaped_byte_and_crc(rx_buf[i]);
        }
    }
    end_tx();
    
    if(rx_buf[2] == 3) { // run program
        #ifndef UPGRADER
            ext_reset();
        #else
            RSTSRC = 0x10; // reset into bootloader
            while(1);
        #endif
    }
}

int _sdcc_external_startup() {
    PCA0MD = 0x00; // disable watchdog
    
    OSCICN = 0xc3; // set clock divider to 1
    
    // disable FETs
    P1 = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << Adc_Ip);
    P1MDOUT = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << ApFET)+(1 << BpFET)+(1 << CpFET);
    P1MDIN = ~(1 << Adc_Ip);
    P1SKIP = (1 << Adc_Ip);
    
    XBR0 = 0x01; // enable uart
    XBR1 = 0xc0; // disable pullups, enable crossbar
    
    return 0; // do normal initialization of static/global variables
}

void main() {
    bit in_escape = 0, in_message = 0;
    
    // configure uart
    TCON = 0x00;
    TMOD = 0x20;
    CKCON = 0x08;
    TH1 = 0x96;
    TCON = 0x40;
    SCON0 = 0x00;
    SCON0_TI = 1;
    SCON0_REN = 1;
    
    while(true) {
        uint8_t c;
        while(!SCON0_RI);
        c = SBUF0;
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
                    handle_message();
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
}
