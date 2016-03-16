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

void send_and_crc(uint8_t byte) {
    send_byte(byte);
    crc_update(byte);
}

void start_tx() {
    crc_init();
    P0MDOUT = (1 << Tx_Out);
    send_byte(0xff);
    send_and_crc(0xe4);
    send_and_crc(0x8c);
    send_and_crc(0xf1);
    send_and_crc(0xcb);
}
void end_tx() {
    uint8_t i;
    crc_finalize();
    for(i = 3; i <= 3; i--) {
        send_byte(crc.as_4_uint8[i]);
    }
    while(!SCON0_TI);
    P0MDOUT = 0;
}

uint8_t __code *id_pointer = (uint8_t __code *)0x1c00;

void main() {
    PCA0MD = 0x00; // disable watchdog
    
    OSCICN = 0xc3; // set clock divider to 1
    
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
    TH1 = 0x96;
    TCON = 0x40;
    SCON0 = 0x00;
    SCON0_TI = 1;
    
    while(true) {
        uint8_t c, buf[21], i;
        while(!SCON0_TI); // wait for any transmissions to finish
        SCON0_RI = 0;
        SCON0_REN = 1;
got_nothing:
        if(read_byte() != 0xd8) goto got_nothing;
got_first:
        crc_init();
        crc_update(0xd8);
        
        c = read_byte();
        if(c == 0xd8) goto got_first;
        else if(c != 0x30) goto got_nothing;
        crc_update(0x30);
        
        c = read_byte();
        if(c == 0xd8) goto got_first;
        else if(c != 0x03) goto got_nothing;
        crc_update(0x03);
        
        c = read_byte();
        if(c == 0xd8) goto got_first;
        else if(c != 0x7d) goto got_nothing;
        crc_update(0x7d);
        
        for(i = 0; i < 21; i++) {
            c = read_byte();
            buf[i] = c;
            crc_update(c);
        }
        crc_finalize();
        for(i = 3; i <= 3; i--) {
            if(read_byte() != crc.as_4_uint8[i]) goto got_nothing;
        }
        
        SCON0_REN = 0;
        
        if(buf[20] != *id_pointer) continue;
        
        if(buf[0] == 0) { // read page
            uint8_t __code *ptr = (uint8_t __code *)(((uint16_t)buf[1]) << 9);
            start_tx();
            {
                uint16_t i;
                for(i = 0; i < 512; i++) {
                    send_and_crc(*ptr++);
                }
            }
            end_tx();
            continue;
        } else if(buf[0] == 1) { // write string
            uint8_t length = buf[1];
            uint16_t addr = ((((uint16_t)buf[2]) << 8) | buf[3]);
            if(length > 16) continue;
            #ifndef UPGRADER
                if(addr < 0x400 || addr >= 0x2000 || addr + length > 0x1E00) continue;
            #endif
            for(i = 0; i < length; i++) {
                PSCTL = 0x01; // PSWE = 1, PSEE = 0
                FLKEY = 0xA5;
                FLKEY = 0xF1;
                *(uint8_t __xdata *)addr = buf[4+i];
                PSCTL = 0x00; // PSWE = 0
                addr++;
            }
        } else if(buf[0] == 2) { // erase page
            #ifndef UPGRADER
                if(buf[1] < 2 || buf[1] >= 14) continue;
            #endif
            PSCTL = 0x03; // PSWE = 1, PSEE = 1
            FLKEY = 0xA5;
            FLKEY = 0xF1;
            *(uint8_t __xdata *)(((uint16_t)buf[1]) << 9) = 0;
            PSCTL = 0x00;
        } else if(buf[0] == 3) { // run program
        } else {
            continue;
        }
        
        start_tx();
        for(i = 0; i < sizeof(buf); i++) {
            send_and_crc(buf[i]);
        }
        end_tx();
        
        if(buf[0] == 3) { // run program
            #ifndef UPGRADER
                ext_reset();
            #else
                RSTSRC = 0x10; // reset into bootloader
                while(1);
            #endif
        }
    }
}
