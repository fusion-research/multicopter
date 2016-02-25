#include "SI_C8051F330_Defs.h"
#include <stdint.h>
#include <stdbool.h>

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

// port 0
#define Rcp_In 5
#define Tx_Out 4

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

union {
    uint32_t as_uint32;
    uint8_t as_4_uint8[4];
} crc;
void crc_init(void) {
    crc.as_uint32 = 0xffffffff;
}
void crc_update(uint8_t byte) {
    int8_t j;
    crc.as_uint32 = crc.as_uint32 ^ byte;
    for(j = 7; j >= 0; j--) {
        uint32_t mask = -((crc.as_uint32 & 1));
        crc.as_uint32 = (crc.as_uint32 >> 1) ^ (0xEDB88320 & mask);
    }
}
void crc_finalize(void) {
    crc.as_uint32 = ~crc.as_uint32;
}

void send_and_crc(uint8_t byte) {
    send_byte(byte);
    crc_update(byte);
}

void main() {
    PCA0MD = 0x00; // disable watchdog
    
    OSCICN = 0xc3; // set clock divider to 1
    
    XBR0 = 0x01; // enable uart
    XBR1 = 0x40; // enable crossbar
    
    // configure uart
    TCON = 0x00;
    TMOD = 0x20;
    CKCON = 0x08;
    TH1 = 0x96;
    TCON = 0x40;
    SCON0 = 0x00;
    SCON0_TI = 1;
    
    while(true) {
        uint8_t c, buf[20], i;
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
        
        for(i = 0; i < 20; i++) {
            c = read_byte();
            buf[i] = c;
            crc_update(c);
        }
        crc_finalize();
        for(i = 3; i <= 3; i--) {
            if(read_byte() != crc.as_4_uint8[i]) goto got_nothing;
        }
        
        SCON0_REN = 0;
        
        if(buf[0] == 0) {
            uint8_t __code *ptr = (uint8_t __code *)(((uint16_t)buf[1]) << 9);
            crc_init();
            send_and_crc(0xe4);
            send_and_crc(0x8c);
            send_and_crc(0xf1);
            send_and_crc(0xcb);
            {
                uint16_t i;
                for(i = 0; i < 512; i++) {
                    send_and_crc(*ptr++);
                }
            }
            crc_finalize();
            for(i = 3; i <= 3; i--) {
                send_byte(crc.as_4_uint8[i]);
            }
            continue;
        } else if(buf[0] == 1) { // write
            uint8_t length = buf[1];
            uint16_t addr = ((((uint16_t)buf[2]) << 8) | buf[3]);
            if(length > 16) continue;
            if(addr < 0x200 || addr >= 0x2000 || addr + length > 0x1E00) continue;
            for(i = 0; i < length; i++) {
                PSCTL = 0x01; // PSWE = 1, PSEE = 0
                FLKEY = 0xA5;
                FLKEY = 0xF1;
                *(uint8_t __xdata *)addr = buf[4+i];
                PSCTL = 0x00; // PSWE = 0
                addr++;
            }
        } else if(buf[0] == 2) { // erase
            if(buf[1] < 2 || buf[1] >= 15) continue;
            PSCTL = 0x03; // PSWE = 1, PSEE = 1
            FLKEY = 0xA5;
            FLKEY = 0xF1;
            *(uint8_t __xdata *)(((uint16_t)buf[1]) << 9) = 0;
            PSCTL = 0x00;
        } else if(buf[0] == 3) { // run program
        } else {
            continue;
        }
        
        crc_init();
        send_and_crc(0xe4);
        send_and_crc(0x8c);
        send_and_crc(0xf1);
        send_and_crc(0xcb);
        for(i = 0; i < sizeof(buf); i++) {
            send_and_crc(buf[i]);
        }
        crc_finalize();
        for(i = 3; i <= 3; i--) {
            send_byte(crc.as_4_uint8[i]);
        }
        
        if(buf[0] == 3) { // run program
            ext_reset();
        }
    }
}
