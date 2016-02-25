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
    PCA0MD &= ~0x40; // disable watchdog
    
    OSCICN |= 0x03; // set clock divider to 1
    
    P0 = 0xFF;
    P0MDOUT = 0;
    P0MDIN = 0xFF;
    P0SKIP = ~((1 << Rcp_In)+(1 << Tx_Out));
    
    P1 = 0xFF;
    P1MDOUT = 0x00;
    P1MDIN = 0xFF;
    P1SKIP = 0xFF;
    
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
    SCON0_TI = 1;
    
    while(true) {
        uint8_t c, buf[20], i;
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
        
        read_byte(); read_byte(); read_byte(); read_byte();
        /*if(read_byte() != crc.as_4_uint8[3]) goto got_nothing;
        if(read_byte() != crc.as_4_uint8[2]) goto got_nothing;
        if(read_byte() != crc.as_4_uint8[1]) goto got_nothing;
        if(read_byte() != crc.as_4_uint8[0]) goto got_nothing;*/
        SCON0_REN = 0;
        
        if(c == buf[0]) {
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
            {
                send_byte(crc.as_4_uint8[3]);
                send_byte(crc.as_4_uint8[2]);
                send_byte(crc.as_4_uint8[1]);
                send_byte(crc.as_4_uint8[0]);
            }
        }
    }
}
