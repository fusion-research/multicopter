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

#define P1_ALL_OFF ((1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << Adc_Ip))

const uint8_t commutation_pattern[12] = { // turns on one of pFETs
    P1_ALL_OFF + (1 << ApFET), // Ap
    P1_ALL_OFF + (1 << CpFET), // Cp
    P1_ALL_OFF + (1 << CpFET), // Cp
    P1_ALL_OFF + (1 << BpFET), // Bp
    P1_ALL_OFF + (1 << BpFET), // Bp
    P1_ALL_OFF + (1 << ApFET), // Ap
    
    P1_ALL_OFF + (1 << BpFET), // Bp
    P1_ALL_OFF + (1 << BpFET), // Bp
    P1_ALL_OFF + (1 << ApFET), // Ap
    P1_ALL_OFF + (1 << ApFET), // Ap
    P1_ALL_OFF + (1 << CpFET), // Cp
    P1_ALL_OFF + (1 << CpFET)  // Cp
};
const uint8_t commutation_pattern2[12] = { // turns on one of nFETs
    P1_ALL_OFF + (1 << ApFET) - (1 << BnFET), // Bn
    P1_ALL_OFF + (1 << CpFET) - (1 << BnFET), // Bn
    P1_ALL_OFF + (1 << CpFET) - (1 << AnFET), // An
    P1_ALL_OFF + (1 << BpFET) - (1 << AnFET), // An
    P1_ALL_OFF + (1 << BpFET) - (1 << CnFET), // Cn
    P1_ALL_OFF + (1 << ApFET) - (1 << CnFET), // Cn
    
    P1_ALL_OFF + (1 << BpFET) - (1 << AnFET), // An
    P1_ALL_OFF + (1 << BpFET) - (1 << CnFET), // Cn
    P1_ALL_OFF + (1 << ApFET) - (1 << CnFET), // Cn
    P1_ALL_OFF + (1 << ApFET) - (1 << BnFET), // Bn
    P1_ALL_OFF + (1 << CpFET) - (1 << BnFET), // Bn
    P1_ALL_OFF + (1 << CpFET) - (1 << AnFET)  // An
};
const uint8_t commutation_comp[12] = {
    0x10, 0x11, 0x13, 0x10, 0x11, 0x13, // C A B C A B
    0x10, 0x11, 0x13, 0x10, 0x11, 0x13  // C A B C A B
};

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

enum MODE {
    MODE_STEPPER = 1,
    MODE_NEGATIVE = 2, // apply negative voltage
    MODE_BACKWARDS = 4 // advance backwards through commutation pattern
};

struct command {
    uint8_t mode;
    uint16_t on_time; // clock cycles
    uint16_t cycles; // used in stepper mode. pwm cycles
};
volatile struct command cmd = { .mode = MODE_NEGATIVE|MODE_BACKWARDS, .on_time = 0 };
volatile uint16_t revs = 0;

volatile bit controller_active = 0;
volatile uint16_t controller_speed; // revs/s
volatile int32_t controller_integral; // 16.16 on_time units
volatile uint16_t controller_speed_measured;
volatile int16_t controller_output;

volatile __xdata uint8_t tx_buf[30];
volatile uint8_t tx_buf_write_pos = 0;
volatile uint8_t tx_buf_read_pos = 0;

#define INCREMENTED_MOD(x, m) (((x) == (m)-1) ? 0 : (x)+1)

volatile bit sending = 0;

void send_byte(uint8_t x) {
    if(INCREMENTED_MOD(tx_buf_write_pos, sizeof(tx_buf)) == tx_buf_read_pos) return; // buffer full
    
    tx_buf[tx_buf_write_pos] = x;
    tx_buf_write_pos = INCREMENTED_MOD(tx_buf_write_pos, sizeof(tx_buf));
    
    if(!sending && tx_buf_read_pos != tx_buf_write_pos) {
        P0MDOUT = (1 << Tx_Out);
        SBUF0 = tx_buf[tx_buf_read_pos];
        tx_buf_read_pos = INCREMENTED_MOD(tx_buf_read_pos, sizeof(tx_buf));
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
void send_escaped_byte_and_crc(uint8_t x) {
    send_escaped_byte(x);
    crc_update(x);
}

volatile bit in_escape = 0, in_message = 0;
volatile __xdata uint8_t rx_buf[20];
volatile uint8_t rx_buf_pos;

uint8_t __code *id_pointer = (uint8_t __code *)0x1c00;


uint16_t timer0_overflows = 0;
/*void timer0_isr() __interrupt TIMER0_IRQn {
    timer0_overflows++;
}*/
uint32_t read_timer0() { // can't be called from interrupts
    uint32_t res;
    bit done = 0;
    while(true) {
        if(TCON_TF0) {
            TCON_TF0 = 0;
            timer0_overflows++;
        }
        {
            uint8_t hi = TH0;
            uint8_t lo = TL0;
            uint8_t hi2 = TH0;
            if(hi2 != hi) continue;
            if(TCON_TF0) continue;
            res = (((uint32_t)timer0_overflows) << 16) | (((uint16_t)hi) << 8) | lo;
            break;
        }
    }
    return res;
}
uint32_t approximate_time;

volatile uint8_t __xdata capture_buf[256];
volatile uint16_t capture_pos = sizeof(capture_buf);

void handle_message() {
    if(rx_buf_pos >= 3 && rx_buf[0] == 2 && rx_buf[1] == *id_pointer) {
        if(rx_buf[2] == 0) {
            if(rx_buf_pos != 3) return;
            RSTSRC = 0x10; // reset into bootloader
            while(1);
        } else if(rx_buf[2] == 1) { // get status
            uint16_t my_revs;
            __critical {
                my_revs = revs;
            }
            if(rx_buf_pos != 3) return;
            send_byte(0xff);
            crc_init();
            send_byte(ESCAPE);
            send_byte(ESCAPE_START);
            send_escaped_byte_and_crc(3);
            send_escaped_byte_and_crc(*id_pointer);
            send_escaped_byte_and_crc(my_revs&0xff);
            send_escaped_byte_and_crc(my_revs>>8);
            send_escaped_byte_and_crc(controller_output&0xff);
            send_escaped_byte_and_crc(controller_output>>8);
            send_escaped_byte_and_crc((controller_speed_measured >>  0) & 0xff);
            send_escaped_byte_and_crc((controller_speed_measured >>  8) & 0xff);
            send_escaped_byte_and_crc((approximate_time >>  0) & 0xff);
            send_escaped_byte_and_crc((approximate_time >>  8) & 0xff);
            send_escaped_byte_and_crc((approximate_time >> 16) & 0xff);
            send_escaped_byte_and_crc((approximate_time >> 24) & 0xff);
            send_escaped_byte_and_crc((controller_integral >>  0) & 0xff);
            send_escaped_byte_and_crc((controller_integral >>  8) & 0xff);
            send_escaped_byte_and_crc((controller_integral >> 16) & 0xff);
            send_escaped_byte_and_crc((controller_integral >> 24) & 0xff);
            crc_finalize();
            { uint8_t i; for(i = 0; i < 4; i++) send_escaped_byte(crc.as_4_uint8[i]); }
            send_byte(ESCAPE);
            send_byte(ESCAPE_END);
        } else if(rx_buf[2] == 2) { // set power
            if(rx_buf_pos != 5) return;
            __critical {
                cmd.on_time = rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
            }
            controller_active = 0;
        } else if(rx_buf[2] == 3) { // start capture
            capture_pos = 0;
        } else if(rx_buf[2] == 4) { // read
            send_byte(0xff);
            crc_init();
            send_byte(ESCAPE);
            send_byte(ESCAPE_START);
            send_escaped_byte_and_crc(3);
            send_escaped_byte_and_crc(*id_pointer);
            send_escaped_byte_and_crc(capture_buf[((uint16_t)(rx_buf[3]) << 8) | rx_buf[4]]);
            crc_finalize();
            { uint8_t i; for(i = 0; i < 4; i++) send_escaped_byte(crc.as_4_uint8[i]); }
            send_byte(ESCAPE);
            send_byte(ESCAPE_END);
        } else if(rx_buf[2] == 5) { // set speed in rev/s 8.8
            if(rx_buf_pos != 5) return;
            controller_speed = rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
            if(!controller_active) {
                controller_integral = (int32_t)cmd.on_time << 16;
                controller_active = 1;
            }
        }
    }
}

void handle_byte(uint8_t c) {
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

void uart0_isr() __interrupt UART0_IRQn {
    if(SCON0_RI) {
        uint8_t c = SBUF0;
        SCON0_RI = 0;
        handle_byte(c);
    }
    if(SCON0_TI) {
        SCON0_TI = 0;
        // sending is 1
        if(tx_buf_read_pos != tx_buf_write_pos) {
            SBUF0 = tx_buf[tx_buf_read_pos];
            tx_buf_read_pos = INCREMENTED_MOD(tx_buf_read_pos, sizeof(tx_buf));
            sending = 1;
        } else {
            sending = 0;
            P0MDOUT = 0;
        }
    }
}

struct {
    uint16_t my_on_time;
    uint16_t my_off_time;
    uint8_t count;
    uint8_t f;
    uint8_t state;
    uint8_t res;
    uint8_t commutation_step;
    uint8_t negative;
} iv = { // isr variables
    .count = 255,
    .state = 0,
    .commutation_step = 0,
};
#define DELAY(n, length) PCA0CP0 = PCA0CP0 + (length); iv.state = n; return; case n:
void pca0_isr() __interrupt PCA0_IRQn {
    PCA0CN_CCF0 = 0;
    
    switch(iv.state) { case 0:
        while(true) {
            iv.my_on_time = cmd.on_time;
            if(iv.my_on_time < 100 || iv.my_on_time > 1000) {
                DELAY(1, 1225)
                continue;
            }
            iv.my_off_time = 1225 - iv.my_on_time;
            
            if(cmd.mode & MODE_NEGATIVE) {
                iv.commutation_step += 6;
                iv.negative = 1;
            } else iv.negative = 0;
            
            CPT0MX = commutation_comp[iv.commutation_step];
            P1 = commutation_pattern[iv.commutation_step];
            for(iv.f = 0; iv.f < iv.count; iv.f++) {
                DELAY(3, iv.my_off_time)
                P1 = commutation_pattern2[iv.commutation_step];
                DELAY(2, iv.my_on_time)
                iv.res = CPT0CN;
                P1 = commutation_pattern[iv.commutation_step];
                if(((iv.res & 0x40) >> 6) ^ (iv.commutation_step & 1)) { break; }
            }
            P1 = P1_ALL_OFF;
            
            if(iv.negative) iv.commutation_step -= 6;
            
            if(cmd.mode & MODE_BACKWARDS) {
                revs--;
                iv.commutation_step--;
                if(iv.commutation_step == 255) iv.commutation_step = 5;
            } else {
                revs++;
                iv.commutation_step++;
                if(iv.commutation_step == 6) iv.commutation_step = 0;
            }
        }
    }
}

uint32_t last_controller_time;
void main() {
    uint16_t last_revs;
    bit last_revs_present = 0;
    
    PCA0MD &= ~0x40; // disable watchdog
    
    OSCICN |= 0x03; // set clock divider to 1
    
    P0 = 0xFF;
    P0MDOUT = 0;
    P0MDIN = ~((1 << Mux_A)+(1 << Mux_B)+(1 << Mux_C)+(1 << Comp_Com));
    P0SKIP = ~((1 << Rcp_In)+(1 << Tx_Out));
    
    P1 = P1_ALL_OFF;
    P1MDOUT = (1 << AnFET)+(1 << BnFET)+(1 << CnFET)+(1 << ApFET)+(1 << BpFET)+(1 << CpFET);
    P1MDIN = ~(1 << Adc_Ip);
    P1SKIP = (1 << Adc_Ip);
    
    XBR0 = 0x01; // enable uart
    XBR1 = 0xc0; // disable pullups, enable crossbar
    
    // configure uart
    TMOD = (TMOD & ~0xF0) | 0x20;
    CKCON |= 0x08;
    TL1 = 0;
    TH1 = 0x96;
    TCON_TR1 = 1;
    SCON0 = 0x00;
    
    CPT0CN = 0x80; // Comparator enabled, no hysteresis
    CPT0MD = 0x00; // Comparator response time 100ns
    
    SCON0_RI = 0;
    SCON0_REN = 1;
    IE_ES0 = 1;
    
    REF0CN = 0b1110;
    ADC0CF = (8 << 3) | 0b100;
    ADC0CN_ADEN = 1;
    AMX0N = 0b00011;// N = P0.3 = Comp_Com
    //AMX0N = 0b10001;
    AMX0P = 0b00010; // P = P0.2 = Mux_A
    
    // start timebase
    TMOD = (TMOD & ~0x0F) | 0x01;
    CKCON |= 0x04; // system clock
    //CKCON = (CKCON & ~0b111) | 0b010; // system clock / 48
    TL0 = 0;
    TH0 = 0;
    TCON_TF0 = 0; // clear overflow flag
    //IE_ET0 = 1; // enable overflow interrupt
    //IP_PT0 = 1; // high priority XXX has to be, otherwise race condition with other high priority
    TCON_TR0 = 1; // run
    
    /*
    CKCON |= 0x10; // timer 2 on system clock
    TMR2CN = 0;
    TMR2RLL = 0;
    TMR2RLH = 0;
    TMR2L = 0;
    TMR2H = 0;
    TMR2CN_TR2 = 1;
    IP_PT2 = 1; // high priority
    IE_ET2 = 1;
    */
    
    // PCA setup
    PCA0MD = 0b00001000; // use system clock
    PCA0CPM0 = 0b01001001;
    EIE1 |= 0x10; // enable interrupt
    EIP1 |= 0x10; // high priority
    PCA0CN_CR = 1; // run
    
    approximate_time = read_timer0();
    
    enable_interrupts();
    
    last_controller_time = read_timer0();
    
    while(true) {
        approximate_time = read_timer0();
        if(TCON_TF0) {
            TCON_TF0 = 0;
            timer0_overflows++;
        }
        if(!ADC0CN_ADBUSY) {
            if(capture_pos < sizeof(capture_buf)) {
                capture_buf[capture_pos++] = ADC0H;
            }
            ADC0CN_ADBUSY = 1;
        }
        while(read_timer0() - last_controller_time >= 95703) { // run at 256 Hz
            last_controller_time += 95703;
            if(!controller_active) continue;
            {
                uint16_t my_revs;
                __critical {
                    my_revs = revs;
                }
                {
                    int16_t est = last_revs_present ? my_revs - last_revs : 0; // revs/s / 256
                    int16_t err = (int16_t)controller_speed - ((int16_t)est << 8); // revs/s
                    int16_t u = (controller_integral >> 16) + (err >> 4);
                    last_revs = my_revs;
                    last_revs_present = 1;
                    controller_speed_measured = est << 8;
                    controller_integral += (int32_t)err << 10;
                    if(controller_integral < (int32_t)-1000<<16) controller_integral = (int32_t)-1000<<16;
                    if(controller_integral > (int32_t)1000<<16) controller_integral = (int32_t)1000<<16;
                    controller_output = u;
                    
                    {
                        struct command my_cmd = {.mode = 0};
                        if(u < 0) {
                            my_cmd.mode |= MODE_NEGATIVE;
                            u = -u;
                        }
                        my_cmd.on_time = 100 + u;
                        if(my_cmd.on_time > 1000) {
                            my_cmd.on_time = 1000;
                        }
                        __critical {
                            if(controller_active) {
                                cmd.mode = my_cmd.mode;
                                cmd.on_time = my_cmd.on_time;
                            }
                        }
                    }
                }
            }
        }
    }
}
