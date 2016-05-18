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

const uint8_t commutation_pattern[6] = { // turns on one of pFETs
    P1_ALL_OFF + (1 << ApFET), // Ap
    P1_ALL_OFF + (1 << CpFET), // Cp
    P1_ALL_OFF + (1 << CpFET), // Cp
    P1_ALL_OFF + (1 << BpFET), // Bp
    P1_ALL_OFF + (1 << BpFET), // Bp
    P1_ALL_OFF + (1 << ApFET)  // Ap
};
const uint8_t commutation_pattern2[6] = { // turns on one of nFETs
    P1_ALL_OFF + (1 << ApFET) - (1 << BnFET), // Bn
    P1_ALL_OFF + (1 << CpFET) - (1 << BnFET), // Bn
    P1_ALL_OFF + (1 << CpFET) - (1 << AnFET), // An
    P1_ALL_OFF + (1 << BpFET) - (1 << AnFET), // An
    P1_ALL_OFF + (1 << BpFET) - (1 << CnFET), // Cn
    P1_ALL_OFF + (1 << ApFET) - (1 << CnFET)  // Cn
};
const uint8_t commutation_comp[6] = { 0x10, 0x11, 0x13, 0x10, 0x11, 0x13 }; // C A B C A B

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

volatile uint16_t on_time = 0;
volatile uint16_t revs = 0;

volatile bit controller_active = 0;
volatile uint16_t controller_speed; // revs/s
volatile uint32_t controller_desired_revs; // 16.16
volatile int32_t controller_integral; // 16.16 on_time units

volatile uint8_t tx_buf[30];
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
volatile uint8_t rx_buf[20];
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

volatile uint8_t __xdata capture_buf[512];
volatile uint16_t capture_pos = sizeof(capture_buf);

void handle_message() {
    if(rx_buf_pos >= 3 && rx_buf[0] == 2 && rx_buf[1] == *id_pointer) {
        if(rx_buf[2] == 0) {
            if(rx_buf_pos != 3) return;
            RSTSRC = 0x10; // reset into bootloader
            while(1);
        } else if(rx_buf[2] == 1) { // get status
            uint16_t my_revs, local_on_time;
            __critical {
                my_revs = revs;
                local_on_time = on_time;
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
            send_escaped_byte_and_crc(local_on_time&0xff);
            send_escaped_byte_and_crc(local_on_time>>8);
            send_escaped_byte_and_crc((controller_desired_revs >>  0) & 0xff);
            send_escaped_byte_and_crc((controller_desired_revs >>  8) & 0xff);
            send_escaped_byte_and_crc((controller_desired_revs >> 16) & 0xff);
            send_escaped_byte_and_crc((controller_desired_revs >> 24) & 0xff);
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
                on_time = rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
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
                {
                    uint16_t my_revs;
                    __critical {
                        my_revs = revs;
                    }
                    controller_desired_revs = (uint32_t)my_revs << 16;
                }
                controller_integral = 0;
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

uint16_t my_on_time;
uint16_t my_off_time;
uint8_t count = 255;
uint8_t f;
uint8_t state = 0;
uint8_t res;
uint8_t commutation_step;
#define DELAY(n, length) PCA0CP0 = PCA0CP0 + (length); state = n; return; case n:
void pca0_isr() __interrupt PCA0_IRQn {
    PCA0CN_CCF0 = 0;
    
    switch(state) { case 0:
        while(true) {
            count = 255;
            my_on_time = on_time;
            if(my_on_time < 100 || my_on_time > 1000) {
                DELAY(1, 2450) // .1 ms
                continue;
            }
            my_off_time = 1225 - my_on_time;
            
            for(commutation_step = 0; commutation_step < 6; commutation_step++) {
                CPT0MX = commutation_comp[commutation_step];
                P1 = commutation_pattern[commutation_step];
                for(f = 0; f < count; f++) {
                    DELAY(3, my_off_time)
                    P1 = commutation_pattern2[commutation_step];
                    DELAY(2, my_on_time)
                    res = CPT0CN;
                    P1 = commutation_pattern[commutation_step];
                    if(((res & 0x40) >> 6) ^ (commutation_step & 1)) { break; }
                }
                P1 = P1_ALL_OFF;
                revs++;
            }
        }
    }
}

uint32_t last_controller_time;
void main() {
    int16_t last_err;
    bit last_err_present = 0;
    
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
            __critical {
                controller_desired_revs += (uint32_t)controller_speed << 8;
            }
            if(!controller_active) continue;
            {
                uint16_t my_revs;
                __critical {
                    my_revs = revs;
                }
                {
                    int16_t err = (uint16_t)(controller_desired_revs >> 16) - my_revs;
                    int16_t d = last_err_present ? err - last_err : 0;
                    int16_t controller_on_time = 250 + (d << 1) + (err >> 1) ;//+ (controller_integral >> 10);
                    last_err = err;
                    last_err_present = 1;
                    if(controller_on_time < 99) controller_on_time = 99;
                    if(controller_on_time > 1000) controller_on_time = 1000;
                    controller_integral += err;
                    __critical {
                        if(controller_active) {
                            on_time = controller_on_time;
                        }
                    }
                }
            }
        }
    }
}
