// Copyright 2016, Takashi Toyoshima <toyoshim@gmail.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include "LPC8xx.h"

#include "lpc8xx_gpio.h"
#include "lpc8xx_uart.h"
#include "lpc8xx_i2c.h"

const char kMsgReady[] = "I2CInspector ready\n";
const char kMsgError[] = "\n*** buffer overflow ***\n";
const char kMsgInvalidCmd[] = "\n*** invalid command ***\n";

// I2C decode status.
enum {
    STATE_WAIT_START,
    STATE_DECODE_ADDRESS,
    STATE_DECODE_RW,
    STATE_DECODE_ADDRESS_ACK,
    STATE_DECODE_10BIT_ADDRESS,
    STATE_DECODE_10BIT_ADDRESS_ACK,
    STATE_DECODE_DATA,
    STATE_DECODE_DATA_ACK
};

enum {
    RW_R,
    RW_W
};

enum {
    NACK,
    ACK
};

enum {
    HL_H,
    HL_L
};

#define MAX_UART_SIZE 256
#define MAX_CMD_SIZE 32

// GPIO common constants.
const uint8_t kBitRx = 0;
const uint8_t kBitScl = 2;
const uint8_t kBitSda = 3;
const uint8_t kBitTx = 4;
const uint8_t kBitLed = 5;
const uint8_t kDirInput = 0;
const uint8_t kDirOutput = 1;
const uint8_t kPort0 = 0;

// SCTimer common constants.
const uint8_t kEvent2 = 1 << 2;
const uint8_t kEvent3 = 1 << 3;
const uint8_t kEvent4 = 1 << 4;

volatile uint8_t state = STATE_WAIT_START;
volatile uint8_t count = 0;
volatile uint16_t address = 0;
volatile uint8_t rw = RW_R;
volatile uint8_t ack = NACK;
volatile uint8_t data;

volatile uint16_t uart_rp = 0;
volatile uint16_t uart_wp = 0;
volatile uint16_t uart_size = 0;
volatile uint8_t uart_data[MAX_UART_SIZE];
volatile uint16_t uart_error = 0;

// Declared in wrong size because these are not used.
volatile uint8_t I2CSlaveTXBuffer[1];
volatile uint8_t I2CSlaveRXBuffer[1];
volatile uint32_t I2CMonBuffer[1];
volatile uint16_t i2c_master = 0;

volatile uint8_t cmd_data[MAX_CMD_SIZE];
volatile uint16_t cmd_rp = 0;
volatile uint16_t cmd_wp = 0;
volatile uint16_t cmd_pp = 0;
volatile uint16_t cmd_error = 0;

uint16_t iinc(uint16_t i, uint16_t max) {
    i++;
    return (i == max) ? 0 : i;
}

uint16_t iadd(uint16_t i, uint16_t n, uint16_t max) {
    i += n;
    return (i < max) ? i : i - max;
}

uint16_t isub(uint16_t a, uint16_t b, uint16_t max) {
    return (a >= b) ? a - b: a + max - b;
}

int atoi(uint8_t c) {
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('a' <= c && c <= 'f')
        return c - 'a' + 10;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

void i2c_on() {
    // Assign I2C to PIO while sending data as a I2C Master.
    LPC_SWM->PINASSIGN7 = (LPC_SWM->PINASSIGN7 & 0x00ffffffUL) | ((uint32_t)kBitSda << 24);
    LPC_SWM->PINASSIGN8 = (LPC_SWM->PINASSIGN8 & 0xffffff00UL) | (uint32_t)kBitScl;
    i2c_master = 1;
}

void i2c_off() {
    // Release I2C assigns.
    LPC_SWM->PINASSIGN7 |= 0xff000000UL;
    LPC_SWM->PINASSIGN8 |= 0x000000ffUL;
    i2c_master = 0;
}

void uart_putc(int c) {
    if (uart_error || uart_size == MAX_UART_SIZE) {
        uart_error = 1;
        return;
    }
    uart_data[uart_wp] = c;
    uart_wp = iinc(uart_wp, MAX_UART_SIZE);
    uart_size++;
}

void uart_putx(int i) {
    uart_putc((i < 10) ? ('0' + i) : ('A' - 10 + i));
}

void uart_putxx(int i) {
    uart_putx(i >> 4);
    uart_putx(i & 0xf);
}

void uart_puts(char* s) {
    while (*s)
        uart_putc(*s++);
}

void uart_write() {
    for (;;) {
        if (0 == uart_size) {
            if (uart_error) {
                UARTSend(LPC_USART0, (uint8_t*)kMsgError, sizeof(kMsgError));
                uart_error = 0;
            }
            return;
        }
        uint32_t size = (uart_wp < uart_rp) ? (MAX_UART_SIZE - uart_rp) : uart_size;
        UARTSend(LPC_USART0, (uint8_t*)&uart_data[uart_rp], size);
        uart_size -= size;
        uart_rp = iadd(uart_rp, size, MAX_UART_SIZE);
    }
}

// Command format:
//   3E<4031 : write data 0x40, 0x31 to address 0x3e with /w (7c)
//   3E>1    : read 1 byte from address 0x3e with r (7d)  // TODO: hasn't tested yet at all.
void uart_read() {
    while (cmd_pp != cmd_wp) {
        if (cmd_data[cmd_pp] == '\n')
            break;
        cmd_pp = iinc(cmd_pp, MAX_CMD_SIZE);
    }
    // Following check should not be cmd_pp == cmd_wp to avoid a race condition.
    if (cmd_data[cmd_pp] != '\n')
        return;
    uint16_t size = isub(cmd_pp, cmd_rp, MAX_CMD_SIZE);
    uint8_t done = 0;
    if (size >= 3) {
        uint16_t addr = atoi(cmd_data[cmd_rp]) * 16;
        cmd_rp = iinc(cmd_rp, MAX_CMD_SIZE);
        addr += atoi(cmd_data[cmd_rp]);
        cmd_rp = iinc(cmd_rp, MAX_CMD_SIZE);
        uint8_t rw = cmd_data[cmd_rp];
        cmd_rp = iinc(cmd_rp, MAX_CMD_SIZE);
        uint8_t i2c_data[32];
        uint8_t i2c_size = 0;
        if (addr < 0) {
            // invalid address or read/write is specified.
        } else if (rw == '<') {
            for (;;) {
                if (cmd_rp == cmd_pp)
                    break;
                int hi = atoi(cmd_data[cmd_rp]);
                cmd_rp = iinc(cmd_rp, MAX_CMD_SIZE);
                if (cmd_rp == cmd_pp)
                    break;
                int lo = atoi(cmd_data[cmd_rp]);
                cmd_rp = iinc(cmd_rp, MAX_CMD_SIZE);
                if (hi < 0 || lo < 0)
                    break;
                i2c_data[i2c_size++] = hi * 16 + lo;
                if (i2c_size == 32)
                    break;
                if (cmd_rp == cmd_pp) {
                    i2c_on();
                    // Send by myself to avoid unstable stall of I2C_MstSend.
                    LPC_I2C->MSTDAT = addr << 1;
                    LPC_I2C->MSTCTL = CTL_MSTSTART;
                    for (uint16_t i = 0; i < i2c_size; i++) {
                        while (!(LPC_I2C->STAT & STAT_MSTPEND));
                        if((LPC_I2C->STAT & MASTER_STATE_MASK) != STAT_MSTTX)
                            break;
                        LPC_I2C->MSTDAT = i2c_data[i];
                        LPC_I2C->MSTCTL = CTL_MSTCONTINUE;
                     }
                     while (!(LPC_I2C->STAT & STAT_MSTPEND));
                    LPC_I2C->MSTCTL = CTL_MSTSTOP | CTL_MSTCONTINUE;
                    I2C_CheckIdle(LPC_I2C);
                    i2c_off();
                    done = 1;
                    uart_putc('\n');
                    break;
                }
            }
        } else if (rw == '>') {
            if (cmd_rp != cmd_pp) {
                int i = atoi(cmd_data[cmd_rp]);
                cmd_rp = iinc(cmd_rp, MAX_CMD_SIZE);
                if (cmd_rp == cmd_pp) {
                    i2c_on();
                    // TODO: This probably has the same problem with I2C_MstSend.
                    I2C_MstReceive(LPC_I2C, (addr << 1) | 1, i2c_data, i);
                    i2c_off();
                    done = 1;
                    uart_putc('\n');
                }
            }
        }
    }
    if (!done)
        uart_puts((char*)kMsgInvalidCmd);

    cmd_rp = cmd_pp = iinc(cmd_pp, MAX_CMD_SIZE);
}

int main () {
    // Enable Clocks we need, IOCON, I2C, SCTimer, and UART0.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 18) | (1 << 5) | (1 << 8) | (1 << 14);

    // Disable Serial Wire Debug and Reset, then enable PIO0_2, PIO0_3, and PIO0_5.
#if USE_LED
    LPC_SWM->PINENABLE0 |= (3 << 2) | (1 << 6);
#else
    LPC_SWM->PINENABLE0 |= (3 << 2);
#endif

    // Configure PIO0_2 and PIO0_3 as inputs to monitor I2C Bus.
    GPIOInit();
    GPIOSetDir(kPort0, kBitScl, kDirInput);
    GPIOSetDir(kPort0, kBitSda, kDirInput);
#if USE_LED
    GPIOSetDir(kPort0, kBitLed, kDirOutput);

    // LED ON.
    GPIOSetBitValue(kPort0, kBitLed, 1);
#endif

    // Set PIO0_2 and PIO0_3 as pseudo open drain with neither pulldown or pullup.
    LPC_IOCON->PIO0_2 = (1 << 10);
    LPC_IOCON->PIO0_3 = (1 << 10);

    // Reset I2C.
    LPC_SYSCON->PRESETCTRL &= ~(1 << 6);
    LPC_SYSCON->PRESETCTRL |= (1 << 6);

    // Configured as I2C Master running at 100KHz, but not assigned here.
    I2C_MstInit(LPC_I2C, 12000000 / 100000 / 2 - 1, CFG_MSTENA, 0);

    // Assign SCL to CTIN_0.
    LPC_SWM->PINASSIGN5 = 0x00ffffffUL | ((uint32_t)kBitScl << 24);

    // Assign SDA to CTIN_1.
    LPC_SWM->PINASSIGN6 = 0xffffff00UL | (uint32_t)kBitSda;

    // Reset SCTimer.
    LPC_SYSCON->PRESETCTRL &= ~(1 << 8);
    LPC_SYSCON->PRESETCTRL |= (1 << 8);

    const uint8_t kOnState0 = 1 << 0;
    const uint8_t kOnState1 = 1 << 1;
    const uint32_t kOnScl = 0 << 6;
    const uint32_t kOnSda = 1 << 6;
    const uint32_t kOnLow = 0 << 10;
    const uint32_t kOnRising = 1 << 10;
    const uint32_t kOnFalling = 2 << 10;
    const uint32_t kOnHigh = 3 << 10;
    const uint32_t kIoMode = 2 << 12;
    const uint32_t kLoadState = 1 << 14;
    const uint32_t kState0 = 0 << 15;
    const uint32_t kState1 = 1 << 15;

    // Synchronize state0/1 with SCL.
    LPC_SCT->EVENT[0].STATE = kOnState1;
    LPC_SCT->EVENT[0].CTRL = kIoMode | kOnLow | kOnScl | kLoadState | kState0;
    LPC_SCT->EVENT[1].STATE = kOnState0;
    LPC_SCT->EVENT[1].CTRL = kIoMode | kOnHigh | kOnScl | kLoadState | kState1;

    // START condition
    LPC_SCT->EVENT[2].STATE = kOnState1;
    LPC_SCT->EVENT[2].CTRL = kIoMode | kOnFalling | kOnSda;

    // STOP condition
    LPC_SCT->EVENT[3].STATE = kOnState1;
    LPC_SCT->EVENT[3].CTRL = kIoMode | kOnRising | kOnSda;

    // Clock edge condition
    LPC_SCT->EVENT[4].STATE = kOnState0;
    LPC_SCT->EVENT[4].CTRL = kIoMode | kOnRising | kOnScl;

    // Release HOLD.
    LPC_SCT->CTRL_L = 0;

    // Enable Interrupt for the event 2, 3 and 4.
    LPC_SCT->EVEN = kEvent2 | kEvent3 | kEvent4;
    NVIC->ISER[0] = (1 << 9);

    // Initialize USART and say Hello.
    LPC_SWM->PINASSIGN0 = 0xffff0000UL | (kBitTx << 0) | (kBitRx << 8);
    UARTInit(LPC_USART0, 230400);
    UARTSend(LPC_USART0, (uint8_t*)kMsgReady, sizeof(kMsgReady));

    for (;;) {
        uart_write();
        uart_read();
    }
    return 0;
}

void logln () {
    if (ack == NACK)
        uart_putc('!');
    uart_putc('\n');
}

void SCT_IRQHandler () {
    if(LPC_SCT->EVFLAG & kEvent2) {
        // Start
        LPC_SCT->EVFLAG = kEvent2;
        if (i2c_master)
            return;
        if (state != STATE_WAIT_START)
            logln();
        state = STATE_DECODE_ADDRESS;
        count = 0;
        address = 0;
    } else if (LPC_SCT->EVFLAG & kEvent3) {
        // Stop
        LPC_SCT->EVFLAG = kEvent3;
        if (i2c_master)
            return;
        logln();
        state = STATE_WAIT_START;
    } else if (LPC_SCT->EVFLAG & kEvent4) {
        // Clock
        LPC_SCT->EVFLAG = kEvent4;
        if (i2c_master)
            return;
        uint32_t bit = GPIOGetPinValue(kPort0, kBitSda);
        switch (state) {
        case STATE_DECODE_ADDRESS:
            address <<= 1;
            address |= bit;
            count++;
            if (count == 7) {
                state = STATE_DECODE_RW;
                uart_putxx(address & 0xff);
            }
            break;
        case STATE_DECODE_RW:
            rw = bit ? RW_R : RW_W;
            state = STATE_DECODE_ADDRESS_ACK;
            break;
        case STATE_DECODE_ADDRESS_ACK:
            ack = bit ? NACK : ACK;
            if ((address & 0x7c) == 0x78) {
                state = STATE_DECODE_10BIT_ADDRESS;
                address &= 3;
                uart_putx(address);
                count = 0;
            } else {
                count = 0;
                state = STATE_DECODE_DATA;
                uart_putc((rw == RW_R) ? '>' : '<');
            }
            break;
        case STATE_DECODE_10BIT_ADDRESS:
            address <<= 1;
            address |= bit;
            count++;
            if (count == 8) {
                state = STATE_DECODE_10BIT_ADDRESS_ACK;
                uart_putxx(address & 0xff);
            }
            break;
        case STATE_DECODE_10BIT_ADDRESS_ACK:
            ack = bit ? NACK : ACK;
            count = 0;
            state = STATE_DECODE_DATA;
            uart_putc((rw == RW_R) ? '>' : '<');
            break;
        case STATE_DECODE_DATA:
            data <<= 1;
            data |= bit;
            count++;
            if (count == 8) {
                uart_putxx(data);
                state = STATE_DECODE_DATA_ACK;
            }
            break;
        case STATE_DECODE_DATA_ACK:
            if (bit)
                ack = NACK;
            count = 0;
            state = STATE_DECODE_DATA;
            break;
        }
    }
}

void UARTCustom_IRQHandler () {
    if (LPC_USART0->STAT & RXRDY) {
        uint8_t rxdata = LPC_USART0->RXDATA;
        LPC_USART0->STAT = RXRDY;
        uint16_t next = iinc(cmd_wp, MAX_CMD_SIZE);
        if (next == cmd_rp) {
            cmd_error = 1;
            return;
        }

        // Echo input characters, but '\n' will done after the request is handled.
        if (rxdata != '\n')
            uart_putc(rxdata);
        cmd_data[cmd_wp] = rxdata;
        cmd_wp = next;
    }
    UART0_IRQHandler();
}
