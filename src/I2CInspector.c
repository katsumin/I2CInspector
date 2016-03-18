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
const char kMsgError[] = "\n*** buffer overflow ***";

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

#define MAX_UART_SIZE 512
#define MAX_CMD_SIZE 16

// GPIO common constants.
const uint8_t kBitScl = 2;
const uint8_t kBitSda = 3;
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
volatile uint8_t data_ack = NACK;

volatile uint16_t uart_rp = 0;
volatile uint16_t uart_wp = 0;
volatile uint16_t uart_size = 0;
volatile uint8_t uart_data[MAX_UART_SIZE];
volatile uint8_t uart_error = 0;

// Declared in wrong size because these are not used.
volatile uint8_t I2CSlaveTXBuffer[1];
volatile uint8_t I2CSlaveRXBuffer[1];
volatile uint32_t I2CMonBuffer[1];

uint8_t cmd_state = STATE_DECODE_ADDRESS;
uint8_t cmd_address = 0;
uint8_t cmd_rw = RW_R;
uint8_t cmd_data[MAX_CMD_SIZE];
uint8_t cmd_data_size = 0;
uint8_t cmd_hl = HL_H;

void uart_putc(int c) {
    if (uart_error || uart_size == MAX_UART_SIZE) {
        uart_error = 1;
        return;
    }
    uart_data[uart_wp++] = c;
    if (uart_wp == MAX_UART_SIZE)
        uart_wp = 0;
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
        uart_rp += size;
        if (uart_rp >= MAX_UART_SIZE)
            uart_rp -= MAX_UART_SIZE;
    }
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

int main () {
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

    // Enable IOCON clock.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 18);

    // Enable pseudo open drain on PIO0_2 and PIO0_3. This only affect when these are reconfigured as I2C Master.
    //LPC_IOCON->PIO0_2 |= (1 << 10);
    //LPC_IOCON->PIO0_3 |= (1 << 10);

    // Enable I2C clock.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 5);

    // Reset I2C.
    LPC_SYSCON->PRESETCTRL &= ~(1 << 6);
    LPC_SYSCON->PRESETCTRL |= (1 << 6);

    // Configured as I2C Master running at 100KHz, but not assigned here.
    I2C_MstInit(LPC_I2C, I2C_SMODE_PRE_DIV, CFG_MSTENA, 0);

    // Assign SCL to CTIN_0.
    LPC_SWM->PINASSIGN5 = 0x00ffffffUL | ((uint32_t)kBitScl << 24);

    // Assign SDA to CTIN_1.
    LPC_SWM->PINASSIGN6 = 0xffffff00UL | (uint32_t)kBitSda;

    // Enable SCTimer clock.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);

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
    UARTInit(LPC_USART0, 230400);
    UARTSend(LPC_USART0, (uint8_t*)kMsgReady, sizeof(kMsgReady));

    // Change UART interrupt priority to lower than I2C so that we can call I2C functions inside UART IRQ handler.
    // TODO: Use an async bridge to call on the user thread.
    *((uint32_t*)0xe000e400) = 1 << 30;

    for (;;)
        uart_write();
    return 0;
}

void logln () {
    if (ack == NACK || data_ack == NACK)
        uart_putc('!');
    uart_putc('\n');
}

void SCT_IRQHandler () {
    if(LPC_SCT->EVFLAG & kEvent2) {
        // Start
        LPC_SCT->EVFLAG = kEvent2;
        if (state != STATE_WAIT_START)
            logln();
        state = STATE_DECODE_ADDRESS;
        count = 0;
        address = 0;
    } else if (LPC_SCT->EVFLAG & kEvent3) {
        // Stop
        LPC_SCT->EVFLAG = kEvent3;
        logln();
        state = STATE_WAIT_START;
    } else if (LPC_SCT->EVFLAG & kEvent4) {
        // Clock
        LPC_SCT->EVFLAG = kEvent4;
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
                data_ack = ACK;
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
            data_ack = ACK;
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
                data_ack = NACK;
            count = 0;
            state = STATE_DECODE_DATA;
            break;
        }
    }
}

// Command format:
//   3E<4031 : write data 0x40, 0x31 to address 0x3e with /w (7c)
//   3E>1    : read 1 byte from address 0x3e with r (7d)  // TODO: hasn't tested yet at all.
// Note:
//   Next command won't be accepted until previous command is shown in the monitor output.
void UARTCustom_IRQHandler () {
    if (LPC_USART0->STAT & RXRDY) {
        LPC_USART0->STAT = RXRDY;
        uint8_t data = LPC_USART0->RXDATA;
        if (cmd_state == STATE_DECODE_ADDRESS) {
            if (data == '<') {
                cmd_rw = RW_W;
                cmd_data_size = 0;
                cmd_state = STATE_DECODE_DATA;
            } else if (data == '>') {
                cmd_rw = RW_R;
                cmd_data_size = 0;
                cmd_state = STATE_DECODE_DATA;
            } else {
                cmd_address <<= 4;
                cmd_address |= atoi(data);
            }
        } else if (cmd_state == STATE_DECODE_DATA) {
            if (data == '\n' || data == ';') {
                // Assign I2C to PIO while sending data as a I2C Master.
                LPC_SWM->PINASSIGN7 = (LPC_SWM->PINASSIGN7 & 0x00ffffffUL) | ((uint32_t)kBitSda << 24);
                LPC_SWM->PINASSIGN8 = (LPC_SWM->PINASSIGN8 & 0xffffff00UL) | (uint32_t)kBitScl;
                GPIOSetDir(kPort0, kBitScl, kDirOutput);
                GPIOSetDir(kPort0, kBitSda, kDirOutput);

                if (cmd_rw == RW_W)
                    I2C_MstSend(LPC_I2C, cmd_address << 1, cmd_data, cmd_data_size);
                else
                    I2C_MstReceive(LPC_I2C, (cmd_address << 1) | 1, cmd_data, cmd_data[0]);

                // Release I2C assigns.
                GPIOSetDir(kPort0, kBitScl, kDirInput);
                GPIOSetDir(kPort0, kBitSda, kDirInput);
                LPC_SWM->PINASSIGN7 |= 0xff000000UL;
                LPC_SWM->PINASSIGN8 |= 0x000000ffUL;

                cmd_address = 0;
                cmd_hl = HL_H;
                cmd_data_size = 0;
                cmd_state = STATE_DECODE_ADDRESS;
            } else if (cmd_data_size < MAX_CMD_SIZE) {
                if (cmd_hl == HL_H) {
                    cmd_data[cmd_data_size] = atoi(data) << 4;
                    cmd_hl = HL_L;
                } else {
                    cmd_data[cmd_data_size] |= atoi(data);
                    cmd_hl = HL_H;
                    cmd_data_size++;
                }
            }
        }
    }
    UART0_IRQHandler();
}
