//
// Created by Jun Kim on 4/18/23.
//

#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

// 31250 bits / sec
// byte = 8 bits
// 3906.25 bytes / sec
// 3.906 bytes / ms
#define BAUD_RATE 31250

// does prescaler need to change?
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
//#define BAUD_PRESCALER 31
#define MIN_INTERVAL_MS 5 // minimum interval in milliseconds between array indexing
#define MAX_INTERVAL_MS 10 // maximum interval in milliseconds between array indexing

#define OCR1A_VALUE 300 // 936
#define TRACK_LENGTH 864 // ~450~500 maximum on mega 3604 to 3605 border for compile error

volatile int count = 0;
volatile int targetCount = 0;

volatile int record1_armed = 0;
volatile int record1 = 0;
volatile int record2_armed = 0;
volatile int record2 = 0;
volatile int record3_armed = 0;
volatile int record3 = 0;

volatile int playTrack = 0;
volatile int metronome_on = 0;
volatile int playback = 1;
volatile int panic = 0;
int prev_step;
//int track1[20][3] = {
//        {0x93, 0x24, 0x40} ,
//        {0x00, 0x00, 0x00} ,
//        {0x00, 0x00, 0x00},
//        {0x83, 0x24, 0x40},
//        {0x00, 0x00, 0x00} ,
//        {0x00, 0x00, 0x00},
//        {0x93, 0x24, 0x40} ,
//        {0x00, 0x00, 0x00} ,
//        {0x00, 0x00, 0x00}
//};
uint8_t track1[TRACK_LENGTH][3];
uint8_t track2[TRACK_LENGTH][3];
uint8_t track3[TRACK_LENGTH][3];
volatile uint32_t step = 0; // volatile?

// timing stuff
// TNCT0 counts from 0 to OCR0A and move onto the next
// step variable increments everytime OCR0A is reached

ISR(PCINT2_vect) {
        // MEGA code
        if (PINK & (1 << PINK0)) {
//            char printReset[25];
//            sprintf(printReset, "RECORD1 ISR\n");
//            UART_putstring(printReset);
            record1_armed = 1;
            PORTE |= (1 << PE4);
        }
        if (PINK & (1 << PINK1)) {
            record2_armed = 1;
            PORTE |= (1 << PE5);
        }
        if (PINK & (1 << PINK2)) {
            record3_armed = 1;
            PORTG |= (1 << PG5);
        }
        if (PINK & (1 << PINK3)) {
            // reset
            // clear tracks
//            char printReset[25];
//            sprintf(printReset, "RESET\n");
//            UART_putstring(printReset);
            step = 0;
            count = 0;
            for (int i=0; i < TRACK_LENGTH; i++) {
                if (track1[i][0] >> 4 == 8) {
                    UART_send(track1[i][0]);
                    UART_send(track1[i][1]);
                    UART_send(track1[i][2]);
                }
                track1[i][0] = 0;
                track1[i][1] = 0;
                track1[i][2] = 0;
                if (track2[i][0] >> 4 == 8) {
                    UART_send(track2[i][0]);
                    UART_send(track2[i][1]);
                    UART_send(track2[i][2]);
                }
                track2[i][0] = 0;
                track2[i][1] = 0;
                track2[i][2] = 0;
                if (track3[i][0] >> 4 == 8) {
                    UART_send(track3[i][0]);
                    UART_send(track3[i][1]);
                    UART_send(track3[i][2]);
                }
                track3[i][0] = 0;
                track3[i][1] = 0;
                track3[i][2] = 0;
            }
            // send panic signal to stop all notes that are playing
            // reset LEDs

        }
        if (PINK & (1 << PINK4)) {
            // playback
            playback = playback ? 0 : 1;
        }
        if (PINK & (1 << PINK5)) {
            // metronome buzzer
//            char printISR[25];
//            sprintf(printISR, "IN ISR\n");
//            UART_putstring(printISR);
            if (metronome_on == 0) {
                metronome_on = 1;
            } else {
                metronome_on = 0;
            }
        }
}


void Initialize() {

    cli(); // disable global interrupts;

    // write your initialization code here
    UART_init(BAUD_PRESCALER);

    // set up LEDs as output
    DDRE |= (1 << DDE4); // channel 1 LED PWM2 PE4
    PORTE &= ~(1 << PE4);
    DDRE |= (1 << DDE5); // channel 2 LED PWM3 PE5
    PORTE &= ~(1 << PE5);
    DDRG |= (1 << DDG5); // channel 3 LED PWM4 PG5
    PORTG &= ~(1 << PG5);
    DDRE |= (1 << DDE3); // set up recording LED Pin 5 PWM5 PE3

    // SET UP BUTTONS AND PCINT MEGA
    // PCINT 16~23 is PCMSK2
    // PCINT 16~23 is PCIE2 for PCICR
    // PIN A8 PK0 PCINT16
    // PIN A9 PK1 PCINT17
    // PIN A10 PK2 PCINT18
    // the I-bit in the Status Register SREG is set one ^?
    DDRK &= ~(1 << DDK0); // set as input
    PORTK |= (1<<PK0); // enable pull up resistor
    PCMSK2 |= (1<<PCINT16); // enable trigger for PCINT16
    PCICR |= (1<<PCIE2); //enable PCINT1 pin change interrupt

    DDRK &= ~(1 << DDK1);
    PORTK |= (1<<PK1);
    PCMSK2 |= (1<<PCINT17);
    PCICR |= (1<<PCIE2);

    DDRK &= !(1<<DDK2);
    PORTK |= (1<<PK2);
    PCMSK2 |= (1<<PCINT18);
    PCICR |= (1<<PCIE2);

    // RESET BUTTON
    // A11
    DDRK &= ~(1 << DDK3);
    PORTK |= (1<<PK3);
    PCMSK2 |= (1<<PCINT19);
    PCICR |= (1<<PCIE2);
    // PLAYBACK BUTTON
    // A12
    DDRK &= ~(1 << DDK4);
    PORTK |= (1<<PK4);
    PCMSK2 |= (1<<PCINT20);
    PCICR |= (1<<PCIE2);

    // METRONOME LED
    DDRH |= (1 << DDH3);
    DDRH |= (1 << DDH4);
    DDRH |= (1 << DDH5);
    DDRH |= (1 << DDH6);

    // METRONOME BUZZER: button output pin with pin change interrupt
    DDRK &= ~(1 << DDK5); // set A13 PK5 as output
    PORTK |= (1<<PK5);
    PCMSK2 |= (1 << PCINT21);
    PCICR |= (1<<PCIE2);
    DDRB |= (1<<DDB4); // PWM 10 PB4 set as output

    // TEST: Analog on Mega as Digital
    // A8 PK0 PCINT 16
//    DDRK |= (1 << PK0);
//    PORTK |= (1 << PK0);

    // set up buttons as input UNO
//    DDRB &= ~(1 << DDB0); // channel 1 button (pin change interrupt) PWM8 PH5
//    PORTB |= (1<<PORTB0); // enable pull up resistor on PB2
//    PCMSK0 |= (1<<PCINT0); // enable trigger for PCINT0
//
//    PCICR |= (1<<PCIE0); //enable PCINT1 pin change interrupt
//
//    DDRB &= ~(1 << DDB1); // channel 2 button (pin change interrupt)
//    PORTB |= (1<<PORTB1); // enable pull up resistor on PB1
//    PCMSK0 |= (1<<PCINT1); // enable trigger for PCINT1
//
//    DDRB &= ~(1 << DDB2); // channel 3 button (pin change interrupt)
//    PORTB |= (1<<PORTB2); // enable pull up resistor on PB2
//    PCMSK0 |= (1<<PCINT2); // enable trigger for PCINT2

    // TIMER for BPM
    // CTC on OCR1A
    // OCR1A 65535 => 232 ms 9362 => 33 ms 936 => 3.3ms 10 => 0.035ms
    OCR1A = OCR1A_VALUE;
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
//    OCR1A = MIN_INTERVAL_MS * (F_CPU / 1000) / 64 - 1; // Set Timer1 compare value to minimum interval
    TCCR1B |= (1 << WGM12); // Set Timer1 to CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10); // Set Timer1 prescaler to 64
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt

    // POTENTIOMETER
    // Reading in potentiometer
    PRR0 &= ~(1 << PRADC); // changed to PRR -> PRR0 for mega
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    // Set the ADC Clock div by 128
    ADCSRA |= (1 << ADPS0);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS2);
    // Select channel 0
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX3);
    // Set to auto trigger
    ADCSRA |= (1 << ADATE);
    // Set to free running
    ADCSRB &= ~(1 << ADTS0);
    ADCSRB &= ~(1 << ADTS1);
    ADCSRB &= ~(1 << ADTS2);
    // Disable digital input buffer on ADC pin
    DIDR0 |= (1 << ADC0D);
    // Enable ADC
    ADCSRA |= (1 << ADEN) | (1 << ADIE);  // Enable ADC and ADC interrupt
    // ADC interrupt

    // start conversion
    ADCSRA |= (1 << ADSC);

    step = 0;

    sei(); // enable global interrupts this stops midi from working
}

void process_midi_message(uint8_t status_byte, uint8_t data_byte1, uint8_t data_byte2) {

    uint8_t message_type = status_byte & 0xF0;

    if (message_type >> 4 == 9 || message_type >> 4 == 8) {
        // note on message
//        char printData[25];
//        sprintf(printData, "ON: %X %X %X\n", status_byte, data_byte1, data_byte2);
//        UART_putstring(printData);

        UART_send(status_byte); // 1 byte always channel 4?
        UART_send(data_byte1); // 1 byte
        UART_send(data_byte2); // 1 byte

//        char printConcat[25];
//        sprintf(printConcat, "STEP: %d CONCAT: %X\n", step, concat);
//        track1[step] = concat;
        if (record1 == 1) {
//            char printRecord[25];
//            sprintf(printRecord, "RECORDING NOTE %X at STEP %d\n", data_byte1, step);
//            UART_putstring(printRecord);
            // fix channel to status_byte
            track1[step][0] = (status_byte & 0xF0); // always channel 1
            track1[step][1] = data_byte1;
            track1[step][2] = data_byte2;
        }
        if (record2 == 1) {
            track2[step][0] = (status_byte & 0xF0) | 0x01; // always channel 2
            track2[step][1] = data_byte1;
            track2[step][2] = data_byte2;
        }
        if (record3 == 1) {
            track3[step][0] = (status_byte & 0xF0) | 0x02; // always channel 3
            track3[step][1] = data_byte1;
            track3[step][2] = data_byte2;
        }

    }
}

void process_midi_data(uint8_t midi_byte) {

    static uint8_t status_byte = 0;
    static uint8_t data_byte1 = 0;
    static uint8_t data_byte2 = 0;
    static uint8_t bytes_received = 0;

    if ((midi_byte >> 4) == 8 || (midi_byte >> 4 == 9)) {
        // This is a status byte
        status_byte = midi_byte;
        bytes_received = 1;
    }
    else {
        // This is a data byte
        if (bytes_received == 1) {
            data_byte1 = midi_byte;
            bytes_received = 2;
        }
        else if (bytes_received == 2) {
            data_byte2 = midi_byte;
            bytes_received = 3;
        }
        else {
            // Error: received unexpected data byte
            bytes_received = 0;
        }
    }

    if (bytes_received == 3) {
        // We have received a complete MIDI message
        process_midi_message(status_byte, data_byte1, data_byte2);
        bytes_received = 0;
    }

}

//void sendMidiSignal(int midi) {
//    // check if valid midi signal
//    if (midi[0] >> 4 == 9 || midi[0] >> 4 == 8) {
//        UART_send(midi >> 16 & 0xFF);
//        UART_send(midi >> 8 & 0xFF);
//        UART_send(midi & 0xFF);
//    }
//}

ISR(TIMER1_COMPA_vect) {
//    char printCount[25];
//    sprintf(printCount, "COUNT: %d TARGET: %d\n", count, targetCount);
//    UART_putstring(printCount);
//    char printCount[25];
//    sprintf(printCount, "TIMER ISR\n");
//    UART_putstring(printCount);
//    char printCount[25];
//    sprintf(printCount, "STEP: %d\n", step);
//    UART_putstring(printCount);

        if (count >= targetCount) {
            // Timer1 compare interrupt service routine
            if (step == TRACK_LENGTH - 10 && (record1_armed || record2_armed || record3_armed)) {
//            char printArm[25];
//            sprintf(printArm, "RECORD START");
//            UART_putstring(printArm);
                record1 = record1_armed ? 1 : 0;
                record2 = record2_armed ? 1 : 0;
                record3 = record3_armed ? 1 : 0;
                // turn on record light
                PORTE |= (1 << PORTE3); // but this turns on when we go from step 0 to 1 for some reason
            } else if (step == (TRACK_LENGTH - 11) && (record1 == 1 || record2 == 1 || record3 == 1)) {
//            char printStop[25];
//            sprintf(printStop, "RECORDS STOP");
//            UART_putstring(printStop);
                if (record1 == 1) {
                    record1_armed = 0;
                    PORTE &= ~(1 << PORTE4); // turn off arm light
                }
                if (record2 == 1) {
                    record2_armed = 0;
                    PORTE &= ~(1 << PORTE5); // turn off arm light
                }
                if (record3 == 1) {
                    record3_armed = 0;
                    PORTG &= ~(1 << PORTG5); // turn off arm light
                }
                record1 = 0;
                record2 = 0;
                record3 = 0;
                PORTE &= ~(1 << PORTE3); // turn off record light
            }
            // set playTrack flag
            playTrack = 1;

            step++; // Increment the step index
            if (step >= TRACK_LENGTH) {
                step = 0; // Reset step index when it reaches the end of the array
            }

            count = 0;

            // METRONOME
            // track length is 864 [0, 863]
            // 864 / 16 = 54
            if (step < TRACK_LENGTH / 4) {
                if (step % (TRACK_LENGTH / 16) == 0) {
                    PORTH |= (1 << PH3);
                    if (metronome_on) PORTB |= (1 << PB4);// buzzer on
                    prev_step = step;
                } else if (step >= prev_step+5) {
                    PORTH &= ~(1 << PH3); // led off
                    if (metronome_on) PORTB &= ~(1 << PB4); // buzzer off
                }
            } else if (step < TRACK_LENGTH / 2) {
                if (step % (TRACK_LENGTH / 16) == 0) {
                    PORTH |= (1 << PH4);
                    if (metronome_on) PORTB |= (1 << PB4);// buzzer on
                    prev_step = step;
                } else if (step >= prev_step+5) {
                    PORTH &= ~(1 << PH4);
                    if (metronome_on) PORTB &= ~(1 << PB4); // buzzer off
                }
            } else if (step < TRACK_LENGTH * 3 / 4) {
                if (step % (TRACK_LENGTH / 16) == 0) {
                    PORTH |= (1 << PH5);
                    if (metronome_on) PORTB |= (1 << PB4);// buzzer on
                    prev_step = step;
                } else if (step >= prev_step+5) {
                    PORTH &= ~(1 << PH5);
                    if (metronome_on) PORTB &= ~(1 << PB4); // buzzer off
                }
            } else if (step <TRACK_LENGTH) {
                if (step % (TRACK_LENGTH / 16) == 0) {
                    PORTH |= (1 << PH6);
                    if (metronome_on) PORTB |= (1 << PB4);// buzzer on
                    prev_step = step;
                } else if (step >= prev_step+5) {
                    PORTH &= ~(1 << PH6);
                    if (metronome_on) PORTB &= ~(1 << PB4); // buzzer off
                }
            }
        } else {
            count++;
        }
}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ISR for ADC
ISR(ADC_vect) {
        int interval = map(ADC, 0, 1023, MIN_INTERVAL_MS, MAX_INTERVAL_MS);
        uint32_t OCR1A_uint = (uint32_t) OCR1A;

        int ms = ((OCR1A_uint + 1) * 64 * 1000 / F_CPU); // 3.748 ms ~ 3
        targetCount = interval / ms; // min interval 5 / 3 = 1 ~ 2 counts 6 ms per step

//        char printInterval[50];
////        sprintf(printInterval, "ADC: %u, interval: %u, ms: %u, targetCount: %u\n",
////                ADC, interval, ms, targetCount);
////        UART_putstring(printInterval);
}

int main(void)
{
    Initialize();
    // set count to initial targetCount to immediately start at step 0?

    while(1)
    {
//        char printTest[25];
//        sprintf(printTest, "METRONOME ON: %d\n", metronome_on);
//        UART_putstring(printTest);
        // PLAY/PAUSE
//        if (panic) {
//            UART_send(0xFE);
//            UART_send(0xFE);
//            UART_send(0xFE);
//            UART_send(0x79);
//            panic = 0;
//        }
        while (!playback) {
            // infinite
        }

        // all prints
//        char printAny[25];
//        sprintf(printAny, "PRINT\n");
//        UART_putstring(printAny);
        // STEP INCREMENT ISR
        if (playTrack) {
//            char printStep[100];
//            sprintf(printStep, "LENGTH: %u, STEP: %lu ADC: %u, OCR1A: %u count: %u targetCount: %u\n\n",
//                    TRACK_LENGTH, step, ADC, OCR1A, count, targetCount);
//            UART_putstring(printStep);
//            char printRecord[100];
//            sprintf(printRecord, "record1_armed: %d record1: %d record2_armed: %d record2: %d record3_armed: %d record3: %d \n\n",
//                    record1_armed, record1, record2_armed, record2, record3_armed, record3);
//            UART_putstring(printRecord);

//            char printTrack[100];
//            sprintf(printTrack, "NOTES: %X %X %X\n\n", track1[step][0], track1[step][1], track1[step][2]);
//            UART_putstring(printTrack);

            // always play what's in the track
            if (track1[step][0] >> 4 == 9 || track1[step][0] >> 4 == 8) {
                UART_send(track1[step][0]);
                UART_send(track1[step][1]);
                UART_send(track1[step][2]);
            }
            if (track2[step][0] >> 4 == 9 || track2[step][0] >> 4 == 8) {
                UART_send(track2[step][0]);
                UART_send(track2[step][1]);
                UART_send(track2[step][2]);
            }
            if (track3[step][0] >> 4 == 9 || track3[step][0] >> 4 == 8) {
                UART_send(track3[step][0]);
                UART_send(track3[step][1]);
                UART_send(track3[step][2]);
            }
            playTrack = 0;
            // what if the UART_send here doesn't complete before the next step increment?
            // think it's fine
        }

        unsigned char data;
        if (UCSR0A & (1 << RXC0)) {
            data = UDR0;
            process_midi_data(data);
        }
    }
}