#define __STDC_LIMIT_MACROS
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdint.h>

// convenience macros
#define OUTB PORTB
#define OUTC PORTC
#define OUTD PORTD
#define INB PINB
#define INC PINC
#define IND PIND
#define DIRB DDRB
#define DIRC DDRC
#define DIRD DDRD

#define TIMER_COUNTER0_CS_MASK ((1 << CS02) | (1 << CS01) | (1 << CS00))
#define TIMER_COUNTER1_CS_MASK ((1 << CS12) | (1 << CS11) | (1 << CS10))
#define TIMER_COUNTER2_CS_MASK ((1 << CS22) | (1 << CS21) | (1 << CS20))

#define TIMER_COUNTER0_CURRENT_CS_MASK (TCCR0B & TIMER_COUNTER0_CS_MASK)
#define TIMER_COUNTER1_CURRENT_CS_MASK (TCCR1B & TIMER_COUNTER1_CS_MASK)
#define TIMER_COUNTER2_CURRENT_CS_MASK (TCCR2B & TIMER_COUNTER2_CS_MASK)

#define RESTORE_TIMER_COUNTER0_CS_MASK(mask) (TCCR0B |= (mask))
#define RESTORE_TIMER_COUNTER1_CS_MASK(mask) (TCCR1B |= (mask))
#define RESTORE_TIMER_COUNTER2_CS_MASK(mask) (TCCR2B |= (mask))

#define DISABLE_TIMER_COUNTER0 (TCCR0B &= ~TIMER_COUNTER0_CS_MASK)
#define DISABLE_TIMER_COUNTER1 (TCCR1B &= ~TIMER_COUNTER1_CS_MASK)
#define DISABLE_TIMER_COUNTER2 (TCCR2B &= ~TIMER_COUNTER2_CS_MASK)

#define ENABLE_TIMER_COUNTER0 (TCCR0B |= ((1 << CS02) | (0 << CS01) | (0 << CS00)))
#define ENABLE_TIMER_COUNTER1 (TCCR1B |= ((1 << CS12) | (0 << CS11) | (0 << CS10)))
#define ENABLE_TIMER_COUNTER2 (TCCR2B |= ((1 << CS22) | (1 << CS21) | (0 << CS20)))

// pin definitions
#define RX_PIN PIND0
#define TX_PIN PIND1

#define RX_LED_PIN PIND0
#define TX_LED_PIN PIND1
#define L_LED_PIN PINB5

#define OUTPUT_COMPARE_O_A PIND6
#define OUTPUT_COMPARE_1_A PINB1
#define OUTPUT_COMPARE_2_A PINB3

#define BTN_X_POS_PIN PINC1
#define BTN_Y_POS_PIN PINC3
#define BTN_Z_POS_PIN PINC5

#define BTN_X_NEG_PIN PINC0
#define BTN_Y_NEG_PIN PINC2
#define BTN_Z_NEG_PIN PINC4

#define MOTOR_X_DIRECTION_PIN PIND2
#define MOTOR_Y_DIRECTION_PIN PIND3
#define MOTOR_Z_DIRECTION_PIN PIND4

// globals
typedef enum AxisDirection {DIRECTION_POSITIVE, DIRECTION_NEGATIVE, DIRECTION_PAUSE} AxisDirection;
typedef struct MotorStates {
  AxisDirection xDirection;
  AxisDirection yDirection;
  AxisDirection zDirection;
} MotorStates;

typedef struct Potentionmeter {
  uint8_t adcValue;
} Potentiometer;

typedef struct GlobalState {
  Potentiometer potentiometer; 
  MotorStates motorStates;
} GlobalState;
GlobalState globalStates = {
  .potentiometer = {.adcValue = 0 },
  .motorStates= {
    .xDirection = DIRECTION_PAUSE, 
    .yDirection = DIRECTION_PAUSE, 
    .zDirection = DIRECTION_PAUSE
  }
};


void setup() {
  cli();

  // set interrutps to CTC mode, prescaler 1024 and connected compare A pin 
  TCCR0A = (0 << COM0A1) | (1 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (0 << WGM00);
  TCCR1A = (0 << COM1A1) | (1 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
  TCCR2A = (0 << COM2A1) | (1 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (1 << WGM21) | (0 << WGM20);
  
  TCCR0B = (0 << FOC0A) | (0 << FOC0B) | (0 << WGM02);
  DISABLE_TIMER_COUNTER0;
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (1 << WGM12);
  DISABLE_TIMER_COUNTER1; 
  TCCR2B = (0 << FOC2A) | (0 << FOC2B) | (0 << WGM22);
  DISABLE_TIMER_COUNTER2;
  
  OCR0A = 128;    
  OCR1A = 128;
  OCR2A = 128;
  
  DIRD |= (1 << OUTPUT_COMPARE_O_A);  
  DIRB |= (1 << OUTPUT_COMPARE_1_A) | (1 << OUTPUT_COMPARE_2_A);  

  // LEDs
  DIRD |= (1 << RX_LED_PIN) | (1 << TX_LED_PIN);
  DIRB |= (1 << L_LED_PIN);
  
  // ADC in free running mode
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 <<  MUX0); 
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (0 << ADIF) | (0 << ADIE) | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRB = (0 << ACME) | (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
  
  // enable pull-up at each iput button
  DIRC |= (0 << BTN_X_POS_PIN) | (0 << BTN_Y_POS_PIN) | (0 << BTN_Z_POS_PIN) | (0 << BTN_X_NEG_PIN) | (0 << BTN_Y_NEG_PIN) | (0 << BTN_Z_NEG_PIN);
  OUTC |= (1 << BTN_X_POS_PIN) | (1 << BTN_Y_POS_PIN) | (1 << BTN_Z_POS_PIN) | (1 << BTN_X_NEG_PIN) | (1 << BTN_Y_NEG_PIN) | (1 << BTN_Z_NEG_PIN);
  
  // motor direction output pins   
  DIRD |= (1 << MOTOR_X_DIRECTION_PIN);
  DIRD |= (1 << MOTOR_Y_DIRECTION_PIN);
  DIRD |= (1 << MOTOR_Z_DIRECTION_PIN);
  
  sei();
}

// read current ADC value
void readPotentiometerValue(void) {
  //while (0 != (ADCSRA | (1 << ADSC)));
  uint8_t newAdcValue = (uint8_t)(ADC >> 2);
  ADCSRA |= (1 << ADSC);
  
  // update only on exceeded threshold
  if ( (newAdcValue < globalStates.potentiometer.adcValue + 2) &&
       (newAdcValue > globalStates.potentiometer.adcValue - 2) ) {
        return; 
  }
  
  // cap value 
  if (newAdcValue < 4) {
      newAdcValue = 4;
  } else if (newAdcValue > 253) {
      newAdcValue = 253;
  }
  
  globalStates.potentiometer.adcValue = newAdcValue;
  
}

// update signal generating interrupt frequency according to the potentiometer ADC value
void updateInterruptFrequency(void) {
  
  uint8_t timerCounterCsMask = TIMER_COUNTER0_CURRENT_CS_MASK;
  DISABLE_TIMER_COUNTER0;
    OCR0A = globalStates.potentiometer.adcValue;
    if (TCNT0 > OCR0A) {
      TCNT0 = OCR0A - 1;
    }
  RESTORE_TIMER_COUNTER0_CS_MASK(timerCounterCsMask);

  timerCounterCsMask = TIMER_COUNTER1_CURRENT_CS_MASK;  
  DISABLE_TIMER_COUNTER1;
    OCR1A = globalStates.potentiometer.adcValue;
    if (TCNT1 > OCR1A) {
      TCNT1 = OCR1A - 1;
    } 
  RESTORE_TIMER_COUNTER1_CS_MASK(timerCounterCsMask);
  
  timerCounterCsMask = TIMER_COUNTER2_CURRENT_CS_MASK;
  DISABLE_TIMER_COUNTER2;
    OCR2A = globalStates.potentiometer.adcValue;
    if (TCNT2 > OCR2A) {
      TCNT2 = OCR2A - 1;
    }
  RESTORE_TIMER_COUNTER2_CS_MASK(timerCounterCsMask);;
}

// read button states and translate to motor directions
void readButtons(void) {
  globalStates.motorStates.xDirection = DIRECTION_PAUSE;
  globalStates.motorStates.yDirection = DIRECTION_PAUSE;
  globalStates.motorStates.zDirection = DIRECTION_PAUSE;
  
  if (0 == (INC & (1 << BTN_X_POS_PIN))) {
    globalStates.motorStates.xDirection = DIRECTION_POSITIVE;
  }
  if (0 == (INC & (1 << BTN_X_NEG_PIN))) {
    globalStates.motorStates.xDirection = DIRECTION_NEGATIVE;    
  }
  
  if (0 == (INC & (1 << BTN_Y_POS_PIN))) {
    globalStates.motorStates.yDirection = DIRECTION_POSITIVE;
  } else if (0 == (INC & (1 << BTN_Y_NEG_PIN))) {
    globalStates.motorStates.yDirection = DIRECTION_NEGATIVE;
  }
  
  if (0 == (INC & (1 << BTN_Z_POS_PIN))) {
    globalStates.motorStates.zDirection = DIRECTION_POSITIVE;
  } else if (0 == (INC & (1 << BTN_Z_NEG_PIN))) {
    globalStates.motorStates.zDirection = DIRECTION_NEGATIVE;
  }
}

// update motor movement according to requested motor directions
void updateMotorState(void) {
   
  if ( globalStates.motorStates.xDirection == DIRECTION_PAUSE) {
    DISABLE_TIMER_COUNTER0;
    OUTD |= (1 << RX_PIN);
    OUTD &= ~(1 << MOTOR_X_DIRECTION_PIN);
  } else {
   
    if ( globalStates.motorStates.xDirection == DIRECTION_POSITIVE ) {
      OUTD |= (1 << MOTOR_X_DIRECTION_PIN);
    } else {
      OUTD &= ~(1 << MOTOR_X_DIRECTION_PIN);
    }
    
    ENABLE_TIMER_COUNTER0;
    OUTD &= ~(1 << RX_PIN);
  }
  
  if ( globalStates.motorStates.yDirection == DIRECTION_PAUSE) {
    DISABLE_TIMER_COUNTER1;
    OUTD |= (1 << TX_PIN);
    OUTD &= ~(1 << MOTOR_Y_DIRECTION_PIN);
  } else {
    
    if ( globalStates.motorStates.yDirection == DIRECTION_POSITIVE ) {
      OUTD |= (1 << MOTOR_Y_DIRECTION_PIN);
    } else {
      OUTD &= ~(1 << MOTOR_Y_DIRECTION_PIN);
    }
    
    ENABLE_TIMER_COUNTER1;
    OUTD &= ~(1 << TX_PIN);
  }
  
  if ( globalStates.motorStates.zDirection == DIRECTION_PAUSE) {
    DISABLE_TIMER_COUNTER2;
    OUTB &= ~(1 << L_LED_PIN);
    OUTD &= ~(1 << MOTOR_Z_DIRECTION_PIN);
  } else {
    
    if ( globalStates.motorStates.zDirection == DIRECTION_POSITIVE ) {
      OUTD |= (1 << MOTOR_Z_DIRECTION_PIN);
    } else {
      OUTD &= ~(1 << MOTOR_Z_DIRECTION_PIN);
    }
    
    ENABLE_TIMER_COUNTER2;
    OUTB |= (1 << L_LED_PIN);
  }
}

void loop(void) {
  while (1) {
    _delay_ms(100);
    
    readPotentiometerValue();
    updateInterruptFrequency();
    
    readButtons();
    updateMotorState();
  }
}


