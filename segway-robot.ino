#include "pid.h"
#include "lightmpu.h"

// MPU parameters
const int16_t MPU_OFFSETS[] = { 266, -239, 2528, 0, 50, 9, 13 };
const int16_t ALPHA = 2;
const int MPU_ADDR = 0x68;
const mpuconfig MPU_CONF = {
    .disableTemp = true,
    .lowpass = 3,
    .sampleRateDivider = 0,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true
};

// Motor PID controller params
const int16_t MOTOR_P = 16, MOTOR_I = 0, MOTOR_D = 0;
// Output limits
const int16_t MOTOR_MIN = -255, MOTOR_MAX = 255;

// Runtime global variables
volatile bool gMpuInterrupt = false;
int16_t gPitch = 0, gMotorPwm = 0, gPitchSetPoint = 0, gMotorBias = 0;
volatile uint8_t gLeftPwm = 0, gRightPwm = 0, gPortDclr = 0xFF, gPortDset = 0;

pidparams gMotorPid;
mpufilter gMpuFilter;

ISR(TIMER1_OVF_vect) {
    PORTD &= gPortDclr;
    PORTD |= gPortDset;

    OCR1A = gLeftPwm;
    OCR1B = gRightPwm;
}

ISR(INT0_vect) {
    gMpuInterrupt = true;
}

void setupMotors() {
    // Set the motor pins as outputs
    DDRB |= _BV(PB1) | _BV(PB2);
    DDRD |= _BV(PD5) | _BV(PD6);
    // Clear the motor bits
    PORTD &= ~(_BV(PD5) | _BV(PD6));

    // Set up the timer for fast PWM at 1 kHz with overflow interrupt
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS11);
    TIMSK1 = _BV(TOIE1);
}

void updateMotors() {
    int pwmLeft = gMotorPwm + gMotorBias,
        pwmRight = gMotorPwm - gMotorBias;

    gPortDclr = 0xFF;
    gPortDset = 0;

    if (pwmLeft >= 0) {
        gPortDclr &= ~_BV(PD6);
    } else {
        gPortDset |= _BV(PD6);
        pwmLeft += 255;
    }

    if (pwmRight >= 0) {
        gPortDclr &= ~_BV(PD5);
    } else {
        gPortDset |= _BV(PD5);
        pwmRight += 255;
    }

    gLeftPwm = pwmLeft;
    gRightPwm = pwmRight;
}

void setup() {
    Wire.begin();
    Wire.setClock(400000L);

    mpuSetup(MPU_ADDR, &MPU_CONF);
    mpuSetupFilter(&MPU_CONF, &gMpuFilter, ALPHA);

    // Setup the MPU pin change interrupt
    // Detect rising edges on INT0 pin
    EICRA = _BV(ISC01) | _BV(ISC00);
    EIMSK = _BV(INT0);
    mpuReadIntStatus(MPU_ADDR);

    pidInit(&gMotorPid, &gPitchSetPoint, &gPitch, &gMotorPwm,
        MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_MIN, MOTOR_MAX);

    setupMotors();
}

void loop() {
    int16_t mpuData[7];

    if (gMpuInterrupt) {
        gMpuInterrupt = false;

        mpuReadIntStatus(MPU_ADDR);
        mpuReadRawData(MPU_ADDR, mpuData);
        mpuApplyOffsets(mpuData, MPU_OFFSETS);
        mpuUpdatePitch(&gMpuFilter, mpuData, &gPitch);

        pidCompute(&gMotorPid);
        updateMotors();
    }
}

