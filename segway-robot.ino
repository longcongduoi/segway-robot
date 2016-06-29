#include "pid.h"
#include "lightmpu.h"

// MPU parameters
const int16_t MPU_OFFSETS[] = { 266, -239, 2528, 0, 50, 9, 13 };
const int16_t ALPHA = 1;
const int MPU_ADDR = 0x68;
const mpuconfig MPU_CONF = {
    .disableTemp = true,
    .lowpass = 3,
    .sampleRateDivider = 0,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true
};

// Motor pin configuration
const int LEFT_A = 6, LEFT_B = 10,
    RIGHT_A = 5, RIGHT_B = 9;

// Motor PID controller params
const int16_t MOTOR_P = 16, MOTOR_I = 0, MOTOR_D = 0;
// Output limits
const int16_t MOTOR_MIN = -255, MOTOR_MAX = 255;

// Runtime global variables
volatile bool gMpuInterrupt = false;
int16_t gPitch = 0, gMotorPwm = 0, gPitchSetPoint = 0, gMotorBias = 0;
pidparams gMotorPid;
mpufilter gMpuFilter;

void setupMotors() {
    pinMode(LEFT_A, OUTPUT);
    pinMode(LEFT_B, OUTPUT);
    pinMode(RIGHT_A, OUTPUT);
    pinMode(RIGHT_B, OUTPUT);
}

void updateMotors() {
    updateMotor(LEFT_A, LEFT_B, gMotorPwm + gMotorBias);
    updateMotor(RIGHT_A, RIGHT_B, gMotorPwm - gMotorBias);
}

void updateMotor(int pinA, int pinB, int pwm) {
    if (pwm >= 0) {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, pwm);
    } else {
        digitalWrite(pinA, HIGH);
        analogWrite(pinB, 255 + pwm);
    }
}

void mpuInterrupt() {
    gMpuInterrupt = true;
}

void setup() {
    setupMotors();

    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000L);

    mpuSetup(MPU_ADDR, &MPU_CONF);
    mpuSetupFilter(&MPU_CONF, &gMpuFilter, ALPHA);
    attachInterrupt(digitalPinToInterrupt(2), mpuInterrupt, RISING);
    mpuReadIntStatus(MPU_ADDR);

    pidInit(&gMotorPid, &gPitchSetPoint, &gPitch, &gMotorPwm,
        MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_MIN, MOTOR_MAX);
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

        Serial.println(gMotorPwm);
    }
}

