#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 0
#define LED_PIN 3
#define ANGLE_DEADBAND 0.7 // degrees


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = 212.5;
double setpoint = originalSetpoint - 0.8;
double movingAngleOffset = 2;
double input, output;

double Kp = 26;
double Kd = 1.0;
double Ki = 60;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ===== YOUR MOTOR VALUES =====
double motorSpeedFactorLeft  = 0.85;
double motorSpeedFactorRight = 0.9;

int ENA = 9;
int IN1 = 6;
int IN2 = 7;
int IN3 = 4;
int IN4 = 5;
int ENB = 10;

// IN3 & IN4 swapped (your fix)
LMotorController motorController(
  ENA, IN1, IN2,
  ENB, IN4, IN3,
  motorSpeedFactorLeft,
  motorSpeedFactorRight
);

// Interrupt
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#endif

  Serial.begin(115200);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // ===== YOUR REAL OFFSETS =====
  mpu.setXGyroOffset(258);
  mpu.setYGyroOffset(-732);
  mpu.setZGyroOffset(-163);
  mpu.setZAccelOffset(1688);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);

    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(6);
    pid.SetOutputLimits(-170, 170);
  }
  else
  {
    Serial.print(F("DMP init failed: "));
    Serial.println(devStatus);
  }
  pinMode(LED_PIN, OUTPUT);

}

void loop()
{
  if (!dmpReady) return;
  if (!mpuInterrupt) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    return;
  }

  if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180 / M_PI + 180;
    if (abs(input - setpoint) < 1.0) {
  digitalWrite(3, HIGH);
} else {
  digitalWrite(3, LOW);
}

    double angleError = input - setpoint;
     double error = input - setpoint;

if (abs(error) < ANGLE_DEADBAND) {
  pid.SetMode(MANUAL);      // stops integral accumulation
  output = 0;
  motorController.move(0, 0);
  return;
} else {
  pid.SetMode(AUTOMATIC);   // resume PID when falling
}


    pid.Compute(); 

    motorController.move(output, MIN_ABS_SPEED);
  }
}
