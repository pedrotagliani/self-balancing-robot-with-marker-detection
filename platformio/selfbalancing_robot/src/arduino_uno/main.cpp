#include <Arduino.h>
#include "AccelStepper.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include "I2Cdev.h"

SoftwareSerial bluetoothModule(1, 0); // RX, TX

// Nema 17 stepper 1 pins
#define STEP_PIN1 4
#define DIR_PIN1 5
#define MS1_PIN1 8
#define MS2_PIN1 9
#define MS3_PIN1 10

// Nema 17 stepper 2 pins
#define STEP_PIN2 6
#define DIR_PIN2 7
#define MS1_PIN2 11
#define MS2_PIN2 12
#define MS3_PIN2 13

// Create the instances of the AccelStepper class
AccelStepper stepper1 = AccelStepper(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2 = AccelStepper(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

// 1/16 microstepping variables
// #define STEPS_PER_REVOLUTION 3200
// #define DEGREES_PER_STEP 0.1125

// 1/8 microstepping variables
#define STEPS_PER_REVOLUTION 1600.0
#define DEGREES_PER_STEP 0.225

// 1/4 microstepping variables
// #define STEPS_PER_REVOLUTION 800
// #define DEGREES_PER_STEP 0.45

MPU6050 mpu;

// RAW values (unprocessed) from the accelerometer and gyroscope on the x, y, z axes
int16_t ax, ay, az;
int16_t gx, gy, gz;

int deg_to_steps(float degAngle);

void setup() {

  Serial.begin(9600);
  bluetoothModule.begin(38400);

  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("Sensor iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor");
  }

  // Set microstepping pins as output
  pinMode(MS1_PIN1, OUTPUT);
  pinMode(MS2_PIN1, OUTPUT);
  pinMode(MS3_PIN1, OUTPUT);
  pinMode(MS1_PIN2, OUTPUT);
  pinMode(MS2_PIN2, OUTPUT);
  pinMode(MS3_PIN2, OUTPUT);

  // Set the 1/8 microstepping level for the stepepr1
  digitalWrite(MS1_PIN1, HIGH);
  digitalWrite(MS2_PIN1, HIGH);
  digitalWrite(MS3_PIN1, LOW);

  // Set the 1/8 microstepping level for the stepepr2
  digitalWrite(MS1_PIN2, HIGH);
  digitalWrite(MS2_PIN2, HIGH);
  digitalWrite(MS3_PIN2, LOW);

  // Set the current position as 0° for the steppers
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  // Set the maximum speed of the steppers
  stepper1.setMaxSpeed(1000.0);
  stepper2.setMaxSpeed(1000.0);

}

void loop() {
  //////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////// Steppers test //////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////

  // Move the stepper1 to 90°
  stepper1.setAcceleration(200);
  stepper1.moveTo(deg_to_steps(90));
  while (stepper1.currentPosition() != (deg_to_steps(90)))  {
    stepper1.run();
  }

  // Move the stepper2 to 90°
  stepper2.setAcceleration(200);
  stepper2.moveTo(deg_to_steps(90));
  while (stepper2.currentPosition() != (deg_to_steps(90)))  {
    stepper2.run();
  }

  // Move both steppers at the same time to different target positions
  stepper1.setAcceleration(100);
  stepper2.setAcceleration(100);
  stepper1.moveTo(deg_to_steps(90+45));
  stepper2.moveTo(deg_to_steps(90+90));

  while (stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0) {
    stepper1.run(); // Non-blocking step for motor 1
    stepper2.run(); // Non-blocking step for motor 2
    // run() method in the AccelStepper library is designed to be non-blocking and state-aware
  }

  delay(5000);

  /////////////////////////////////////////////////////////////////////////////
  //////////////////////////////// HC-05 test /////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  // Follow this tutorial: https://programarfacil.com/blog/arduino-blog/robot-arduino-bluetooth-hc05/

  // // Receive ---------------> Check if there is any data available on the Bluetooth module's port
  // if (bluetoothModule.available())
  // {
  //   // Display the received information on the serial monitor
  //   Serial.write(bluetoothModule.read());
  // }

  // // Send ------------> Check if there is any data in the serial monitor to be sent to the Bluetooth module
  // if (Serial.available())
  // {
  //   // Send the information via Bluetooth
  //   bluetoothModule.write(Serial.read());
  // }


  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// MPU6050 test /////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  // // Read the accelerations and angular velocities
  // mpu.getAcceleration(&ax, &ay, &az);
  // mpu.getRotation(&gx, &gy, &gz);

  // // Display the readings separated by a [tab]
  // // Serial.print("a[x y z] g[x y z]:\t");
  // Serial.print(ax); Serial.print("\t");
  // Serial.print(ay); Serial.print("\t");
  // Serial.print(az); Serial.print("\t");
  // Serial.print(gx); Serial.print("\t");
  // Serial.print(gy); Serial.print("\t");
  // Serial.println(gz);

  // delay(100);

}


// pio run -e uno -t upload

int deg_to_steps(float degAngle){
    return round(degAngle*(STEPS_PER_REVOLUTION/360.0));
    // [deg] to [steps]
    // [deg/s] to [steps/s]
    // [degs/s2] to [steps/s2]
}
