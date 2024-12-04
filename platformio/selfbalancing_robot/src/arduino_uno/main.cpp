#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include <PID_v1.h>

// Functions declaration
int deg_to_steps(float degAngle);
float pitch();
void processBluetoothData(String data);
bool check_mpu();

// Create an instance of SoftwareSerial class
// SoftwareSerial bluetoothModule(15, 14); // RX, TX
// RX pin from bluetooth module connects to TX pin (14 ---> A0) from Arduino Uno
// TX pin from bluetooth module connects to RX pin (15 ---> A1) from Ardino Uno

// Stepper 1 ---> Left motor
// Stepper 2 ---> Right motor

// Nema 17 stepper 1 pins
#define STEP_PIN1 4 // PORTD, bit 4
#define DIR_PIN1 5 // PORTD, bit 5
#define MS1_PIN1 8
#define MS2_PIN1 9
#define MS3_PIN1 10

// Nema 17 stepper 2 pins
#define STEP_PIN2 6 // PORTD, bit 6
#define DIR_PIN2 7 // PORTD, bit 7
#define MS1_PIN2 11
#define MS2_PIN2 12
#define MS3_PIN2 13

#define ZERO_SPEED 65535
#define MAX_ACCEL 7

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

// 1/16 microstepping variables
// #define STEPS_PER_REVOLUTION 3200
// #define DEGREES_PER_STEP 0.1125

// Full stepping variables
// #define STEPS_PER_REVOLUTION 200
// #define DEGREES_PER_STEP 1.8

// 1/8 microstepping variables
#define STEPS_PER_REVOLUTION 1600.0
#define DEGREES_PER_STEP 0.225

// 1/4 microstepping variables
// #define STEPS_PER_REVOLUTION 800
// #define DEGREES_PER_STEP 0.45

// Create an instance of MPU6050 class
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu(0x68);
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU6050 resources:
// https://www.luisllamas.es/arduino-orientacion-imu-mpu-6050/#obtener-la-orientaci%C3%B3n-con-filtro-complementario
// https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
// https://www.luisllamas.es/medir-la-inclinacion-imu-arduino-filtro-complementario/
// https://github.com/nzipin/Self-Balancing-Robot/tree/master
// https://www.sinaptec.alomar.com.ar/2017/10/tutorial-23-esp8266-obtener-inclinacion.html

// Interrupt pin from MPU6050
// If new data is available, the MPU6050 will send a pulse on this interrupt pin
#define INTERRUPT_PIN 2

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Set the flag related to the interrupt (if mpuInterrupt is True, new data is available)
volatile bool MPUInterrupt  = false;
// The volatile keyword tells the compiler that a variable can be changed by something outside the normal program flow (the interrupt, in this case)

// The function to execute when the interrupt triggers
void DMPDataReady() {
    MPUInterrupt = true;
}

// PID constants
double kP = 17; // 24 good
double kI = 100;
double kD = 0.4; // 0.8 a lot

// PID variables
double setpoint = 0;
double input;
double output = 0;

PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup

unsigned long previousMillis = millis();
unsigned long currentMillis;
const long interval = 5;
// If this interval is too low (for example, interval = 1), this will case an interfereance in the measurements of the MPU6050
// And thus, will generate an undesired offset of the measurements. For example, 0° may be transformed in -6.3°, which doesn't makes sense

// unsigned long previousMillisBluetooth = millis();
// unsigned long currentMillisBluetooth;
// const long intervalBluetooth = 160;

float lastMPUMeasurement;
bool lastMPUMeasurementInitialized = false;
// bool happendLastTime = false;

int16_t motor1;
int16_t motor2;

int16_t speed_m[2];           // Actual speed of motors
uint8_t dir_m[2];             // Actual direction of steppers motors
int16_t actual_robot_speed;          // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;          // overall robot speed (measured from steppers speed)
float estimated_speed_filtered;

uint16_t counter_m[2];        // counters for periods
uint16_t period_m[2][8];      // Eight subperiods 
uint8_t period_m_index[2];    // index for subperiods

// 200ns => 4 instructions at 16Mhz
void delay_200ns()  
{
  __asm__ __volatile__ (
		"nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
		"nop"); 
}

ISR(TIMER1_COMPA_vect)
{
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]])
    {
    counter_m[0] = 0;
    if (period_m[0][0]==ZERO_SPEED)
      return;
    if (dir_m[0])
      SET(PORTD,5);  // DIR Motor 1
    else
      CLR(PORTD,5);
    // We need to wait at lest 200ns to generate the Step pulse...
    period_m_index[0] = (period_m_index[0]+1)&0x07; // period_m_index from 0 to 7
    //delay_200ns();
    SET(PORTD,4); // STEP Motor 1
    delayMicroseconds(1);
    CLR(PORTD,4);
    }
  if (counter_m[1] >= period_m[1][period_m_index[1]])
    {
    counter_m[1] = 0;
    if (period_m[1][0]==ZERO_SPEED)
      return;
    if (dir_m[1])
      SET(PORTD,7);   // DIR Motor 2
    else
      CLR(PORTD,7);
    period_m_index[1] = (period_m_index[1]+1)&0x07;
    //delay_200ns();
    SET(PORTD,6); // STEP Motor 2
    delayMicroseconds(1);
    CLR(PORTD,6);
    }
}

// Dividimos en 8 subperiodos para aumentar la resolucion a velocidades altas (periodos pequeños)
// subperiod = ((1000 % vel)*8)/vel;
// Examples 4 subperiods:
// 1000/260 = 3.84  subperiod = 3
// 1000/240 = 4.16  subperiod = 0
// 1000/220 = 4.54  subperiod = 2
// 1000/300 = 3.33  subperiod = 1 
void calculateSubperiods(uint8_t motor)
{
  int subperiod;
  int absSpeed;
  uint8_t j;
  
  if (speed_m[motor] == 0)
    {
    for (j=0;j<8;j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
    }
  if (speed_m[motor] > 0 )   // Positive speed
    {
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
    }
  else                       // Negative speed
    {
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
    }
    
  for (j=0;j<8;j++)
    period_m[motor][j] = 1000/absSpeed;
  // Calculate the subperiod. if module <0.25 => subperiod=0, if module < 0.5 => subperiod=1. if module < 0.75 subperiod=2 else subperiod=3
  subperiod = ((1000 % absSpeed)*8)/absSpeed;   // Optimized code to calculate subperiod (integer math)
  if (subperiod>0)
   period_m[motor][1]++;
  if (subperiod>1)
   period_m[motor][5]++;
  if (subperiod>2)
   period_m[motor][3]++;
  if (subperiod>3)
   period_m[motor][7]++;
  if (subperiod>4)
   period_m[motor][0]++;
  if (subperiod>5)
   period_m[motor][4]++;
  if (subperiod>6)
   period_m[motor][2]++;
  
  // DEBUG
  /*
  if ((motor==0)&&((debug_counter%10)==0)){
    Serial.print(1000.0/absSpeed);Serial.print("\t");Serial.print(absSpeed);Serial.print("\t");
    Serial.print(period_m[motor][0]);Serial.print("-");
    Serial.print(period_m[motor][1]);Serial.print("-");
    Serial.print(period_m[motor][2]);Serial.print("-");
    Serial.println(period_m[motor][3]);
    }
  */  
}

void setMotorSpeed(uint8_t motor, int16_t tspeed)
{
  // WE LIMIT MAX ACCELERATION
  if ((speed_m[motor] - tspeed)>MAX_ACCEL)
    speed_m[motor] -= MAX_ACCEL;
  else if ((speed_m[motor] - tspeed)<-MAX_ACCEL)
    speed_m[motor] += MAX_ACCEL;
  else
    speed_m[motor] = tspeed;
  
  calculateSubperiods(motor);  // We use four subperiods to increase resolution
  
  // To save energy when its not running...
  // if ((speed_m[0]==0)&&(speed_m[1]==0))
  //   digitalWrite(4,HIGH);   // Disable motors
  // else
  //   digitalWrite(4,LOW);   // Enable motors
}

void setup() {
  Serial.begin(115200);

  // delay(2000);

  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN2, LOW);

  // pinMode(ENA_PIN1, OUTPUT);
  // pinMode(ENA_PIN2, OUTPUT);

  // // Disable motors
  // digitalWrite(ENA_PIN1, HIGH);
  // digitalWrite(ENA_PIN2, HIGH);

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

  pinMode(INTERRUPT_PIN, INPUT);

// https://github.com/ElectronicCats/mpu6050/issues/16
// https://github.com/ElectronicCats/mpu6050/pull/69
// https://forum.arduino.cc/t/solved-i2cdev-and-wire-hangs-or-something-electrical/575044

// Possible alternative (but also use Wire.h): https://github.com/gabriel-milan/TinyMPU6050

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      Wire.setWireTimeout(3000, true); //timeout value in uSec
      // https://github.com/jrowberg/i2cdevlib/issues/543
      // https://github.com/jrowberg/i2cdevlib/issues/519
      // https://www.fpaynter.com/2020/07/i2c-hangup-bug-cured-miracle-of-miracles-film-at-11/
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // bluetoothModule.begin(9600);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  // mpu.setRate(5); // https://github.com/ElectronicCats/mpu6050/issues/16
  devStatus = mpu.dmpInitialize();

  // Calibration values obtained (offsets)
  mpu.setXAccelOffset(877);
  mpu.setYAccelOffset(-2439);
  mpu.setZAccelOffset(1722);
  mpu.setXGyroOffset(108);
  mpu.setYGyroOffset(15);
  mpu.setZGyroOffset(100);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    // mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  //We are going to overwrite the Timer1 to use the stepper motors
  
  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 
  
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  OCR1A = 80;   // 25Khz
  TCNT1 = 0;

  delay(1000);

  TIMSK1 |= (1<<OCIE1A);  // Enable Timer1 interrupt
  // digitalWrite(4,LOW);    // Enable stepper drivers
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-500,500);

  Serial.flush();
}

// char bluetoothBuffer[100];
// unsigned int bufferIndex = 0;


void loop() {
  // if programming failed, don't try to do anything
  if (!DMPReady)
  {
    Serial.println("DMP not ready!");
    return;
  }

  // currentMillis = millis();

  // if (currentMillis - previousMillisBluetooth > intervalBluetooth) {
  //   previousMillisBluetooth = currentMillis;
  //   if (bluetoothModule.available()) {
  //     String receivedData = bluetoothModule.readStringUntil('\n'); // Read a line of data
  //     processBluetoothData(receivedData); // Process the received data
  //   }
  // }

  // if (currentMillis - previousMillisBluetooth > intervalBluetooth) {
  //   if (bluetoothModule.available()) {
  //       char receivedChar = bluetoothModule.read();
  //       if (receivedChar == '\n') {
  //           bluetoothBuffer[bufferIndex] = '\0'; // Finaliza la cadena
  //           processBluetoothData(String(bluetoothBuffer));
  //           bufferIndex = 0; // Resetea el índice
  //       } else if (bufferIndex < sizeof(bluetoothBuffer) - 1) {
  //           bluetoothBuffer[bufferIndex++] = receivedChar;
  //       }
  //   }
  //  }

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // Serial.println(ypr[1] * 180/M_PI);
      // Serial.println(output);
      if (lastMPUMeasurementInitialized == false) {
        lastMPUMeasurement = pitch();
        input = pitch();
        pid.Compute();
        lastMPUMeasurementInitialized = true;
      } else {
        if (pitch() < 10 + lastMPUMeasurement && pitch() > lastMPUMeasurement - 10){
          input = pitch();
          pid.Compute();
        }
      }
  }
    // Serial.println(output);

  setMotorSpeed(0, output);
  setMotorSpeed(1, output);
}



int deg_to_steps(float degAngle) {
    return round(degAngle*(STEPS_PER_REVOLUTION/360.0));
    // [deg] to [steps]
    // [deg/s] to [steps/s]
    // [degs/s2] to [steps/s2]
}

// In this case, we'll use the pitch angle based on the position of the MPU6050
// This function converts the values from the accel-gyro arrays into degrees
float pitch() {
  return (ypr[1] * 180/M_PI);
}

bool check_mpu() {

  bool MPUFound = false;

  for (int tries = 0; tries < 20; tries++)
  {
    Wire.beginTransmission (0x68);
    if (Wire.endTransmission () == 0)
      {
        MPUFound = true;
        break;
      }
      delay(10);
  }
  return MPUFound;
}


void processBluetoothData(String data) {
  // Split the input string into tokens based on commas
  int kPIndex = data.indexOf("kP:");
  int kIIndex = data.indexOf("kI:");
  int kDIndex = data.indexOf("kD:");

  // Parse kP value
  if (kPIndex != -1) {
    int endIndex = data.indexOf(',', kPIndex);
    String value = data.substring(kPIndex + 3, endIndex == -1 ? data.length() : endIndex);
    kP = value.toDouble();
    pid.SetTunings(kP, kI, kD); // Update PID tunings
  }

  // Parse kI value
  if (kIIndex != -1) {
    int endIndex = data.indexOf(',', kIIndex);
    String value = data.substring(kIIndex + 3, endIndex == -1 ? data.length() : endIndex);
    kI = value.toDouble();
    pid.SetTunings(kP, kI, kD); // Update PID tunings
  }

  // Parse kD value
  if (kDIndex != -1) {
    int endIndex = data.indexOf(',', kDIndex);
    String value = data.substring(kDIndex + 3, endIndex == -1 ? data.length() : endIndex);
    kD = value.toDouble();
    pid.SetTunings(kP, kI, kD); // Update PID tunings
  }

  // Print updated PID values for debugging
  Serial.print("Updated PID values -> kP: ");
  Serial.print(kP);
  Serial.print(", kI: ");
  Serial.print(kI);
  Serial.print(", kD: ");
  Serial.println(kD);
}
