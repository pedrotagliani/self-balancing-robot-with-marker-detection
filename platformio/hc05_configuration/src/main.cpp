#include <Arduino.h>
#include <SoftwareSerial.h>  // Incluimos la librería  SoftwareSerial

// Create an instance of SoftwareSerial class
SoftwareSerial bluetoothModule(15, 14); // RX, TX
// RX pin from bluetooth module connects to TX pin (14 ---> A0) from Arduino Uno
// TX pin from bluetooth module connects to RX pin (15 ---> A1) from Ardino Uno

// Serial monitor           9600       Both NL & CR
// The ">" character indicates the user entered text.
// AT+NAME=name         AT+UART=115200,0,0          */
// Hold the bluetooth module's bottom button every time you send an AT command

char c=' ';
boolean NL = true;

void setup() {
Serial.begin(9600);
bluetoothModule.begin(115200);    // this is for Bluetooth in AT mode, irrespective of comms speed.
Serial.println("Serial active...enter  AT command");
}

void loop() { 
      if (bluetoothModule.available())   
    {
        c = bluetoothModule.read();
        Serial.write(c);
    }
 
    if (Serial.available())     
    {
        c = Serial.read();
        bluetoothModule.write(c);       
        if (NL) { Serial.print(">");  NL = false; }  // Echo 
        Serial.write(c);
        if (c==10) { NL = true; }
    }
}