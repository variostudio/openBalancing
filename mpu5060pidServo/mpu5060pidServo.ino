/*
  This is a first example of using MPU5060 with PID algorithm of error correcting and servo rotating MPU sensor.
It is used Yaw axis to stabilize.

The software is used in this example:
1. I2Cdev library https://github.com/jrowberg/i2cdevlib
2. MPU5060 Arduino library found here http://playground.arduino.cc/Main/MPU-6050 and here http://diyhacking.com/projects/MPU6050.zip
3. Servo.h found in Arduino sketches
4. PID algorithm got from ArduinoPID library

 Thanks all guys for their work because it is really helpfull
 */

/*
I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
  ========================================================================= 

  =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <Servo.h>

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define AXIS_NUMBER 3

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define TUNE_PIN 0 //Analog pin for tuning pids by potentiometer
#define SERVO_PIN 9 //Servo attached to

#define LED_BLINK_INTERVAL_INITIALIZE  250;
#define LED_BLINK_INTERVAL_LOOP  1000;

bool blinkState = false;
int ledBlinkInterval = 250;
long ledSwitchTime;
bool zeroInitialized = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Servo myservo;

unsigned long startTime;

double kp[AXIS_NUMBER], ki[AXIS_NUMBER], kd[AXIS_NUMBER];
double zeroValue[AXIS_NUMBER];
double errorSum[AXIS_NUMBER];
double errorLast[AXIS_NUMBER];


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    myservo.attach(SERVO_PIN); //Yaw servo is attached to pin #9
    myservo.write(90);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING); //Arduino pin #2
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    for (int i=0; i<AXIS_NUMBER; i++) {
      ki[i] = 0.0086;
      kp[i] = 0.0086;
      kd[i] = 0.0086;

      errorSum[i] = 0;
      errorLast[i] = 0;
      zeroValue[i] = 0;
    }

    ledSwitchTime = millis();
    Serial.println("Please wait 20 seconds for setting zeros\t");
    startTime = millis();
}

// ================================================================
// ===                        ROUTINES                          ===
// ================================================================

//Calculates control value based on PID algorithm
double calcControlPID(double input, double setPoint, int axe) {
    unsigned long now = millis();
    double timeChange = (double)(now - startTime);

    double error = setPoint - input;

    errorSum[axe] += error*timeChange;
    double dError = (error - errorLast[axe])/timeChange;

    double res = kp[axe]*error + ki[axe]*errorSum[axe] + kd[axe]*dError;
    
    errorLast[axe] = error;
    return res;
}

//Processes each axis, calculates required control valu and outputs to Monitor
double processAxis(String axisName, int axis) {
    double value = ypr[axis] * 180/M_PI;
    double controlValue = calcControlPID(value, zeroValue[axis], axis);

    /*
    Serial.print(axisName);
    Serial.print("\t");
    Serial.print(zeroValue[axis] - value);
    Serial.print("\t");
    Serial.print(controlValue);
    Serial.println();
    */

    return controlValue;
}

//Tuning PID values by potentiometer 
void setUpPIDfromPotentiometer() {
    int value = analogRead(TUNE_PIN);

    for (int i=0; i<AXIS_NUMBER; i++) {
        ki[i] = (double)value/10000;
        kp[i] = (double)value/10000;
        kd[i] = (double)value/10000;
    }  
}

void printPIDs() {
    Serial.print("PID: ");
    for (int i=0; i<AXIS_NUMBER; i++) {
        Serial.print(kp[i], 4);
        Serial.print("\t");
        Serial.print(ki[i], 4);
        Serial.print("\t");
        Serial.print(kd[i], 4);
        Serial.print("\t");
    }  
    Serial.println("");
}

//Converts each control value to servo movement
void controlToServo(double value) {
    double v = (-1)*value;
    if (v < -90) {
        v = -90;
    }
    if (v > 90) {
        v = 90;
    }
    int pos = map(v, -90, 90, 0, 180);
    myservo.write(pos);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if (zeroInitialized) {
        startTime = millis();
    }
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if (!zeroInitialized) {
            ledBlinkInterval = LED_BLINK_INTERVAL_INITIALIZE;

            for (int i=0; i<AXIS_NUMBER; i++) {
                zeroValue[i] = ypr[i] * 180/M_PI;
            }
            
            if (millis() - startTime > 20000) {
                zeroInitialized = true;
    
                Serial.print("Zeros are set to\t");

                for (int i=0; i<AXIS_NUMBER; i++) {
                  Serial.print(zeroValue[i]);
                  Serial.print("\t");
                }
                
                Serial.print("\n");
            }

        } else {
            ledBlinkInterval = LED_BLINK_INTERVAL_LOOP;

            //Uncomment this line to have Arduino PIDs configured with potentiometer
            //setUpPIDfromPotentiometer();  
          
            double controlValueYaw = processAxis("Yaw", 0);
            controlToServo(controlValueYaw);

            printPIDs();
//            processAxe("Pitch", 1);
//            processAxe("Roll", 2);
        }

        if (millis() - ledSwitchTime >= ledBlinkInterval) {
            blinkState = !blinkState;
            ledSwitchTime = millis();
        }
        digitalWrite(LED_PIN, blinkState);
    }
}
