// Include Libraries
#include <Servo.h>                //Using servo library to control ESC
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SD.h>                   // SD Card Library
#include <Encoder.h>              // Encoder Library



// Create Classes
MPU6050 mpu;
Servo esc; 
Servo esc2;
Encoder myEnc(5, 6);   // Encoder



// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_YAWPITCHROLL
Quaternion q;   
VectorFloat gravity;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pitch;
float pitchr;
float g_x;

unsigned long prevTime;
unsigned long elapseTime;

long oldPos = 0;
long newPos = 0;

const int chipSelect = BUILTIN_SDCARD; // datalogger
unsigned long currenTime;
int g = 0;
int f = 1040;
float prev_error;
float cum_error;

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

    esc2.attach(7);
    esc.attach(8); //Specify the esc signal pin,Here as D8
    esc.writeMicroseconds(1000); //initialize the signal to 1000
    esc2.writeMicroseconds(1000);
    delay(2000);
    Serial.println("ESC SETUP");
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    mpu.initialize();

    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(mpu.getXGyroOffset());
    mpu.setYGyroOffset(mpu.getYGyroOffset());
    mpu.setZGyroOffset(mpu.getZGyroOffset());
    mpu.setZAccelOffset(mpu.getZAccelOffset());

    if (devStatus == 0) {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      attachInterrupt(4, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    Serial.println("DMP SETUP");
    // SD card setup
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
}

void loop() {
  // if programming failed, don't try to do anything
  //if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
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
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        //Serial.println("working");
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            gx = gx/131;
            gx = gx*(M_PI/180);
            pitchr = ypr[2];
            pitch = pitchr * 180/M_PI;
        #endif
  
  }
  // Encoder Angle
currenTime = millis();
oldPos = newPos;
newPos = myEnc.read();
newPos = (newPos/4)/180 * M_PI ; // get encoder angle in radians
deriv_enc = (newPos - oldPos)/(currenTime - prevTime);

 
  // Serial Input
   if(Serial.available() > 0){
      g = Serial.parseInt();
   }

   if (g != 0){
      sp = g;
    }

    // State Feedback Controller
    float error = sp - pitchr;
    cum_error = error + prev_error;
    prev_error = cum_error;

    float error2 = 0 - newPos;
    cum_error2 = error2 + prev_error2;
    prev_error2 = cum_error2;
    
    float u_1 = -0.97*cum_error;
    float state = 1.01*gx + 1.8*pitchr -0.25*deriv_enc - 0.43newPos;
    float F1 = -u_1 - state;
    float u_2 = -0.98*cum_error2;
    float state2 = 0.29*gx + 0.54*pitchr + 1.07*deriv_enc + 1.74*newPos;
    float F2 = -u_2 - state2;
    float control1 = (F1 + 5.976)/0.0057;
    float control2 = (F2 + 6.2217)/0.0057;
   
    // Limit outputs
    if (control > 1150){
      control = 1150;
      }

    if (control2 > 1150){
      control2 = 1150;
      }

      
    // Write control action
    esc.writeMicroseconds(control1); //using val as the signal to esc
    esc2.writeMicroseconds(control2); 
    

    // Logging Data
    String dataString = "";
    dataString = String(pitch) + String(",") + String(newPos) +String(",") +String(currenTime) ;
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }
    prevTime = currenTime;  


}
