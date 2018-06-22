#include <Wire.h>

// servo driver address 0x40
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMID  360

// MPU6050
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t acX,acY,acZ,tmp,gyX,gyY,gyZ;
double aax,aay,aaz,at,agx,agy,agz;

typedef struct {
  char data[25];
  bool reading;
  int idx;
} Message;

// Messaging
Message pc; // from pc/python over usb

void setup() {
  wakeUpMPU();
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  Serial.begin(9600);
  Serial.println("spiduino ready");
}

void loop() {
  getMPU();
  readMsg();
  delay(10);
}

void readMsg() {
  while(Serial.available()) {
    char c = Serial.read();
    if (pc.reading) {
      if (c == '>') {
        pc.reading = false;
        pc.data[pc.idx] = '\0';
        receivedMessage(pc);
      } else {
        // read in char
        pc.data[pc.idx] = c;
        pc.idx++;
      }
    } else if (c == '<') {
      pc.reading = true;
      pc.idx = 0;
    }
  }
}

void receivedMessage(Message msg) {
  if (msg.data[0] == 'm') {
    // respond with motion data
    printMPU();
    return;
  } else if (msg.data[0] == 's') {
    int angles[8];

    char cs[4];
    cs[3] = '\0';

    int i, j;
    for (i = 0; i < 8; i+=1) {
      for (j = 0; j < 3; j++) {
        cs[j] = msg.data[1+i*3+j];
      }
      
      angles[i] = atoi(cs);
    }
    
    set(angles);
  } else {
    Serial.println("unknown message");
  }
}

void wakeUpMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void getMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  acX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  acY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  tmp = tmp/340.00+36.53; // to degrees C from datasheet

  // avg everything out
  aax += (acX-aax)*.2;
  aay += (acY-aay)*.2;
  aaz += (acZ-aaz)*.2;
  at += (tmp-at)*.2;
  agx += (gyX-agx)*.2;
  agy += (gyY-agy)*.2;
  agz += (gyZ-agz)*.2;
  
  Wire.endTransmission(true);
}

void printMPU() {
  Serial.print(aax);
  Serial.print("\t"); Serial.print(aay);
  Serial.print("\t"); Serial.print(aaz);
  Serial.print("\t"); Serial.print(at);
  Serial.print("\t"); Serial.print(agx);
  Serial.print("\t"); Serial.print(agy);
  Serial.print("\t"); Serial.println(agz);
}

void set(int *angles) {
  for (int i = 0; i < 8; i++) {
    int a = SERVOMID + lim(angles[i]);
    pwm.setPWM(i, 0, a);
  }
}

int lim(int angle) {
  // takes angle [0, 299]
  // returns angle [-150,150]
  angle -= 150;
  if (angle > 150) {
    return 150;
  } else if (angle < -150) {
    return -150;
  }
  
  return angle;
}
