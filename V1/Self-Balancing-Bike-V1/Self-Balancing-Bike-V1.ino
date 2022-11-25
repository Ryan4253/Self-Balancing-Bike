#include <analogWrite.h>
#include "DCMotor.h"
#include "EZServo.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" 
#include "PID.h"

int motor1pin1 = 14;
int motor1pin2 = 27;
int ENABLE = 26; 

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

double targetAngle = 0;
const double ANGLERATE = 0.1;

double input;

DCMotor motor(motor1pin1, motor1pin2, ENABLE, true);
//EZServo servo(19, 65, 100);
PID balancePID(8, 20, 10); // 25 35 30 // 10 10 20 // 8 10 15
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; 
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z] // ＝
VectorFloat gravity;    // [x, y, z] //
float ypr[3]; 

volatile bool mpuInterrupt = false;     // MPU中断

void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  mpu.setXGyroOffset(115);
  mpu.setYGyroOffset(38);
  mpu.setZGyroOffset(7);
  mpu.setZAccelOffset(1493); 
  Wire.begin(21 , 22);
  Serial.begin(115200);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

   // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

    
  // supply your own gyro offsets here, scaled for min sensitivity


  if (devStatus == 0)
  {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

      //對應到MPU6050 INIT腳
      pinMode(32, INPUT);
      attachInterrupt(32, dmpDataReady, RISING);

      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize(); 
    }
    else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
    Serial.println("LEAVING SETUP");
}

void loop() {
  //motor.setPower(255);
    //Serial.println("HI");
   // 如果程序失败直接return。停止 
   //Serial.println(balancePID.getIntegral());
   if (!dmpReady) return;
    //Serial.println(fifoCount < packetSize);
   // 等待MPU中斷或額外數據包可用
   while (!mpuInterrupt && fifoCount < packetSize ){
                
        //Print the value of Input and Output on serial monitor to check how it is working.
        
     //Serial.println(input); //Serial.print(" =>"); Serial.println(output);//開始實測時，請註解此行
       
    
   }
     //reset
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    //  獲取當前FIFO計數
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
       // 重置
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // 否則，檢查DMP數據就緒中斷(經常發生)
    }
    
    else if (mpuIntStatus & 0x02)
    {
        //  等待正確的可用數據長度
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // 從FIFO讀取數據包
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // 跟蹤FIFO計數在這裏，以防有多餘1個包可用
    // (可以幫助我們立即閱讀更多內容而無需等待中斷)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //q值
        mpu.dmpGetGravity(&gravity, &q); //重力值
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //ypr值

        input = ypr[2] * 180/M_PI;

        if(input < targetAngle){
          targetAngle += ANGLERATE * 0.01;
        }
        else if(input > targetAngle){
          targetAngle -= ANGLERATE * 0.01;
        }
        //Serial.println(input);
        //Serial.println(balancePID.getIntegral());
        Serial.println(input);
        double power = balancePID.step(targetAngle-input);
        motor.setPower(power);
        delay(10);
    }
   
}
