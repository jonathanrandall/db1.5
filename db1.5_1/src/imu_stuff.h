#ifndef IMU_STUFF_H
#define IMU_STUFF_H

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

TaskHandle_t run_imu;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
void run_imu_loop(void *params);

void start_imu()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(141);
    mpu.setYGyroOffset(107);
    mpu.setZGyroOffset(18);
    mpu.setZAccelOffset(934); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        // Serial.println(F(")..."));
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        xTaskCreatePinnedToCore(
            run_imu_loop, /* Function to implement the task */
            "imu_loop",   /* Name of the task */
            1024,         /* Stack size in words */
            NULL,         /* Task input parameter */
            0,            /* Priority of the task */
            &run_imu,     /* Task handle. */
            1);           /* Core where the task should run */
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
}

void run_imu_loop(void *params)
{
    // if programming failed, don't try to do anything
    for (;;)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);

        while (!dmpReady)
        {
        }
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        { // Get the Latest packet
            // #ifdef OUTPUT_READABLE_QUATERNION
            //             // display quaternion values in easy matrix form: w x y z
            //             mpu.dmpGetQuaternion(&q, fifoBuffer);
            //             Serial.print("quat\t");
            //             Serial.print(q.w);
            //             Serial.print("\t");
            //             Serial.print(q.x);
            //             Serial.print("\t");
            //             Serial.print(q.y);
            //             Serial.print("\t");
            //             Serial.println(q.z);
            // #endif

            // #ifdef OUTPUT_READABLE_EULER
            //             // display Euler angles in degrees
            //             mpu.dmpGetQuaternion(&q, fifoBuffer);
            //             mpu.dmpGetEuler(euler, &q);
            //             Serial.print("euler\t");
            //             Serial.print(euler[0] * 180 / M_PI);
            //             Serial.print("\t");
            //             Serial.print(euler[1] * 180 / M_PI);
            //             Serial.print("\t");
            //             Serial.println(euler[2] * 180 / M_PI);
            // #endif

            // #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);
            // #endif

            // #ifdef OUTPUT_READABLE_REALACCEL
            //             // display real acceleration, adjusted to remove gravity
            //             mpu.dmpGetQuaternion(&q, fifoBuffer);
            //             mpu.dmpGetAccel(&aa, fifoBuffer);
            //             mpu.dmpGetGravity(&gravity, &q);
            //             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            //             Serial.print("areal\t");
            //             Serial.print(aaReal.x);
            //             Serial.print("\t");
            //             Serial.print(aaReal.y);
            //             Serial.print("\t");
            //             Serial.println(aaReal.z);
            // #endif

            // #ifdef OUTPUT_READABLE_WORLDACCEL
            //             // display initial world-frame acceleration, adjusted to remove gravity
            //             // and rotated based on known orientation from quaternion
            //             mpu.dmpGetQuaternion(&q, fifoBuffer);
            //             mpu.dmpGetAccel(&aa, fifoBuffer);
            //             mpu.dmpGetGravity(&gravity, &q);
            //             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            //             mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            //             Serial.print("aworld\t");
            //             Serial.print(aaWorld.x);
            //             Serial.print("\t");
            //             Serial.print(aaWorld.y);
            //             Serial.print("\t");
            //             Serial.println(aaWorld.z);
            // #endif

            // #ifdef OUTPUT_TEAPOT
            //             // display quaternion values in InvenSense Teapot demo format:
            //             teapotPacket[2] = fifoBuffer[0];
            //             teapotPacket[3] = fifoBuffer[1];
            //             teapotPacket[4] = fifoBuffer[4];
            //             teapotPacket[5] = fifoBuffer[5];
            //             teapotPacket[6] = fifoBuffer[8];
            //             teapotPacket[7] = fifoBuffer[9];
            //             teapotPacket[8] = fifoBuffer[12];
            //             teapotPacket[9] = fifoBuffer[13];
            //             Serial.write(teapotPacket, 14);
            //             teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
            // #endif

            // blink LED to indicate activity
            // blinkState = !blinkState;
            // digitalWrite(LED_PIN, blinkState);
        }
    }
}

#endif

