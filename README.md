# Rolling Stone Illusion
This project is just a demo for the rolling stone illusion. 

## Hardware
![Interface Image](pics/stick1.png)

<img src="pics/stick2.png" width="600" >

This is the hardware requiered to build a rolling stone stick
- Stick: Any drum stick or aluminium stick is OK
- Microcontroller: Arduino UNO or MEGA is OK. 
- Accelerometer: MPU 6050 (https://www.amazon.co.jp/-/en/MPU-6050-3-Axis-Gyroscope-Acceleration-Sensor/dp/B008BOPN40)
- Audio Amplifier: Hapbeat Amplifier. (but any audio amplifier is OK) 
- Vibrator, serveral actuators can be used:
    - Hapbeat motor (not good)
    - ALPS transducers (available in the lab) GOOD
    - Haptuator (also available at the lab) NOT TESTED YET

### Connection PINs (Arduino pro mini)

<img src="pics/ard-mega-pro-pinout.jpg" width="600" >

**Arduino Mini Pro pinout**

Arduino PIN | Conection
--- | ---
SCL (pin D21) |  Accelerometer SCL pin
SDA (pin D20) |  Accelerometer SDA pin
GND  |  Accelerometer GND
5V  |  Accelerometer 5V
Audio PWM (pin D11)  |  Hapbeat audio IN (L/R)
GND  |  Hapbeat audio GND


## Software
To install some Arduino libraries are needed to make the software run: 
- Mozzi (https://sensorium.github.io/Mozzi/)
- MPU 6050 non block IC2 libraries (included in the project)
- MPU Arduino libraries, for the accelerometer calibration (https://github.com/jrowberg/i2cdevlib)

Note. Since audio synthesis needs to run as fast as possible, it is not possible to use the usual MPU 6050 Arduino libraries. Non block libraries to read the acceleromter data are needed for the code run smoothly. 

### Accelerometer calibration 
It is mandatory to calibrate the MPU 6050 sensor in order to get accurate tilt angle readings. Otherwise the reading will drift or be unacurate. 

The calibration values for MPU 6050 ARE UNIQUE FOR EVERY SENSOR. The calibration values must be extracted using a script included in the MPU6050 Arduino libraries (https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/IMU_Zero/IMU_Zero.ino).

When running the calibration script, the accelerometer must be placed in a flat surface and it should not be moved. 

The calibration values must the introduced in `accel_control.h` in the following constants:
````
#define ACC_X_OFFSET  -1840
#define ACC_Y_OFFSET  -2186
#define ACC_Z_OFFSET  1199
````

