
#include <twi_nonblock.h>

static volatile byte acc_status = 0;
#define ACC_IDLE 0
#define ACC_READING 1
#define ACC_WRITING 2

int accbytedata[14];

#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b
#define MPU6050_ACCEL_OUT_REGISTER    0x3b
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0
// Sensor calibration offset values
//-1840  -2186 1199  89  39  29  (my accelerometer values)
#define ACC_X_OFFSET  -1840
#define ACC_Y_OFFSET  -2186
#define ACC_Z_OFFSET  1199

int minVal=265;
int maxVal=402;
int accx_i; int accy_i; int accz_i;
float accx; float accy; float accz;
int xAng; int yAng; int zAng;
double tilt_x; double tilt_y; double tilt_z;

// Writes val to address register on device
void acc_writeTo(byte address, byte val) {
  twowire_beginTransmission(MPU6050_ADDR); // start transmission to device
  twowire_send( address );
  twowire_send( val );
  twowire_endTransmission();
}

// Initial setup of the accelerometer 
void setup_accelero(){
  initialize_twi_nonblock();

  acc_writeTo(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  acc_writeTo(MPU6050_CONFIG_REGISTER, 0x00);
  acc_writeTo(MPU6050_GYRO_CONFIG_REGISTER, 0x00); //0x08
  acc_writeTo(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
  acc_writeTo(MPU6050_PWR_MGMT_1_REGISTER, 0x01);
  
  acc_status = ACC_IDLE;
}

/// ---------- non-blocking version ----------
void initiate_read_accelero(){
  // Reads num bytes starting from address register on device in to _buff array
  // set address of targeted slave
  txAddress = MPU6050_ADDR; //MMA7660_ADDR;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  // put byte in tx buffer
  txBuffer[txBufferIndex] = MPU6050_ACCEL_OUT_REGISTER; // Start reading from register for X
  ++txBufferIndex;
  // update amount in buffer
  txBufferLength = txBufferIndex;

  twi_initiateWriteTo(txAddress, txBuffer, txBufferLength);
  acc_status = ACC_WRITING;
}

void initiate_request_accelero(){
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  byte read = twi_initiateReadFrom(MPU6050_ADDR, 14); // 14 is the number of bytes to read
  acc_status = ACC_READING;
}

void finalise_request_accelero() {
  byte read = twi_readMasterBuffer( rxBuffer, 14 ); 
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  byte i = 0;
  while( rxBufferLength - rxBufferIndex > 0) { // device may send less than requested (abnormal)
    accbytedata[i] = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
    i++;
  }

  acc_status = ACC_IDLE;
}
/// ----end------ non-blocking version ----------

// Main loop that gets the acc raw data and computes the tild angles
unsigned long ms = millis();
unsigned long readTime = ms;
void getAccInfo(){
  //ms = millis();
  //if (ms > readTime){
  //  readTime += 20;

    switch( acc_status ){
    case ACC_IDLE:
      accx_i = (accbytedata[0] << 8 | accbytedata[1]); // accelerometer x reading, reduced to 8 bit
      accx = (float)(accx_i + ACC_X_OFFSET) / 16384.0;
      accy_i = (accbytedata[2] << 8 | accbytedata[3]); // accelerometer y reading, 8 bit
      accy = (float)(accy_i + ACC_Y_OFFSET) / 16384.0;
      accz_i = (accbytedata[4] << 8 | accbytedata[5]); // accelerometer z reading
      accz = (float)(accz_i + ACC_Z_OFFSET) / 16384.0;
      
    //  Serial.print("aX ");Serial.print(accx);
    //  Serial.print("\taY ");Serial.print(accy);
    //  Serial.print("\taZ ");Serial.print(accz);
    //  Serial.println();

      // Method from the tilt arduino sample: https://how2electronics.com/measure-tilt-angle-mpu6050-arduino/#:~:text=The%20MPU6050%20consist%20of%203,shown%20in%20the%20below%20figure.
      // ONLY TILT ANGLE IN X IS NEEDED FOR THE ROLLING STONE DEMO
      xAng = map(accx_i,minVal,maxVal,-90,90);
      yAng = map(accy_i,minVal,maxVal,-90,90);
      zAng = map(accz_i,minVal,maxVal,-90,90);

//      tilt_x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
//      tilt_y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
//      tilt_z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

      tilt_x= (atan2(-yAng, -zAng)+PI);
      tilt_y= (atan2(-xAng, -zAng)+PI);
      tilt_z= (atan2(-yAng, -xAng)+PI);
      
//      Serial.print("x Angle ");Serial.print(tilt_x);
//      Serial.print("\ty Angle ");Serial.print(tilt_y);
//      Serial.print("\tz Angle ");Serial.print(tilt_z);
//      Serial.println();
      
      initiate_read_accelero();      
      break;
    case ACC_WRITING:
      if ( TWI_MTX != twi_state ){
        initiate_request_accelero();
      }
      break;
    case ACC_READING:
      if ( TWI_MRX != twi_state ){
        finalise_request_accelero();
      }
      break;
    }
  //}
}
