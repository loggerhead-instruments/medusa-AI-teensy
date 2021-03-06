int CompassAddress = 0x0C;  //0x0C internal compass on 9150
int GyroAddress = 0x68;

#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)

#define BIT_I2C_READ        (0x80)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_MST_VDDIO   (0x80)

#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38

int mpuInit(boolean mode)
{
  int ecode;
   if (printDiags) Serial.print("MPU Init\n");
   if(mode==0)
  {
     ecode = I2Cwrite(GyroAddress, 0x6B, 0x40);  //Sleep mode, internal 8 MHz oscillator  //another mode is cycle where it wakes up periodically to take a value
   // ecode = I2Cwrite(GyroAddress, 0x6B, 0x48);  //Sleep mode, power down PTAT voltage generator, internal oscillator  //higher sleep power draw than using 0x40
     return ecode;
  }

    //set clock source
    ecode = I2Cwrite(GyroAddress, 0x6B, 0x01);  //everything awake; clock from X gyro reference
  
    // configure frame sync and LPF
    I2Cwrite(GyroAddress, 0x1A, 0x03);  //no frame sync; Gyro to sample at 1 kHz with DLPF 41 Hz (4.8 ms delay)
    
    // set gyro range
    I2Cwrite(GyroAddress, 0x1B, 0x10);  // 0x10 +/- 1000 deg/s ; 0x18 +/-2000 deg/s Fchoice_b = 00 (use DLPF)
    
    // set sample rate divider
    I2Cwrite(GyroAddress, 0x19, 9);  //  0x31=49=>20Hz; 1kHz/(1+4)=200; divide 1 kHz/(1+9)=100 Hz sample rate for all sensors

        // set accel range
    I2Cwrite(GyroAddress, 0x1C, 0x18); // 0x18 =  +/- 16 g  DEFAULT
    if (accel_scale == 2) I2Cwrite(GyroAddress, 0x1C, 0x00); // 2g
    if (accel_scale == 4) I2Cwrite(GyroAddress, 0x1C, 0x08); // 4g
    if (accel_scale == 8) I2Cwrite(GyroAddress, 0x1C, 0x10); // 8g
  
    // Accelerometer Configuration 2
    I2Cwrite(GyroAddress, 0x1D, 0x03); // low pass filter at 41 Hz (11.8 ms delay)

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the Arduino as master.
     I2Cwrite(GyroAddress, INT_PIN_CFG, 0x22);
    // Enable data ready (bit 0) interrupt
     I2Cwrite(GyroAddress, INT_ENABLE, 0x01);

    // setup compass
    setup_compass();

    // using FIFO mode to automatically move magnetometer readings
    I2Cwrite(GyroAddress, 0x6A, 0x07); // reset FIFO
    I2Cwrite(GyroAddress, 0x6A, 0x60); // FIFO enabled, Master Mode enabled

   return ecode;
}

byte I2Cwrite(byte addr, byte reg, byte val)
{
  Wire.beginTransmission(addr);  
  Wire.write(reg);  // gyro scale, sample rate and LPF
  Wire.write(val);  
  byte ecode=Wire.endTransmission(); //end transmission
  if (printDiags == 2) Serial.print(ecode);
  delay(5);
  return ecode;
}

void readImu(){
  readBytes(GyroAddress, 0x3B, 20, imuTempBuffer);
}

 /* This initialization is similar to the one in ak8975.c. */
int setup_compass(void)
{
   byte data;
   /* Set up master mode, master clock, and ES bit. */
    data = 0x40;
    if (I2Cwrite(GyroAddress, 0x24, data))
        return -6;

    /* Slave 0 reads from AKM data registers. */
    data = BIT_I2C_READ | CompassAddress;
    if (I2Cwrite(GyroAddress, 0x25, data))
        return -7;

    /* Compass reads start at this register. */
    data = AKM_REG_HXL;
    if (I2Cwrite(GyroAddress, 0x26, data))
        return -8;

    /* Enable slave 0, 6-byte reads. */
    data = BIT_SLAVE_EN  | BIT_SLAVE_GROUP | BIT_SLAVE_BYTE_SW | 6;
    //data= BIT_SLAVE_EN | 8;
    if (I2Cwrite(GyroAddress, 0x27, data))
        return -9;

    /* Slave 1 changes AKM measurement mode. */
    data = CompassAddress;
    if (I2Cwrite(GyroAddress, 0x28, data))
        return -10;

    /* AKM measurement mode register. */
    data = AKM_REG_CNTL;
    if (I2Cwrite(GyroAddress, 0x29, data))
        return -11;

    /* Enable slave 1, 1-byte writes. */
    data = BIT_SLAVE_EN | 1;
    if (I2Cwrite(GyroAddress, 0x2A, data))
        return -12;

    /* Set slave 1 data. */
    data = AKM_SINGLE_MEASUREMENT;
    if (I2Cwrite(GyroAddress, 0x64, data))
        return -13;

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data = 0x03;
    if (I2Cwrite(GyroAddress, 0x67, data))
        return -14;

    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
    data = BIT_I2C_MST_VDDIO;
    if (I2Cwrite(GyroAddress, 0x01, data))
        return -15;
        
    return 0;
}

void calcImu(){
  accel_x = (int16_t) (((int16_t)imuTempBuffer[0] << 8) | imuTempBuffer[1]);    
  accel_y = (int16_t) (((int16_t)imuTempBuffer[2] << 8) | imuTempBuffer[3]);   
  accel_z = (int16_t) -(((int16_t)imuTempBuffer[4] << 8) | imuTempBuffer[5]);    

  gyro_temp = (int16_t) (((int16_t)imuTempBuffer[6]) << 8 | imuTempBuffer[7]);   
 
  gyro_x = (int16_t) (((int16_t)imuTempBuffer[8] << 8) | imuTempBuffer[9]);  
  gyro_y = (int16_t) (((int16_t)imuTempBuffer[10] << 8) | imuTempBuffer[11]);   
  gyro_z = (int16_t) -(((int16_t)imuTempBuffer[12] << 8) | imuTempBuffer[13]);  
  
  mag_y = (int16_t)  (((int16_t)imuTempBuffer[14] << 8) | imuTempBuffer[15]);   
  mag_x = (int16_t)  (((int16_t)imuTempBuffer[16] << 8) | imuTempBuffer[17]);     
  mag_z = (int16_t) (((int16_t)imuTempBuffer[18] << 8) | imuTempBuffer[19]); 

// NED orientation

//  gyro_x = imu.gx;
//  gyro_y = imu.gy;
//  gyro_z = -imu.gz;
//  accel_x = imu.ax;
//  accel_y = imu.ay;
//  accel_z = -imu.az;
//  mag_x = imu.my - magYoffset;
//  mag_y = imu.mx - magXoffset;
//  mag_z = imu.mz - magZoffset;
  
}
