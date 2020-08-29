/* ==================================================  */
/*                   INCLUDES                          */
/* ==================================================  */
#include <Wire.h>
#include <Math.h>

/* ==================================================  */
/*                     MACROS                          */
/* ==================================================  */

#define WHO_AM_I  (0x75)        // Register 117 – Who Am I
#define DEFAULT_WHO_AM_I (0x68) // Default value of register Who am I
#define MPU_I2C_ADD (0x68)      // The I2C address of MPU6050.
#define PWR_MGMT_1 (0x6B)           //Register 107 – Power Management 1


#define GYRO_XOUT_H (0x43)      // Registers 67 to 72 – Gyroscope Measurements
#define GYRO_CONFIG (0x1B)      // Register 27 – Gyroscope Configuration
#define ACCEL_CONFIG (0x1C)     // Register 28 – Accelerometer Configuration
#define ACCEL_XOUT_H (0x3B)     // Registers 59 to 64 – Accelerometer Measurements

#define G_TO_M_S2 (9.80665)     // Multipy accelerometer g values with this constant to convert it to m/s2 unit.

/* ==================================================  */
/*                   DATA TYPES                        */
/* ==================================================  */

// This enum lists the full scale range options for gyroscope.
typedef enum gFsSel
{
  G_FS_RANGE_250  = 0b00,
  G_FS_RANGE_500  = 0b01,
  G_FS_RANGE_1000 = 0b10,
  G_FS_RANGE_2000 = 0b11
} gFsSel_e;

// This enum lists the full scale range options for accelerometer.
typedef enum aFsSel
{
  A_FS_RANGE_2  = 0b00,
  A_FS_RANGE_4  = 0b01,
  A_FS_RANGE_8  = 0b10,
  A_FS_RANGE_16 = 0b11
} aFsSel_e;

typedef struct gyroSt
{
  float x;
  float y;
  float z;
} gyroAcc_s;

typedef struct SensFact
{
  float gSensFact;  // This variable stores the LSB sensitivity of gyroscope based on the full scale range.
  float aSensFact;  // This variable stores the LSB sensitivity of accelerometer based on the full scale range.
} sensFact_s;

/* ==================================================  */
/*                   GLOBAL VARIABLES                  */
/* ==================================================  */

sensFact_s gyroAccSensFact;
unsigned long currTime, prevTime;
float elapsedTime;
gyroAcc_s gyroValues, gError, accValues, aError;
float pitch, roll, yaw, finalPitch, finalRoll, finalYaw;

/* ==================================================  */
/*                FUNCTION DEFINITIONS                 */
/* ==================================================  */

gyroAcc_s readRawGyroValues()
{
  gyroAcc_s gValues;

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.requestFrom(MPU_I2C_ADD, 6); // 3 axis, 2 byte for each axis.

  while (Wire.available() < 6);

  // Read the raw values
  gValues.x = Wire.read() << 8 | Wire.read();
  gValues.y = Wire.read() << 8 | Wire.read();
  gValues.z = Wire.read() << 8 | Wire.read();

  return gValues;
}

/* This function reads the raw values of gyroscope */
gyroAcc_s readGyroValues()
{
  gyroAcc_s gValues;

  gValues = readRawGyroValues();

  // Divide the raw values with the sensitivity factor to get the actual values in dps. Then subtract it by the error offset.
  gValues.x = gValues.x / gyroAccSensFact.gSensFact - gError.x;
  gValues.y = gValues.y / gyroAccSensFact.gSensFact - gError.y;
  gValues.z = gValues.z / gyroAccSensFact.gSensFact - gError.z;

  return gValues;

}

/* This function reads the raw values of accelerometer */
gyroAcc_s readRawAccValues()
{
  gyroAcc_s aValues;

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.requestFrom(MPU_I2C_ADD, 6); // 3 axis, 2 byte for each axis.

  while (Wire.available() < 6);

  // Read the raw values
  aValues.x = Wire.read() << 8 | Wire.read();
  aValues.y = Wire.read() << 8 | Wire.read();
  aValues.z = Wire.read() << 8 | Wire.read();

  return aValues;
}
gyroAcc_s readAccValues()
{
  gyroAcc_s aValues;

  aValues = readRawAccValues();

  // Divide the raw values with the sensitivity factor to get the actual values in dps.
  aValues.x = aValues.x / gyroAccSensFact.aSensFact - aError.x;
  aValues.y = aValues.y / gyroAccSensFact.aSensFact - aError.y;
  aValues.z = aValues.z / gyroAccSensFact.aSensFact - aError.z;

  return aValues;
}

float setGFsSel(gFsSel_e gFsVal)
{
  uint8_t regVal = 0b00000111;
  float gSensFact = 1.0;

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.write(GYRO_CONFIG);
  regVal |= gFsVal <<  3;
  Wire.write(regVal);
  Wire.endTransmission();

  switch (gFsVal)
  {
    // Set the LSB sensitivity for gyroscope. See table in 4.19 Registers 67 to 72 – Gyroscope Measurements
    case G_FS_RANGE_250:
      gSensFact = (float)131.0;
      break;

    case G_FS_RANGE_500:
      gSensFact = (float) 65.5;
      break;

    case G_FS_RANGE_1000:
      gSensFact = (float) 32.8;
      break;

    case G_FS_RANGE_2000:
      gSensFact = (float) 16.4;
      break;

    default:
      break;
  }
  gyroAccSensFact.gSensFact = gSensFact;
}

void setAFsSel(aFsSel_e aFsVal)
{
  uint8_t regVal = 0b00000111;
  float aSensFact = 1;

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.write(ACCEL_CONFIG);
  regVal |= aFsVal <<  3;
  Wire.write(regVal);
  Wire.endTransmission();

  switch (aFsVal)
  {
    case A_FS_RANGE_2:
      aSensFact = (float) 16384.0;
      break;

    case A_FS_RANGE_4:
      aSensFact = (float) 8192.0;
      break;


    case A_FS_RANGE_8:
      aSensFact = (float) 4096.0;
      break;

    case A_FS_RANGE_16:
      aSensFact = (float) 2048.0;
      break;

    default:
      break;
  }
  gyroAccSensFact.aSensFact = aSensFact;
}

uint8_t ReadOneRegister(uint32_t regAdd)
{
  uint8_t value;

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.write(regAdd);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_I2C_ADD);
  Wire.requestFrom(MPU_I2C_ADD, 1);
  value = Wire.read();
  Wire.endTransmission();
  
  return value;
}

void calcGyroAccErr(const uint16_t sampleCnt)
{
  uint16_t sampleIte;
  gyroAcc_s gValues, sumGvalues, aValues, sumAvalues;

  sumGvalues.x = 0;
  sumGvalues.y = 0;
  sumGvalues.z = 0;

  sumAvalues.x = 0;
  sumAvalues.y = 0;
  sumAvalues.z = 0;

  for (sampleIte = 0; sampleIte < sampleCnt; sampleIte++)
  {
    gValues = readRawGyroValues();
    aValues = readRawAccValues();

    sumGvalues.x += gValues.x / gyroAccSensFact.gSensFact;
    sumGvalues.y += gValues.y / gyroAccSensFact.gSensFact;
    sumGvalues.z += gValues.z / gyroAccSensFact.gSensFact;

    sumAvalues.x += aValues.x / gyroAccSensFact.aSensFact;
    sumAvalues.y += aValues.y / gyroAccSensFact.aSensFact;
    sumAvalues.z += (aValues.z / gyroAccSensFact.aSensFact - 1); // Z axis value for accelerometer is 1 when IMU is kept stationary on ground.

    //  Serial.print("X: "); Serial.print(gValues.x/gyroAccSensFact.gSensFact); Serial.print("\tY: "); Serial.print(gValues.y/gyroAccSensFact.gSensFact);  Serial.print("\tZ: "); Serial.print(gValues.z/gyroAccSensFact.gSensFact);
    //  Serial.print("\t\t sumGvalues: X: "); Serial.print(sumGvalues.x); Serial.print("\t Y: "); Serial.print(sumGvalues.y); Serial.print("\t Z: "); Serial.print(sumGvalues.z);
    //  Serial.print("\t\t sumAvalues: X: "); Serial.print(sumAvalues.x); Serial.print("\t Y: "); Serial.print(sumAvalues.y); Serial.print("\t Z: "); Serial.println(sumAvalues.z);
  }


  gError.x = sumGvalues.x / sampleCnt;
  gError.y = sumGvalues.y / sampleCnt;
  gError.z = sumGvalues.z / sampleCnt;

  aError.x = sumAvalues.x / sampleCnt;
  aError.y = sumAvalues.y / sampleCnt;
  aError.z = sumAvalues.z / sampleCnt;
}

void setup()
{
  Serial.begin(115200);

  if (ReadOneRegister(WHO_AM_I) != DEFAULT_WHO_AM_I)
  {
    Serial.println("Detection of MPU6050 failed! Re-check your connections!");
  }
  else
  {
    Serial.println("Who am I passed. Detected MPU6050 successfully.");
  }

  Wire.beginTransmission(MPU_I2C_ADD); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // set to zero to wake up the MPU-6050
  Wire.endTransmission(true);

  // Set the Gyroscope full scale range
  setGFsSel(G_FS_RANGE_250);
  
  // Set the Accelerometer full scale range
  setAFsSel(A_FS_RANGE_2);

  // Calculate the error in measurements for gyroscope and accelerometer
  calcGyroAccErr(300);
  
  Serial.print("Gyro Errors - X: "); Serial.print(gError.x); Serial.print("\tY: "); Serial.print(gError.y); Serial.print("\tZ: "); Serial.println(gError.z);
  Serial.print("Acc  Errors - X: "); Serial.print(aError.x); Serial.print("\tY: "); Serial.print(aError.y); Serial.print("\tZ: "); Serial.println(aError.z);
}

void loop() {

  gyroAcc_s gyroAngVel; // This variable stores the realistic gyroscope readings after error correction. Unit: degrees/sec.
  gyroAcc_s accAngVel;  // This variable stores the realistic accelerometer readings after error correction. Unit: g.
  gyroAcc_s accValuesUnFil;
  
  float alpha = 0.5;    // Smoothing factor for complementary filter

  currTime = millis();
  prevTime = currTime;
  
  while (1)
  {
    prevTime = currTime;
    currTime = millis();
    gyroAngVel = readGyroValues();
    accAngVel = readAccValues();

    // Convert g to m/s2
    accValuesUnFil.x = accAngVel.x * G_TO_M_S2;
    accValuesUnFil.y = accAngVel.y * G_TO_M_S2;
    accValuesUnFil.z = accAngVel.z * G_TO_M_S2;

    // Complimentary filter formula: Yt = alpha * Xt + (1 - alpha) * Yt-1
    // Where yt is our filtered signal, Yt-1 the previous filtered signal, Xt the accelerometer reading and alpha the smoothing factor.
    
    accValues.x = alpha * accValuesUnFil.x + ((1 - alpha) * accValues.x);
    accValues.y = alpha * accValuesUnFil.y + ((1 - alpha) * accValues.y);
    accValues.z = alpha * accValuesUnFil.z + ((1 - alpha) * accValues.z);

    // Convert the unit of values from g to degrees using atan2
    pitch = (atan2(-1 * accValues.x, sqrt(accValues.y * accValues.y + accValues.z * accValues.z)) * 180) / M_PI;
    roll = (atan2(accValues.y, accValues.z) * 180) / M_PI;

    elapsedTime = 0.01; // seconds
    //elapsedTime = currTime - prevTime;
    if (elapsedTime > 0)
    {
      gyroValues.x = (gyroValues.x + (gyroAngVel.x * elapsedTime)) * 0.95 + (roll * 0.05);
      gyroValues.y = (gyroValues.y + (gyroAngVel.y * elapsedTime)) * 0.95 + (pitch * 0.05);
      gyroValues.z = (gyroValues.z + (gyroAngVel.z * elapsedTime));
    }

    finalPitch = gyroValues.y; //(0.95 * gyroValues.y) + (0.05 * pitch);
    finalRoll =  gyroValues.x; //(0.95 * gyroValues.x) + (0.05 * roll);
    
    // Gyro X = Roll, Y = Pitch, Z = Yaw
    //Serial.print("XGReal: "); Serial.print(gyroValues.x); Serial.print("\tYGReal: "); Serial.print(gyroValues.y); Serial.print("\tZGReal: "); Serial.print(gyroValues.z);   //250 dps
    //Serial.print("\tXAReal: "); Serial.print((float)accValues.x); Serial.print("\tYAReal: "); Serial.print(accValues.y); Serial.print("\tZAReal: "); Serial.print(accValues.z); Serial.print("\t");   //8g
    Serial.print("Pitch: "); Serial.print(finalPitch); Serial.print("\tRoll: "); Serial.println(finalRoll);

    delay(10 - (millis() - currTime));
  }
}
