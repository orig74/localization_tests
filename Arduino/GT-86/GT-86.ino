#include "Wire.h"
//#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include <MS5611.h>

MS5611 ms5611;


uint32_t time;

MPU6050 accelgyro;
HMC5883L mag;


typedef struct {
  uint16_t header;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
  float absoluteAltitude;
  uint32_t t_stemp;
  uint16_t footer; 
} data_struct;

data_struct ds;

void chksum()
{
  uint16_t sum=0;
  ds.header=0xa5a5;
  ds.t_stemp=millis();
  for(int i=1;i<(sizeof(ds)/2-1);i++)
  {
    uint16_t* pds=(uint16_t*)&ds;
    sum+=pds[i];
  }
  ds.footer=sum;
}

#define LED_PIN 17
bool blinkState = false;
int iters=0;

void setup() {
  
   Wire.begin();
   accelgyro.setI2CMasterModeEnabled(false);
   accelgyro.setI2CBypassEnabled(true) ;
   accelgyro.setSleepEnabled(false);

   Serial.begin(115200);

   // initialize device
   Serial.println("Initializing I2C devices...");
   accelgyro.initialize();
   mag.initialize();
   Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

   // verify connection
   Serial.println("Testing device connections...");
   Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

   ms5611.setOversampling(MS5611_ULTRA_HIGH_RES);
   ms5611.begin();

   // configure Arduino LED for
   pinMode(LED_PIN, OUTPUT);
}


void loop() {
   
   accelgyro.getMotion6(&ds.ax, &ds.ay, &ds.az, &ds.gx, &ds.gy, &ds.gz);
   mag.getHeading(&ds.mx, &ds.my, &ds.mz);
     // Read raw values
   //uint32_t rawTemp = ms5611.readRawTemperature();
   //uint32_t rawPressure = ms5611.readRawPressure();

   // Read true temperature & Pressure
   //double realTemperature = ms5611.readTemperature();
   int32_t realPressure = ms5611.readPressure();

   // Calculate altitude
   ds.absoluteAltitude = ms5611.getAltitude(realPressure);
   //float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);


   // display tab-separated accel/gyro x/y/z values
#if 0
   Serial.print("a/g:\t");
   Serial.print(ax); Serial.print("\t");
   Serial.print(ay); Serial.print("\t");
   Serial.print(az); Serial.print("\t");
   Serial.print(gx); Serial.print("\t");
   Serial.print(gy); Serial.print("\t");
   Serial.print(gz);Serial.print("\t");
   
   Serial.print("mag:\t");
   Serial.print(mx); Serial.print("\t");
   Serial.print(my); Serial.print("\t");
   Serial.print(mz); Serial.print("\t");

// To calculate heading in degrees. 0 degree indicates North
   float heading = atan2(my, mx);
   if(heading < 0)
     heading += 2 * M_PI;
   Serial.print("heading:\t");
   Serial.print(heading * 180/M_PI);Serial.print("\t");

   Serial.print("alt:\t");
   Serial.println( absoluteAltitude );
#else

   chksum();
   Serial.write((const uint8_t*)&ds,sizeof(ds));
#endif
   // blink LED to indicate activity
   iters+=1;
   if(iters%10==0){
       blinkState = !blinkState;
       digitalWrite(LED_PIN, blinkState);
   }
   delay(100); 
}
