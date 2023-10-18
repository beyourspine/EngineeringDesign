#include <Wire.h>
#include <ArduinoBLE.h>
#include <ESP32Time.h>
#include <Preferences.h>


typedef struct sensorData {
  float roll;
  float pitch;
  float yaw;
};

typedef struct sensorTime {
  sensorData* sensor[4];
  long int time;
};

typedef struct sensorError {
  float accErrorX;
  float accErrorY;
  float gyroErrorX;
  float gyroErrorY;
  float gyroErrorZ;
};

BLEService sensorService("c47cace0-6c57-11ee-b962-0242ac120002");
BLEIntCharacteristic sensorBLEData("c47caf88-6c57-11ee-b962-0242ac120002", BLERead | BLENotify);


ESP32Time rtc(0);



const int MPU = 0x68;
int ledPin = D9, sensorCount = 4;
int sensors[4] = {D2, D3, D5, D6};
int errorAddress[4] = {0, 20, 41, 62};
sensorTime sensorArray[10] = {};
sensorError sensorErrorArray[4] = {};
float oldTime;
float accX, accY, accZ, gyroX, gyroY, gyroZ, elapsedTime, currentTime, previousTime, accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ, yaw;

void setup() 
{
  Serial.begin(19200);
  Wire.begin();
  ESP32Time rtc(0);
  resetSensors();
  
  if (false)
  {
    Serial.println("Warning: Sensors are uncalibrated, calibration will run in 5 seconds");
    
    calculateError();
  }
  else
  {
    retrieveErrorData();
  }

  if (!BLE.begin()) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    while (1);
  }
  BLE.setLocalName("PostureMonitor");
  BLE.setDeviceName("ESP32-E");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorBLEData);
  BLE.addService(sensorService);
  sensorBLEData.writeValue(1);

  BLE.advertise();

  pinMode(ledPin, OUTPUT);
  pinMode(sensors[0], OUTPUT);
  pinMode(sensors[1], OUTPUT);
  pinMode(sensors[2], OUTPUT);
  pinMode(sensors[3], OUTPUT);

  digitalWrite(ledPin, HIGH);

  digitalWrite(sensors[0], HIGH);
  digitalWrite(sensors[1], HIGH);
  digitalWrite(sensors[2], HIGH);
  digitalWrite(sensors[3], HIGH);

}

void loop() 
{
  
  BLEDevice central = BLE.central();

  sensorBLEData.writeValue(1);
 
  Serial.println(sensorArray[0].time);
  Serial.print("|");
  Serial.print(sensorArray[0].sensor[0]->pitch);
  Serial.print("|");
  Serial.print(sensorArray[0].sensor[0]->roll);
  Serial.print("|");
  Serial.print(sensorArray[0].sensor[0]->yaw);
  Serial.println("");
  

  delay(1000);
  if (central)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }

  
  //updateSensorData(oldTime);
   sensorBLEData.writeValue(0);
  freeMem(sensorArray);
}

sensorTime getAllData() 
{
  
  sensorTime allData;
  
  allData.time = rtc.getEpoch();

  for (int i = 0; i < sensorCount; i++)
  {
    digitalWrite(sensors[i], LOW);
    allData.sensor[i] = (sensorData *) malloc(sizeof(sensorData));

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    accX = (Wire.read() << 8|Wire.read()) / 16384.0;
    accY = (Wire.read() << 8|Wire.read()) / 16384.0;
    accZ = (Wire.read() << 8|Wire.read()) / 16384.0;

    accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) + sensorErrorArray[i].accErrorX; 
    accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) + sensorErrorArray[i].accErrorY;

    previousTime = currentTime; 
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000;

    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    gyroX = (Wire.read() << 8|Wire.read()) / 131.0;
    gyroY = (Wire.read() << 8|Wire.read()) / 131.0;
    gyroZ = (Wire.read() << 8|Wire.read()) / 131.0;

    gyroX = gyroX + sensorErrorArray[i].gyroErrorX;
    gyroY = gyroY + sensorErrorArray[i].gyroErrorY;
    gyroZ = gyroZ + sensorErrorArray[i].gyroErrorZ;

    gyroAngleX = gyroAngleX + gyroX * elapsedTime;
    gyroAngleY = gyroAngleY + gyroY * elapsedTime;
    yaw = yaw + gyroZ * elapsedTime;
    
    gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
    gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;
   

    allData.sensor[i]->roll = gyroAngleX;
    allData.sensor[i]->pitch = gyroAngleY; 
    allData.sensor[i]->yaw = yaw;

    digitalWrite(sensors[i], HIGH);
  }

  return allData;
}

sensorData* getData(int sensorNr)
{
  sensorData* data;

  return data;
}

float updateSensorData(float oldTime)
{
  float time = millis();
  if ((time - oldTime) / 1000 > 10)
  {
    sensorBLEData.writeValue(0);
    oldTime = currentTime;
  }
  return oldTime;
}

void resetSensors()
{
  for (int i = 0; i < sensorCount; i++)
  {
    digitalWrite(sensors[i], LOW);
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
  }
}

void freeMem(sensorTime array[])
{
  for (int i = 0; i < 10; i++)
  {
    if (array[i].sensor[0] != NULL)
    {
      for (int j = 0; j < 4; j++)
      {
        free(array[i].sensor[j]);
      }
    }
  }
}

void retrieveErrorData()
{
  for (int i = 0; i < 4; i++)
  {
    //sensorErrorArray[i] = Preferences.;
  }
}


void calculateError()
{
  for (int i = 0; i < sensorCount; i++)
  {
    int cycles = 0;

    while (cycles < 200)
    {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6);

      accX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      accY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      accZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

      sensorErrorArray[i].accErrorX = sensorErrorArray[i].accErrorX + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
      sensorErrorArray[i].accErrorY = sensorErrorArray[i].accErrorY + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
      cycles++;
    }

    sensorErrorArray[i].accErrorX = sensorErrorArray[i].accErrorX / 200;
    sensorErrorArray[i].accErrorY = sensorErrorArray[i].accErrorY / 200;
    cycles = 0;

    while (cycles < 200)
    {
      Wire.beginTransmission(MPU);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6);

      gyroX = Wire.read() << 8 | Wire.read();
      gyroY = Wire.read() << 8 | Wire.read();
      gyroZ = Wire.read() << 8 | Wire.read();
    
      sensorErrorArray[i].gyroErrorX = sensorErrorArray[i].gyroErrorX + (gyroX / 131.0);
      sensorErrorArray[i].gyroErrorY = sensorErrorArray[i].gyroErrorY + (gyroY / 131.0);
      sensorErrorArray[i].gyroErrorZ = sensorErrorArray[i].gyroErrorZ + (gyroZ / 131.0);
      cycles++;
    }

    sensorErrorArray[i].gyroErrorX = sensorErrorArray[i].gyroErrorX / 200;
    sensorErrorArray[i].gyroErrorX = sensorErrorArray[i].gyroErrorX / 200;
    sensorErrorArray[i].gyroErrorX = sensorErrorArray[i].gyroErrorX / 200;

    
  }

  //Preferences.putBytes()

}
