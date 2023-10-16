#include <Wire.h>
#include <ArduinoBLE.h>
#include <ESP32Time.h>

typedef struct sensorData {
  float roll;
  float pitch;
  float yaw;
};

typedef struct sensorTime {
  sensorData* sensor[4];
  long int time;
};

BLEService sensorService("1101");
BLEUnsignedCharCharacteristic sensorsData("2101", BLERead | BLENotify);


ESP32Time rtc(0);

const int MPU = 0x68;
int ledPin = D9, sensorCount = 4;
int sensors[4] = {D2, D3, D5, D6};
sensorTime sensorArray[10] = {NULL};
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ, oldTime;
float accX, accY, accZ, gyroX, gyroY, gyroZ, elapsedTime, currentTime, previousTime, accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ, yaw;

void setup() 
{
  Serial.begin(19200);
  Wire.begin();
  ESP32Time rtc(0);
  resetSensors();

  if (!BLE.begin()) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    while (1);
  }
  BLE.setLocalName("PostureMonitor");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorsData);
  BLE.addService(sensorService);



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

  sensorArray[0] = getAllData();

  if (central)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
  updateSensorData(oldTime);

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

    accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) + accErrorX; 
    accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) + accErrorY;

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

    gyroX = gyroX + gyroErrorX;
    gyroY = gyroY + gyroErrorY;
    gyroZ = gyroZ + gyroErrorZ;

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

sensorData *getData(int sensorNr)
{
  sensorData* data;

  return data;
}

float updateSensorData(float oldTime)
{
  float time = millis();
  if ((time - oldTime) / 1000 > 10)
  {
    sensorsData.writeValue(0);
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

void calculateError()
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

    accErrorX = accErrorX + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
    accErrorY = accErrorY + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
    cycles++;
  }

  accErrorX = accErrorX / 200;
  accErrorY = accErrorY / 200;
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
  
    gyroErrorX = gyroErrorX + (gyroX / 131.0);
    gyroErrorY = gyroErrorY + (gyroY / 131.0);
    gyroErrorZ = gyroErrorZ + (gyroZ / 131.0);
    cycles++;
  }

  gyroErrorX = gyroErrorX / 200;
  gyroErrorY = gyroErrorY / 200;
  gyroErrorZ = gyroErrorZ / 200;
}
