#include <Wire.h>
#include <ArduinoBLE.h>
#include <ESP32Time.h>
#include <Preferences.h>
Preferences prefs;


typedef struct sensorData {
  float roll;
  float pitch;
};

typedef struct sensorTime {
  sensorData* sensor[4];
  long int time;
};

typedef struct sensorError {
  float accErrorX;
  float accErrorY;
};

BLEService sensorService("c47cace0-6c57-11ee-b962-0242ac120002");
BLEIntCharacteristic sensorBLEData("c47caf88-6c57-11ee-b962-0242ac120002", BLERead | BLENotify);


ESP32Time rtc(0);



const int MPU = 0x68;
int ledPin = D9, sensorCount = 4;
int sensors[4] = { D2, D3, D5, D6 };
int flag = 0;
char* sensorChar[4] = { "D2", "D3", "D5", "D6" };
sensorTime sensorArray[10] = {};
sensorError sensorErrorArray[4] = {};
float A_ = 0.7;
float dt = 0.01;

double oldTime;
double samplePeriod = 10;

void setup() {
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

  Serial.begin(19200);
  Wire.begin();
  ESP32Time rtc(0);
  prefs.begin("sensorError");
  resetSensors();
  

  if (prefs.getType(sensorChar[0]) == 9) {
    retrieveErrorData();
    Serial.println("data retrieved");
  } else {
    Serial.println("data missing, caluclating error");
    calculateError();
  }

  if (!BLE.begin()) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    while (1)
      ;
  }

  BLE.setLocalName("PostureMonitor");
  BLE.setDeviceName("ESP32-E");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorBLEData);
  BLE.addService(sensorService);
  sensorBLEData.writeValue(1);

  BLE.advertise();

}

void loop() {
  BLEDevice central = BLE.central();
  double currentTime = millis();
  
  if ((currentTime - oldTime) >= samplePeriod)
  {
    oldTime = currentTime;
    sensorArray[0] = getAllData();
    
  }
  delay(200);
  for (int i = 0; i < 4; i++)
  {
  Serial.print("|");
  Serial.print(sensorArray[0].sensor[i]->pitch);
  Serial.print("|");
  Serial.print(sensorArray[0].sensor[i]->roll);
  Serial.print("|");
  Serial.println(""); 
  }
  
  
  if (central)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
 
  checkSensorData();
  
  freeMem(sensorArray);
}

sensorTime getAllData() {
  
  sensorTime allData;

  allData.time = rtc.getEpoch();

  for (int i = 0; i < sensorCount; i++) {
    digitalWrite(sensors[i], LOW);
    allData.sensor[i] = (sensorData*)malloc(sizeof(sensorData));

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 12);

    float accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float gyroX = (Wire.read() << 8 | Wire.read()) / 131;
    float gyroY = (Wire.read() << 8 | Wire.read()) / 131;
    float gyroZ = (Wire.read() << 8 | Wire.read()) / 131;

    
    if (accX > 2)
    {
      accX = accX - 4;
    }
    if (accY > 2)
    {
      accY = accY - 4;
    }
    if (accZ > 2)
    {
      accZ = accZ - 4;
    }
    if (gyroX > 250)
    {
      gyroX = gyroX - 500;
    }
    if (gyroY > 250)
    {
      gyroY = gyroY - 500;
    }
    if (gyroZ > 250)
    {
      gyroZ = gyroZ - 500;
    }

    //float inteGyroX = gyroX * dt;
    //float inteGyroY = gyroY * dt;

    float accAngleX = (-atan2(accX, sqrt((accY * accY) + (accZ * accZ))) * 180 / PI);
    float accAngleY = (atan2(accY, sqrt((accX * accX) + (accZ * accZ))) * 180 / PI);

    //allData.sensor[i]->roll = A_ * (allData.sensor[i]->roll + inteGyroX) + (1 - A_) * (accAngleX);
    //allData.sensor[i]->pitch = A_ * (allData.sensor[i]->pitch + inteGyroY) + (1 - A_) * (accAngleY);
    allData.sensor[i]->roll = accAngleX;
    allData.sensor[i]->pitch = accAngleY;

    digitalWrite(sensors[i], HIGH);
    
  }

  return allData;
}

void checkSensorData() {
  if (sensorArray[0].sensor[1]->roll > 25 || sensorArray[0].sensor[1]->roll < -10)
  {
    sensorBLEData.writeValue(1);
    delay(500);
    sensorBLEData.writeValue(0);
    Serial.println("Back bad");
  }
  
}

void resetSensors() {
  for (int i = 0; i < sensorCount; i++) {
    digitalWrite(sensors[i], LOW);
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
  }
}

void freeMem(sensorTime array[]) {
  for (int i = 0; i < 10; i++) {
    {
    for (int j = 0; j < 4; j++)
      if (array[i].sensor[j] != 0)
      {
        free(array[i].sensor[j]);
      }
    }
  }
}

void retrieveErrorData() {
  for (int i = 0; i < 4; i++) {
    //(char*) sensors[i]
    size_t len = prefs.getBytesLength(sensorChar[i]);
    char buffer[len];
    prefs.getBytes(sensorChar[i], buffer, len);

    sensorError* errorBuffer = (sensorError*)buffer;
    sensorErrorArray[i] = *errorBuffer;
  }
}

void calculateError() {

  prefs.clear();

  for (int i = 0; i < sensorCount; i++) {
    int cycles = 0;

    while (cycles < 200) {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 12);

      float accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
      float accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
      float accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
      float gyroX = (Wire.read() << 8 | Wire.read()) / 131;
      float gyroY = (Wire.read() << 8 | Wire.read()) / 131;
      float gyroZ = (Wire.read() << 8 | Wire.read()) / 131;

      if (accX > 2)
      {
        accX = accX - 4;
      }
      if (accY > 2)
      {
        accY = accY - 4;
      }
      if (accZ > 2)
      {
        accZ = accZ - 4;
      }
      if (gyroX > 250)
      {
        gyroX = gyroX - 500;
      }
      if (gyroY > 250)
      {
        gyroY = gyroY - 500;
      }
      if (gyroZ > 250)
      {
        gyroZ = gyroZ - 500;
      }

      //float inteGyroX = gyroX * dt;
      //float inteGyroY = gyroY * dt;


      sensorErrorArray[i].accErrorX = sensorErrorArray[i].accErrorX + (-atan2(accX, sqrt((accY * accY) + (accZ * accZ))) * 180 / PI);
      sensorErrorArray[i].accErrorY = sensorErrorArray[i].accErrorY + (atan2(accY, sqrt((accX * accX) + (accZ * accZ))) * 180 / PI);
      cycles++;
    }

    sensorErrorArray[i].accErrorX = sensorErrorArray[i].accErrorX / 200;
    sensorErrorArray[i].accErrorY = sensorErrorArray[i].accErrorY / 200;
    

    

    Wire.endTransmission(true);


    float errorArray[] = { sensorErrorArray[i].accErrorX,
                           sensorErrorArray[i].accErrorY,
                         };


    prefs.putBytes(sensorChar[i], errorArray, sizeof(errorArray));
  }
}
