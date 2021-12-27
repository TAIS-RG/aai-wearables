/*
  Type 2 configuration.
  @author: Juan Antonio Castro-Garcia
  @email: jacastro@us.es
  @license: CC-BY-SA-NC 4.0

  Note: ArduinoBle and Arduino_LSM6DS3 libraries have been modified by the authors.
*/
#define IMU_FS 26

#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

#define MTU_LEN  36
#define EDA_INIT 0
#define ACC_INIT 2
#define GYR_INIT 8
#define BAT_INIT 14
#define TA_INIT  15
#define ST_INIT  16
#define EDA_CONFIG  17

#define A_BAT A0
#define A_EDA A1
#define A_ST  A2
#define CEDA00 2
#define CEDA01 3
#define CEDA10 4
#define CEDA11 5




uint16_t frame[17];//MTU_LEN / 2

char c_eda = EDA_INIT;
char c_acc = ACC_INIT;
char c_gyr = GYR_INIT;
//char c_bat = BAT_INIT;
//char c_ta = TA_INIT;
//uint16_t contador = 0; 
char CEDA_selection = 2;


// MODIFICAR A PARTIR DE AQUÍ <--------------------------------------------------
int16_t aX, aY, aZ;
 // BLE Battery Service
BLEService batteryService("ACC0");

// BLE Battery Level Characteristic
BLECharacteristic batteryLevelChar("ACC5",  // standard 16-bit characteristic UUID
    BLERead | BLENotify,//); // remote clients will be able to get notifications if this characteristic changes
    MTU_LEN);
    
BLEByteCharacteristic switchCharacteristic("ACC6", BLERead | BLEWriteWithoutResponse);

int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms
long lastTXMillis = 0;  // last time the battery level was checked, in ms
long currentMillis = 0;

void setup() {
  //Serial.begin(115200);    // initialize serial communication
  //while (!Serial);

    if (!IMU.begin(IMU_FS)) {
   // Serial.println("Failed to initialize IMU!");
    while (true); // halt program
  } 

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    //Serial.println("starting BLE failed!");

    while (1);
  }

  BLE.setLocalName("WristMonitor");
  BLE.setAdvertisedService(batteryService); // add the service UUID
  batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  batteryService.addCharacteristic(switchCharacteristic);
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  switchCharacteristic.setValue(2);
  BLE.addService(batteryService); // Add the battery service
  BLE.advertise();

  //Serial.println("Bluetooth device active, waiting for connections...");
  //Serial.println(IMU.accelerationSampleRate());
  pinMode(CEDA00, OUTPUT);
  pinMode(CEDA01, OUTPUT);
  pinMode(CEDA10, OUTPUT);
  pinMode(CEDA11, OUTPUT);
  digitalWrite(CEDA00, LOW);
  digitalWrite(CEDA01, HIGH);
  digitalWrite(CEDA10, LOW);
  digitalWrite(CEDA11, LOW);
}

void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    //Serial.print("Connected to central: ");
    // print the central's BT address:
    //Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      currentMillis = millis();
      if (IMU.accelerationAvailable()) {
        //Serial.println(" ");
        

        
        

        switch(c_eda)
        {
          case 0:
            frame[c_eda] = analogRead(A_EDA);
            IMU.readAcceleration(aX, aY, aZ);
            frame[c_acc]=aX;
            frame[c_acc+1]=aY;
            frame[c_acc+2]=aZ;
            IMU.readGyroscope(aX, aY, aZ);
            frame[c_gyr]=aX;
            frame[c_gyr+1]=aY;
            frame[c_gyr+2]=aZ;
            c_acc+=3;
            c_gyr+=3;
            c_eda++;
            //Serial.println(currentMillis - previousMillis);
            break;
            
          case 1:
            frame[c_eda] = analogRead(A_EDA);
            IMU.readAcceleration(aX, aY, aZ);
            frame[c_acc]=aX;
            frame[c_acc+1]=aY;
            frame[c_acc+2]=aZ;
            IMU.readGyroscope(aX, aY, aZ);
            frame[c_gyr]=aX;
            frame[c_gyr+1]=aY;
            frame[c_gyr+2]=aZ;
            IMU.readTemperature(aX);
            frame[TA_INIT]=aX;
            //frame[TA_INIT]=contador; // modifico aquí
            //contador++;
            frame[BAT_INIT] = analogRead(A_BAT);
            frame[ST_INIT] = analogRead(A_ST);
            frame[EDA_CONFIG] = CEDA_selection;
            
            c_acc = ACC_INIT;
            c_gyr = GYR_INIT;
            batteryLevelChar.writeValue(frame,MTU_LEN,true);
            c_eda = EDA_INIT;
            /*Serial.print(currentMillis - previousMillis);
            Serial.print(" ");
            Serial.println(currentMillis - lastTXMillis);
            lastTXMillis = currentMillis;*/
        }

        //previousMillis = currentMillis;
        
                


        
      }
      /*unsigned char c = 0;
      switchCharacteristic.readValue(&c, 1);
      if(c)
        Serial.println(c);*/
      BLE.poll();
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
  }
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  //Serial.print("Characteristic event, written: ");

  CEDA_selection = switchCharacteristic.value();
  //Serial.println(CEDA_selection, HEX);
  if(CEDA_selection&0x01)
  {
    digitalWrite(CEDA00, HIGH);
  }
  else
  {
    digitalWrite(CEDA00, LOW);
  }
  if(CEDA_selection&0x02)
  {
    digitalWrite(CEDA01, HIGH);
  }
  else
  {
    digitalWrite(CEDA01, LOW);
  }
  if(CEDA_selection&0x04)
  {
    digitalWrite(CEDA10, HIGH);
  }
  else
  {
    digitalWrite(CEDA10, LOW);
  }
  if(CEDA_selection&0x08)
  {
    digitalWrite(CEDA11, HIGH);
  }
  else
  {
    digitalWrite(CEDA11, LOW);
  }
}
