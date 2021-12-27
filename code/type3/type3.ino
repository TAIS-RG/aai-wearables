/*
  Type 3 configuration.
  @author: Juan Antonio Castro-Garcia
  @email: jacastro@us.es
  @license: CC-BY-SA-NC 4.0

  Note: ArduinoBle and Arduino_LSM6DS3 libraries have been modified by the authors.
*/

#define IMU_FS 208
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

#define MTU_LEN 64//72
#define ECG_INIT 0
#define ACC_INIT 16//20
#define GYR_INIT 22//26
#define BR_INIT 28//32
#define BAT_INIT 30//34
#define TA_INIT 31//35

#define A_BAT A0
#define A_ECG A1
#define A_BR A2

#define ECG_MAX 19 //20-1

//uint16_t contador = 0; 

uint16_t frame[36];//MTU_LEN / 2

char c_ecg = ECG_INIT;
char c_acc = ACC_INIT;
char c_gyr = GYR_INIT;
char c_br = BR_INIT;
//char c_bat = BAT_INIT;
//char c_ta = TA_INIT;

// MODIFICAR A PARTIR DE AQUÍ <--------------------------------------------------
int16_t aX, aY, aZ;
 // BLE Battery Service
BLEService batteryService("ACC0");

// BLE Battery Level Characteristic
BLECharacteristic batteryLevelChar("ACC5",  // standard 16-bit characteristic UUID
    BLERead | BLENotify,//); // remote clients will be able to get notifications if this characteristic changes
    MTU_LEN);

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

  BLE.setLocalName("ChestMonitor");
  BLE.setAdvertisedService(batteryService); // add the service UUID
  batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  BLE.addService(batteryService); // Add the battery service
  BLE.advertise();

  //Serial.println("Bluetooth device active, waiting for connections...");
  //Serial.println(IMU.accelerationSampleRate());
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
        

        frame[c_ecg] = analogRead(A_ECG);
        IMU.readAcceleration(aX, aY, aZ);
        c_ecg++;

        switch(c_ecg)
        {
          case 8:
            frame[c_acc]=aX;
            frame[c_acc+1]=aY;
            frame[c_acc+2]=aZ;
            IMU.readGyroscope(aX, aY, aZ);
            frame[c_gyr]=aX;
            frame[c_gyr+1]=aY;
            frame[c_gyr+2]=aZ;
            frame[c_br] =analogRead(A_BR);
            //Serial.println(frame[c_br]);
            c_acc+=3;
            c_gyr+=3;
            c_br++;
            break;
          case 16:
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
            frame[c_br] = analogRead(A_BR);
            //Serial.println(frame[c_br]);
            frame[BAT_INIT] = analogRead(A_BAT);
            
            c_acc = ACC_INIT;
            c_gyr = GYR_INIT;
            c_br = BR_INIT;
            batteryLevelChar.writeValue(frame,MTU_LEN,true);
            c_ecg = ECG_INIT;
            
        }
        /*if(c_ecg < 20)
        {
          //Serial.println(currentMillis - previousMillis);
        }
        else
        {
          Serial.print(currentMillis - previousMillis);
          Serial.print(" ");
          Serial.println(currentMillis - lastTXMillis);
          lastTXMillis = currentMillis;
          c_ecg = ECG_INIT;
        }
        previousMillis = currentMillis;*/
        
                


        
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
  }
}
