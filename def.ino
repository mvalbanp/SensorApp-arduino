/*
  ECGSignal Monitor

  This example creates a BLE peripheral with the standard service and
  level characteristic. The A0 pin is used to calculate the ECGSignal level.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

 // BLE ECG Service
BLEService ECGService("19B10010-E8F2-537E-4F6C-D104768A1214");

// BLE Signal Level Characteristic
BLECharacteristic ECGSignalChar("19B10021-E8F2-537E-4F6C-D104768A1214",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 219); // remote clients will be able to get notifications if this characteristic changes
BLEIntCharacteristic BPMCharra("19B10022-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

long previousMillis = 0;  // last time the Signal level was checked, in ms

String values;
char charvalues[220];
int counter = 0;

long BPMMillis = 0;
int BPMavg = 0;
int BPMsum = 0;
bool beatup = false;
bool BPMTiming = false;
int BPM = 80;
int peak = 0;

const int ledPin = 2;

#include <PeakDetection.h> // import lib

PeakDetection peakDetection; // create PeakDetection object

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);    // initialize serial communication
  peakDetection.begin(36, 4, 0.4); // sets the lag, threshold and influence
  //while (!Serial);

  //pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("ECG Monitor");
  BLE.setAdvertisedService(ECGService); // add the service UUID
  ECGService.addCharacteristic(ECGSignalChar); // add the Signal level characteristic
  ECGService.addCharacteristic(BPMCharra);
  BLE.addService(ECGService); // Add the ECG service
  ECGSignalChar.writeValue(""); // set initial value for this characteristic
  BPMCharra.writeValue(0);

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  if(peak==1){
    digitalWrite(ledPin, HIGH);
  }
  
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    //digitalWrite(LED_BUILTIN, HIGH);
    analogReadResolution(12);
    // check the  level every millisecond
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 5ms have passed, check the  level:
      if (currentMillis - previousMillis >= 5) {
        previousMillis = currentMillis;
        int ECGSignal = analogRead(A0);
        int ECGSignalLevel = map(ECGSignal, 0, 4095, 100, 999);
        int PPG1Signal = analogRead(A1);
        int PPG1SignalLevel = map(PPG1Signal, 0, 4095, 100, 999);

        peakDetection.add(ECGSignalLevel); // adds a new data point
        peak = peakDetection.getPeak(); // returns 0, 1 or -1

          if(peak==1){
            digitalWrite(ledPin, HIGH);
            if(beatup) {
              BPM=currentMillis-BPMMillis;
              BPM=int(30/(float(BPM)/2000));
              BPMTiming=false;
              beatup=false;
            }
            if(BPMTiming==false) {
              BPMMillis=millis();
              BPMTiming=true;
            }
          }
          if((peak==0)&(BPMTiming))
          beatup=true;

          if(peak==0) digitalWrite(ledPin, LOW);


          
        Serial.print("PPG: ");
        Serial.println(PPG1SignalLevel);
        Serial.print("ECG: ");
        Serial.println(ECGSignalLevel);
        values+= ECGSignalLevel;
        values+= PPG1SignalLevel;
        counter = counter+1;
        
      }

      if (counter >= 36) {
        values+= BPM;
        values.toCharArray(charvalues, 220);
        ECGSignalChar.writeValue(charvalues);
        //BPMCharra.writeValue(BPM);
        //Serial.print(BPM);
        //Serial.println(charvalues);
        values="";
        counter=0;
      }
    }
    // when the central disconnects, turn off the LED:
    //digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
