#include <ArduinoBLE.h>
#include <Wire.h>
#include <PeakDetection.h> // import lib
#include <KickFiltersRT.h>

//Data Types: BYTE is an 8-bit space, and WORD is a 16-bit space

const byte address_dev = 36;
const word cmd_I2C_ADDR = 0x00B7; //Long Address
const byte cmd_CHIP_ID = 0x08;
const byte cmd_TS_FREQ = 0x0D;
const byte SIGNAL1_L_A = 0x30;  //Signal Channel 1 lower half Time Slot A
const byte SIGNAL1_H_A = 0x31;  //Signal Channel 1 upper half Time Slot A

const byte SIGNAL1_L_B = 0x38;  //Signal Channel 1 lower half Time Slot B
const byte SIGNAL1_H_B = 0x39;  //Signal Channel 1 upper half Time Slot B

const byte SIGNAL1_L_C = 0x40;  //Signal Channel 1 lower half Time Slot C 
const byte SIGNAL1_H_C = 0x41;  //Signal Channel 1 upper half Time Slot C
const byte FIFO_DATA = 0x2F;



 // BLE ECG Service
BLEService ECGService("19B10010-E8F2-537E-4F6C-D104768A1214");

// BLE Signal Level Characteristic
BLECharacteristic ECGSignalChar("19B10021-E8F2-537E-4F6C-D104768A1214",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 219); // remote clients will be able to get notifications if this characteristic changes
BLEByteCharacteristic ConfigData_DUTY("19B10022-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLENotify);
BLEByteCharacteristic ConfigData_LED1("19B10023-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLENotify);
BLEByteCharacteristic ConfigData_LED3("19B10024-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLENotify);

long previousMillis = 0;  // last time the Signal level was checked, in ms

String values;
char charvalues[220];
int counter = 0;

byte config_duty = 0x1E;
byte config_led1 = 0x03;
byte config_led3 = 0x03;

const int ledPin = 2;

KickFiltersRT<float> filtersRT1;
KickFiltersRT<float> filtersRT2;
KickFiltersRT<float> filtersRT3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  config_ECG_PPG();
  config_ECG_SlotA();
  config_PPG_SlotB();
  config_PPG_SlotC();
  
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("ECG Monitor");
  BLE.setAdvertisedService(ECGService); // add the service UUID
  ECGService.addCharacteristic(ECGSignalChar); // add the Signal level characteristic
  ECGService.addCharacteristic(ConfigData_DUTY);
  ECGService.addCharacteristic(ConfigData_LED1);
  ECGService.addCharacteristic(ConfigData_LED3);
  BLE.addService(ECGService); // Add the ECG service
  ECGSignalChar.writeValue(""); // set initial value for this characteristic
  
  

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(ReadShort(SIGNAL1_H_B), DEC);
  //Serial.println(ReadShort(SIGNAL1_L_B), DEC);
  
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    analogReadResolution(12);
    // check the  level every millisecond
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 5ms have passed, check the  level:
      if (currentMillis - previousMillis >= 5) {
        previousMillis = currentMillis;
        
        int ECGSignal = analogRead(A0);
        String ECGSignalLevel = encode_analog(ECGSignal);
        
        unsigned long PPG1Signal = Read_CH1_SB();   //RED
        unsigned long filtered_ppg1 = filtersRT2.lowpass(PPG1Signal, 3, 200);
        String PPG1SignalLevel = encode_i2c(PPG1Signal);
        
        unsigned long PPG2Signal = Read_CH1_SC();   //GREEN
        unsigned long filtered_ppg2 = filtersRT3.lowpass(PPG2Signal, 3, 200);
        String PPG2SignalLevel = encode_i2c_2(PPG2Signal);

        Serial.println(PPG1Signal);
        Serial.print(",");              //separator
        Serial.println(filtered_ppg1);
        
        values+= ECGSignalLevel;
        values+= PPG1SignalLevel;
        values+= PPG2SignalLevel;
        counter = counter+1;
      }

      if (counter >= 24) {
        values.toCharArray(charvalues, 220);
        ECGSignalChar.writeValue(charvalues);
        values="";
        counter=0;
      }

      if (ConfigData_DUTY.written()) {
        config_duty = ConfigData_DUTY.value();
      }
      if (ConfigData_LED1.written()) {
        config_led1 = ConfigData_LED1.value();
      }
      if (ConfigData_LED3.written()) {
        config_led3 = ConfigData_LED3.value();
      }
      set_configs(config_duty, config_led1, config_led3);
      
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void config_ECG_PPG() {
  WriteShort (0x000F, 0x00, 0x06); //Use the 1MHz oscillator
  WriteShort (0x0010, 0x02, 0x01); //Enable slot A, B and C
  WriteShort (0x001E, 0x00, 0x00); //Disable FIFO status bytes
  WriteShort (0x0020, 0x00, 0x00); //In sleep float inputs
  WriteShort (0x000D, 0x27, 0x10); //Sampling Rate 100Hz
  WriteShort (0x0014, 0x00, 0x00); //FIFO interrupt disabled
  WriteShort (0x0021, 0x00, 0x02); //PAIR34 as differential pair
  WriteShort (0x0022, 0x00, 0x00); //GPIO disabled
  WriteShort (0x0023, 0x00, 0x00); //GPIO disabled
}

void config_ECG_SlotA() {   //NOT USED: the ECG is acquired filtered by AD8232 in A0
  WriteLong (0x0100, 0x00, 0x00); //Input resistor 500 ohms
  WriteLong (0x0101, 0x00, 0xE6); //TIA, integrator, and ADC; skip preconditioning; No bpf
  WriteLong (0x0102, 0x00, 0x70); //IN3&IN4 differential pair to CH 1. Refer to 0x0021
  WriteLong (0x0103, 0x00, 0x00);
  WriteLong (0x0104, 0xE2, 0x81); //integrator resistor 400k; TIA vref 0.8855V; TIA gain, CH2 resistor gain 200k, CH1 resistor gain 100k
  WriteLong (0x010D, 0x00, 0x00); //No Pattern
  WriteLong (0x0110, 0x00, 0x03); //3 bytes signal
  WriteLong (0x0112, 0x00, 0x00); //No decimation
  WriteLong (0x0107, 0x01, 0x40); //1 ADC cycle; 64 number of pulses
  WriteLong (0x0108, 0x10, 0x00); //float connect mode, min period
  WriteLong (0x010A, 0x00, 0x03); //Integrator clock width 3us
  WriteLong (0x010B, 0x00, 0x0D); //Integrator timing offset
  WriteLong (0x010C, 0x02, 0x10); //Mod pulse width, mod offset 16us
  WriteLong (0x0105, 0x00, 0x00); //LEDs off
  WriteLong (0x0106, 0x00, 0x00); //LEDs off
  WriteLong (0x010E, 0x00, 0x00); //No ADC adjust CH1
  WriteLong (0x010F, 0x00, 0x00); //No ADC adjust CH2
}

void config_PPG_SlotB() {
  WriteLong (0x0120, 0x00, 0x00); //Input resistor 500 ohms
  WriteLong (0x0121, 0x41, 0xDA); //8us preconditioning, TIA, BPF, integrator, and ADC
  WriteLong (0x0122, 0x00, 0x01); //IN1 connected to CH 1. IN2 disconnected.
  WriteLong (0x0123, 0x50, 0x82); //Precondition with TIA_VREF.
  WriteLong (0x0124, 0xE3, 0xC0); //integrator resistor 400k; TIA vref 1.265V; TIA gain, CH2 resistor gain 200k, CH1 resistor gain 200k
  WriteLong (0x0125, 0x00, 0x03); //LED1A at 200mA and LED2A off 
  WriteLong (0x0126, 0x00, 0x00); //LED3A off
  WriteLong (0x0127, 0x01, 0x40); //1 ADC cycle; 64 number of pulses
  WriteLong (0x0128, 0x00, 0x00); //TIA continous connect mode, min period
  WriteLong (0x0129, 0x1E, 0x10); //led width = 30us = 46% cycle / led offset = 16 us
  WriteLong (0x012A, 0x00, 0x1F); //Integrator clock width 31us
  WriteLong (0x012B, 0x10, 0x10); //Integrator timing offset 16.5us
  WriteLong (0x012C, 0x00, 0x01); //No modulation
  WriteLong (0x0130, 0x00, 0x03); //3 bytes signal
  WriteLong (0x012D, 0x00, 0x99); //-++- int pattern
  WriteLong (0x012E, 0x00, 0x00); //No ADC adjust CH1
  WriteLong (0x012F, 0x00, 0x00); //No ADC adjust CH2
  WriteLong (0x0132, 0x00, 0x00); //No decimation
}

void config_PPG_SlotC() {
  WriteLong (0x0140, 0x00, 0x00); //Input resistor 500 ohms
  WriteLong (0x0141, 0x41, 0xDA); //8us preconditioning, TIA, BPF, integrator, and ADC
  WriteLong (0x0142, 0x00, 0x01); //IN1 connected to CH 1. IN2 disconnected.
  WriteLong (0x0143, 0x50, 0x82); //Precondition with TIA_VREF.
  WriteLong (0x0144, 0xE3, 0xC0); //integrator resistor 400k; TIA vref 1.265V; TIA gain, CH2 resistor gain 200k, CH1 resistor gain 200k
  WriteLong (0x0145, 0x00, 0x00); //LED1A and LED2A off 
  WriteLong (0x0146, 0x00, 0x03); //LED3A at 200mA
  WriteLong (0x0147, 0x01, 0x40); //1 ADC cycle; 64 number of pulses
  WriteLong (0x0148, 0x00, 0x00); //TIA continous connect mode, min period
  WriteLong (0x0149, 0x1E, 0x10); //led width = 30us = 46% cycle / led offset = 16 us
  WriteLong (0x014A, 0x00, 0x1F); //Integrator clock width 31us
  WriteLong (0x014B, 0x10, 0x10); //Integrator timing offset 16.5us
  WriteLong (0x014C, 0x00, 0x01); //No modulation
  WriteLong (0x0150, 0x00, 0x03); //3 bytes signal
  WriteLong (0x014D, 0x00, 0x99); //-++- int pattern
  WriteLong (0x014E, 0x00, 0x00); //No ADC adjust CH1
  WriteLong (0x014F, 0x00, 0x00); //No ADC adjust CH2
  WriteLong (0x0152, 0x00, 0x00); //No decimation
}

word ReadShort (const byte register_add) {
  word result = 0;
  byte aux1 = 0;
  byte aux2 = 0;

  Wire.beginTransmission(address_dev);
  Wire.write(register_add);
  Wire.endTransmission();

  Wire.requestFrom(address_dev, 2);
  aux1 = Wire.read();
  aux2 = Wire.read();
  result = (aux1 * 256) + aux2;
  return result;
}

word ReadLong (const word register_add) {
  byte add_1, add_2;
  add_2 = register_add;
  add_1 = ((unsigned int)register_add >> 8) + 0x80; //Due to LS bit

  word result = 0;
  byte aux1 = 0;
  byte aux2 = 0;

  Wire.beginTransmission(address_dev);
  Wire.write(add_1);
  Wire.write(add_2);
  Wire.endTransmission();

  Wire.requestFrom(address_dev, 2);
  aux1 = Wire.read();
  aux2 = Wire.read();
  result = (aux1 * 256) + aux2;
  return result;
}

void WriteShort (const byte register_add, const byte register_conf_H, const byte register_conf_L) {
  Wire.beginTransmission(address_dev);
  Wire.write(register_add);
  Wire.write(register_conf_H);
  Wire.write(register_conf_L);
  int err;
  err = Wire.endTransmission();
  if (err != 0) {
    Serial.println("ERROR WRITING!");
  }
}

void WriteLong (const word register_add, const byte register_conf_H, const byte register_conf_L) {
  byte add_1, add_2;
  add_2 = register_add;
  add_1 = ((unsigned int)register_add >> 8) + 0x80; //Due to LS bit

  Wire.beginTransmission(address_dev);
  Wire.write(add_1);
  Wire.write(add_2);
  Wire.write(register_conf_H);
  Wire.write(register_conf_L);
  int err;
  err = Wire.endTransmission();
  if (err != 0) {
    Serial.println("ERROR WRITING!");
  }
}

unsigned long Read_CH1_SA(){
  unsigned long result = 0;
  word aux1 = 0;
  word aux2 = 0;
  
  aux1 = ReadShort(SIGNAL1_H_A);
  aux2 = ReadShort(SIGNAL1_L_A);
  result = (aux1 * 65536) + aux2;
  return result;
}

unsigned long Read_CH1_SB(){
  unsigned long result = 0;
  word aux1 = 0;
  word aux2 = 0;
  
  aux1 = ReadShort(SIGNAL1_H_B);
  aux2 = ReadShort(SIGNAL1_L_B);
  result = (aux1 * 65536) + aux2;
  return result;
}

unsigned long Read_CH1_SC(){
  unsigned long result = 0;
  word aux1 = 0;
  word aux2 = 0;
  
  aux1 = ReadShort(SIGNAL1_H_C);
  aux2 = ReadShort(SIGNAL1_L_C);
  result = (aux1 * 65536) + aux2;
  return result;
}

String encode_analog(int data) {
  String encoded_string;
  String string = String(data, HEX);
  if (string.length() == 1) {
    encoded_string = String("00"+string);
  } 
  else if (string.length() == 2) {
    encoded_string = String("0"+string);
  } 
  else encoded_string = String(string);
  return encoded_string;
}

String encode_i2c(unsigned long data) {
  String encoded_string;
  long a1 = constrain(data, 0, 524287);
  long aux = map(a1, 0, 524287, 0, 4095);
  String string = String(aux, HEX);
  if (string.length() == 1) {
    encoded_string = String("00"+string);
  } 
  else if (string.length() == 2) {
    encoded_string = String("0"+string);
  } 
  else encoded_string = String(string);
  return encoded_string;
}

String encode_i2c_2(unsigned long data) {
  String encoded_string;
  long a1 = constrain(data, 0, 524287);
  long aux = map(a1, 0, 524287, 0, 4095);
  String string = String(aux, HEX);
  if (string.length() == 1) {
    encoded_string = String("00"+string);
  } 
  else if (string.length() == 2) {
    encoded_string = String("0"+string);
  } 
  else encoded_string = String(string);
  return encoded_string;
}

void set_configs(byte con_d, byte con_1, byte con_3) {      
        WriteLong (0x0125, 0x00, con_1);
        WriteLong (0x0146, 0x00, con_3);
        WriteLong (0x0129, con_d, 0x10);
        WriteLong (0x012A, 0x00, (con_d+1));
        WriteLong (0x0149, con_d, 0x10);
        WriteLong (0x014A, 0x00, (con_d+1));
}
