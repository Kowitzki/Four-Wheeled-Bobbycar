/*TODO: 
- Speed und Torque gleichzeitig
- Geschwindigkeiten, Spannung, Commands, Throttle, Brake nach Android
- Hupe

*/

//Bremslicht und Lenkwinkelsensor ohne Display

#define TRQ

#include "Arduino.h"
#include "Wire.h"
#include "EasyPCF8574.h"
#include <LCD_I2C.h>
//#include "AS5600.h"
#include "SPI.h"
#include "SD.h"

// ########################## DEFINES ##########################
#define MC1_BAUD 115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define MC2_BAUD 115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200 // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define BLE_BAUD 115200     // [-] Baud rate for Bluetooth Dongle
#define START_FRAME 0xABCD // [-] Start frame definition for reliable serial communication
#define TIME_SEND 10       // [ms] Sending time interval
//#define DEBUG_RX

//#define USB Serial
#define MC1 Serial1 //Front A9 gelb TX, A10 weiß RX
#define MC2 Serial2 //Rear A2 gelb TX, A3 weiß RX
#define BLE Serial6

#define pinThrottle     PA1
#define pinBrake        PA0
#define pinDeadman      PB1
#define pinHorn         PB13

#define   pinBrakeOut   1
#define   pinHornOut    0

/*
 * SCL PB6
 * SDA PB9
 * 
 * LCD 0x27
 * PCF 0x26 */
 
// Global variables
double speedF = 0.0;
double speedR = 0.0;
double steerF = 0.0;
double steerR = 0.0;
double kommando = 0.0;
double speedMeasF = 0.0;
double speedMeasR = 0.0;


// Settings
#define stepTime 5
#define steps 100
#define settingLoopT 100
#define waitT 50

#define throttleMax settingValues[0]
#define throttleMin settingValues[1]
#define brakeMax settingValues[2]
#define brakeMin settingValues[3]
#define deadband settingValues[4]
#define driveF settingValues[5]
#define driveR settingValues[6]
#define brakeF settingValues[7]
#define brakeR settingValues[8]
#define allowHorn settingValues[9]
#define allowDerating settingValues[10]
#define allowTrqShft settingValues[11]
#define cells settingValues[12]
#define TrqShft settingValues[13] //linksrechts
#define BalanceTh settingValues[14] //Gas geben
#define BalanceBr settingValues[15] //Bremsbalance
#define DeratingLint settingValues[16]
#define DeratingHint settingValues[17]
#define powerCut settingValues[18]
#define brakeCut settingValues[19]
#define freeWheeling settingValues[20]
#define controlMode settingValues[21] //Torque = 0, Speed = 1
#define allowBrakelight settingValues[22]
#define displayBacklight settingValues[23]
#define maxSpeed settingValues[24] // durch 10
#define TrqShftF settingValues[25]
#define TrqShftR settingValues[26]

#define maxShftSpeed 20.0
#define svLength 26

bool invertTorqueShift = 0;

int settingValues[] = {3531, 415, 4095, 492, 40, 1, 1, 1, 1, 1, 1, 0, 13, 0, 500, 500, 650, 250, 500, 500, 950, 0, 1, 1, 600, 500, 500};
double DeratingL = 0.5 * cells;
double DeratingH = 0.2 * cells;

double voltage = 49.0;
double voltageMin = 60.0;
double cellVoltage = 0.0;
double speedMeas = 0.0;
double speedMeasMax = 0.0;
double maxVoltage = 4.2 * cells;   //13S * 4.2V
double minVoltage = 3.0 * cells;   //13S * 3.0V
bool speedSix = 0;
short part = 0;
double steeringAngle = 0.0;

//byte portExp = 0b00000000;

uint8_t idx1 = 0;        // Index for new data pointer
uint16_t bufStartFrame1; // Buffer Start Frame
byte *p1;                // Pointer declaration for the new received data
byte incomingByte1;
byte incomingBytePrev1;

uint8_t idx2 = 0;        // Index for new data pointer
uint16_t bufStartFrame2; // Buffer Start Frame
byte *p2;                // Pointer declaration for the new received data
byte incomingByte2;
byte incomingBytePrev2;

typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

SerialFeedback NewFeedback1;
SerialFeedback NewFeedback2;

SerialCommand CommandMC1;
SerialFeedback FeedbackMC1;

SerialCommand CommandMC2;
SerialFeedback FeedbackMC2;

unsigned long lastTimeHeardOfMC1 = 0;
unsigned long lastTimeHeardOfMC2 = 0;
bool heardOfMC1 = false;
bool heardOfMC2 = false;

LCD_I2C lcd(0x27);
EasyPCF8574 pcf(0x26,0);
//AS5600 encoder;

#define Sd2Card card;
#define SdVolume volume;
#define SdFile root;
const int chipSelect = PB0;

unsigned long iTimeSend = 0;
unsigned long iTimeCalc = 0;

const char *Line1 = "Buckle Up!";
const char *Line2 = "Be  Careful!";


void setup() {
#ifdef USB
  USB.begin(SERIAL_BAUD);
  //while (!USB){}
#endif
  BLE.begin(BLE_BAUD);

  MC1.begin(MC1_BAUD);
  MC2.begin(MC2_BAUD);
  
  Wire.begin();
  
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print(Line1);
  lcd.setCursor(2, 1);
  lcd.print(Line2);

  //PinModes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinThrottle, INPUT_ANALOG);
  pinMode(pinBrake, INPUT_ANALOG);
  pinMode(pinDeadman, INPUT_PULLDOWN);
  pinMode(pinHorn, INPUT_PULLUP);

  pcf.startI2C();

  //set Pins
  digitalWrite(LED_BUILTIN, HIGH);
  readSettings();

  pcf.WriteBit(true,pinBrakeOut);
  
  if(brake() > 50.0){
  } else {
    speedSix = 1;
    powerCut = 250;
    brakeCut = 250;
  }

  delay(300);
  
  while (analogRead(pinBrake) > (brakeMin + 20) || analogRead(pinThrottle) > (throttleMin + 20));

  lcd.clear();
  iTimeSend = 0;
  iTimeCalc = 0;
}

void loop() {
  unsigned long timeNow = millis();
  


  // ######### GET DATA
  ReceiveMC1();
  ReceiveMC2();

  // ######### CALC DATA
  if (lastTimeHeardOfMC1 + 100 > millis()) {
    heardOfMC1 = true;
  }
  else {
    heardOfMC1 = false;
    FeedbackMC1.speedL_meas = 5.8;
    FeedbackMC1.speedR_meas = - 5.8;
    FeedbackMC1.batVoltage = 0.0;
  }
  if (lastTimeHeardOfMC2 + 100 > millis()) {
    heardOfMC2 = true;
  }
  else {
    heardOfMC2 = false;
    FeedbackMC2.speedL_meas = 5.8;
    FeedbackMC2.speedR_meas = - 5.8;
    FeedbackMC2.batVoltage = 0.0;
  }
  
  speedMeasF = max(FeedbackMC1.speedL_meas, - FeedbackMC1.speedR_meas);
  speedMeasR = max(FeedbackMC2.speedL_meas, - FeedbackMC2.speedR_meas);
  speedMeas = (speedMeasF + speedMeasR) * 0.0305 / 2;
  speedMeasMax = max(speedMeasMax, speedMeas);
  if(heardOfMC1 || heardOfMC2){
    voltage = (FeedbackMC1.batVoltage + FeedbackMC2.batVoltage) / ((double(heardOfMC1) + double(heardOfMC2))*100.0);
    cellVoltage = voltage / cells;
    if (voltage > 30.0) {
      voltageMin = min(voltageMin, voltage);
    }
  }

  //if (digitalRead(pinDeadman)) { //iTimeCalc < timeNow &&
    //iTimeCalc = timeNow + TIME_CALC;

    // ######### CALC SPEED

#ifdef SPD
    speedR = speedR * double(freeWheeling) / 1000.0 + (throttle() * double(powerCut) - brake() * double(brakeCut)) / 1000.0;
    speedF = speedR * (50.0 + double(TrqShft)) / 100.0;

    if (speedSix) {
      speedR = min(speedR, 200.0);
      speedF = min(speedF, 200.0);
    }
#endif

#ifdef TRQ
    kommando = (throttle() * double(powerCut) - brake() * double(brakeCut)) / 100;

    if (speedSix && speedMeas >= 0.0) {
      speedR = min(kommando, (590 - speedMeas * 100));
      speedF = min(kommando, (590 - speedMeas * 100));
    }
    else if (maxSpeed < 500 && speedMeas >= 0.0){
      speedR = min(kommando, (maxSpeed*10 - speedMeas * 100));
      speedF = min(kommando, (maxSpeed*10 - speedMeas * 100));
    }
    else {
      if (kommando >= 0) { //mehr Gas
        if (speedMeas < maxShftSpeed) speedF = (kommando * driveF * (BalanceTh/10 + (speedMeas * (100-BalanceTh/10))/20.0)) / 100;
        else speedF = kommando * driveF;
        speedR = kommando * driveR;
        if (speedMeasR < speedMeasF && speedMeasF > 1.0) { //wenn vorne durchdreht +100 oder so
          speedF = speedF * (speedMeasR / (speedMeasF)) * (speedMeasR / (speedMeasF)); //
        }
      }
      else { //mehr Bremsen
        speedF = kommando * brakeF;
        if (speedMeas < maxShftSpeed) speedR = (kommando * brakeR * (BalanceBr/10 + (speedMeas * (100-BalanceBr/10))/20.0)) / 100;
        else speedR = kommando * brakeR;
        if (speedMeasR < speedMeasF && speedMeasF > 1.0) { //wenn hinten blockiert +100 oder so
          speedR = speedR * (speedMeasR / (speedMeasF)) * (speedMeasR / (speedMeasF)); // * (speedMeasR / (speedMeasF+0.0001))
        }
      }
    }
    if (speedMeas < 0.0) {
      if (speedR < 0) speedR = 0;
      if (speedF < 0) speedF = 0;
    }

#endif

     //######### CALC STEER
        if(allowTrqShft){
          steerF = -(steeringAngle * steeringAngle) * 2 * TrqShftF / 1000 * speedF / 1000;
          steerR = -(steeringAngle * steeringAngle) * 2 * TrqShftR / 1000 * speedR / 1000;
          if (steeringAngle < 0) {
            steerF = -steerF;
            steerR = -steerR;
          }
          if (invertTorqueShift){
            steerF = -steerF;
            steerR = -steerR;
          }
        }
        else{
          steerF = 0.0;
          steerR = 0.0;
        }

  //}
  if (voltage < minVoltage) { // do not do this with maxVoltage, it will brake and kill itself
    speedF = 0.0;
    speedR = 0.0;
    steerF = 0.0;
    steerR = 0.0;
  }

  //digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000); //Blink LED
  //digitalWrite(LED_BUILTIN, !LOW);
  

  // ######## SEND DATA
  if (iTimeSend < timeNow) {
    iTimeSend = timeNow + TIME_SEND;
    
    if (!digitalRead(pinHorn) && allowHorn){ //TODO mach das mal kleiner
      pcf.WriteBit(true,pinHornOut);
    } else {
      pcf.WriteBit(false,pinHornOut);
    }
    if (brake() > 1.0){
      pcf.WriteBit(true,pinBrakeOut);
    }
    else{
      pcf.WriteBit(false,pinBrakeOut);
    }
    delayMicroseconds(300);
    steeringAngle = readAngle();
    delayMicroseconds(300);
    //displayLCD();
    serialCom();
    /*if (!digitalRead(pinDeadman) && (speedF > 0.0 || speedR > 0.0)){
      speedF = 0.0;
      speedR = 0.0;
      steerF = 0.0;
      steerR = 0.0;
    }*/
    #ifndef TESTMODE
      SendMC1(steerF, speedF);
      SendMC2(steerR, speedR);
      /*
      BLE.print(TrqShftR);
      BLE.print("\t");
      BLE.print(TrqShftF);
      BLE.print("\t");
      BLE.print(0);
      BLE.print("\t");
      BLE.print(steerF);
      BLE.print("\t");
      BLE.println(steerR);*/
    #endif
    
  }
}

double readAngle(){
  long _lsb = 0;
  long _msb = 0;
  long _msbMask = 0b00001111;
  double SA = 0.0;
  
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  Wire.endTransmission();
  delayMicroseconds(300);

  Wire.requestFrom(0x36, 1);

  if(Wire.available() <=1) {
    _msb = Wire.read();
  }

  Wire.requestFrom(0x36, 1);

  Wire.beginTransmission(0x36);
  Wire.write(0x0F);
  Wire.endTransmission();

  if(Wire.available() <=1) {
    _lsb = Wire.read();
  }
  _lsb = (_lsb) + (_msb & _msbMask) * 256;
  SA = -(_lsb - 905)/11.38;
  return SA;
}

/*
void displayLCD() {

  if (displayBacklight == 0){
    lcd.noBacklight();
  } else {//(displayBacklight == 1)
    lcd.backlight();
  }
  
  part++;
  part = part%4;
  if (part == 0) {
    lcd.setCursor(0, 0);
    //lcd.print("B");
    //lcd.print("B:   ");
    //lcd.setCursor(2, 0);
    //lcd.print(int(brake()));
    //lcd.print(speedF);
  }/*
  else if (part == 1) {
    lcd.setCursor(8, 0);
    lcd.print("T:   ");
    lcd.setCursor(9, 0);
    lcd.print(int(throttle()));
    //lcd.print();
    lcd.setCursor(15, 0);
    lcd.print(heardOfMC1);
  }
  else if (part == 2) {
    if (speedMeas > 5.0 || brake() < 10.0) {
      lcd.setCursor(0, 1);
      lcd.print("V:");
      lcd.print(cellVoltage, 2);
    }
    else {
      lcd.setCursor(0, 1);
      lcd.print("V,:");
      lcd.print((voltageMin / cells), 2);
    }
  }
  else if (part == 3) {
    if (speedMeas > 5.0 || brake() < 10.0) {
      lcd.print(" S:      ");
      lcd.setCursor(9, 1);
      lcd.print(speedMeas, 1);
      lcd.setCursor(15, 1);
      lcd.print(heardOfMC2);
    }
    else {
      lcd.setCursor(7, 1);
      lcd.print(" S':     ");
      lcd.setCursor(11, 1);
      lcd.print(speedMeasMax, 1);
    }
  }
}*/

#ifdef USB
void serialCom() {
  //USB.print("\t");
  USB.print(kommando * 10);
  USB.print("\t");
  USB.print(speedF * 10);
  USB.print("\t");
  USB.print(speedR * 10);
  USB.print("\t");
  USB.print(speedMeasF);
  USB.print("\t");
  USB.print(speedMeasR);
  USB.print("\t");
  USB.print(speedMeas * 10);
  //USB.print("\t");
  //USB.print();
  USB.println();
}
#endif

#ifdef BLE
void serialCom() { //command wird 3mal statt einmal gesendet
  char a = ' ';
  char command[10];
  bool commandDone = true;
  byte y = 0;
  while (BLE.available()){
    a = char(BLE.read());
    if (a == '+'){
      command[0] = '+';
      y = 1;
      commandDone = false;
    } else {
      if (y < 10){
        command[y] = a;
        y++;
      }
    }
    if(!commandDone && command[0] == '+' && command[4] == ':' && command[9] == '-'){
      BLE.print(command);
    }
    delayMicroseconds(100);
  }

  char delimiter[] = "+:-";
  byte comm = atoi(strtok(command, delimiter));
  int value2 = atoi(strtok(NULL, delimiter));

  
  if (commandDone == false){
    if (comm == 100){
      if (value2 == 0){
        if(analogRead(pinThrottle) < 2048){
          throttleMin = analogRead(pinThrottle);
        }
        if(analogRead(pinBrake) < 2048){
          brakeMin = analogRead(pinBrake);
        }
      } else if (value2 == 1){
        if(analogRead(pinThrottle) > 2048){
          throttleMax = analogRead(pinThrottle);
        }
        if(analogRead(pinBrake) > 2048){
          brakeMax = analogRead(pinBrake);
        }
      }
    } else if (comm == 99){
      if(value2 == 1){
        speedSix = false;
      } else if (value2 == 0){
        speedSix = true;
      }
    } else if (comm == 98){
      if (value2 == 1){
        saveSettings();
      }
    } else if (comm == 97){
      if (value2 == 0){
        BLE.print(voltage/13.0);
      } else if (value2 == 1) {
        BLE.print(voltageMin/13.0);
      }
    } else if (comm == 96){
      if (value2 == 0){
        BLE.print(speedMeas);
      } else if (value2 == 1) {
        BLE.print(speedMeasMax);
      }
    } else if (comm == 95){
      if (value2 == 0){
        invertTorqueShift = 0;
      } else if (value2 == 1) {
        invertTorqueShift = 1;
      }
    } else if (comm <= svLength && comm >= 0){
      settingValues[comm] = value2;
    }
  }
  commandDone = true;
}
#endif



double throttle(){
  double throttle = double(map(analogRead(pinThrottle), throttleMin + deadband, throttleMax - deadband, 0, 1000)) / 10.0;
  // derating if Voltage exceeds minVoltage at throttle
  
  if (allowDerating && voltage < (minVoltage + DeratingL)) {
    throttle = throttle * (1 - (((minVoltage + DeratingL) - voltage) / (DeratingL)));
  }

  //throttle = pow(throttle, 1.8)/39.0;
  //throttle = (throttle*sqrt(throttle))/10.0;
  throttle = throttle * throttle / 100.0;
  
  if (voltage < minVoltage && allowDerating) throttle = 0.0; // || brake() > 5.0
  
  throttle = constrain(throttle, 0.0, 100.0);
  return throttle;
}

double brake(){
  double brake = double(map(analogRead(pinBrake), brakeMin + deadband, brakeMax - deadband, 0, 1000)) / 10.0;
  // derating if Voltage exceeds maxVoltage at braking

  if (allowDerating && (voltage > (maxVoltage - DeratingH)) && voltage < maxVoltage) {
    brake = brake * (1 - ((voltage - (maxVoltage - DeratingH)) / DeratingH));
  }

  brake = brake * brake / 100;

  //brake = (brake*sqrt(brake))/10.0;
  brake = constrain(brake, 0.0, 100.0);
  return brake;
}
/*
double readSteeringAngle(){
  double SA = encoder.getPosition();
  SA += 15;

  return SA;
}*/


// ########################## SEND ##########################
void SendMC1(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  CommandMC1.start = (uint16_t)START_FRAME;
  CommandMC1.steer = (int16_t)uSteer;
  CommandMC1.speed = (int16_t)uSpeed;
  CommandMC1.checksum = (uint16_t)(CommandMC1.start ^ CommandMC1.steer ^ CommandMC1.speed);

  // Write to Serial
  MC1.write((uint8_t *)&CommandMC1, sizeof(CommandMC1));
}

void SendMC2(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  CommandMC2.start = (uint16_t)START_FRAME;
  CommandMC2.steer = (int16_t)uSteer;
  CommandMC2.speed = (int16_t)uSpeed;
  CommandMC2.checksum = (uint16_t)(CommandMC2.start ^ CommandMC2.steer ^ CommandMC2.speed);

  // Write to Serial
  MC2.write((uint8_t *)&CommandMC2, sizeof(CommandMC2));
}

// ########################## RECEIVE ##########################
void ReceiveMC1()
{
  // Check for new data availability in the Serial buffer
  if (MC1.available())
  {
    incomingByte1 = MC1.read();                                          // Read the incoming byte
    bufStartFrame1 = ((uint16_t)(incomingByte1) << 8) | incomingBytePrev1; // Construct the start frame

  }
  else
  {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  USB.print(incomingByte1);
  return;
#endif

  // Copy received data
  if (bufStartFrame1 == START_FRAME)
  { // Initialize if new data is detected
    p1 = (byte *)&NewFeedback1;
    *p1++ = incomingBytePrev1;
    *p1++ = incomingByte1;
    idx1 = 2;
  }
  else if (idx1 >= 2 && idx1 < sizeof(SerialFeedback))
  { // Save the new received data
    *p1++ = incomingByte1;
    idx1++;
  }

  // Check if we reached the end of the package
  if (idx1 == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback1.start ^ NewFeedback1.cmd1 ^ NewFeedback1.cmd2 ^ NewFeedback1.speedR_meas ^ NewFeedback1.speedL_meas ^ NewFeedback1.batVoltage ^ NewFeedback1.boardTemp ^ NewFeedback1.cmdLed); // ,

    // Check validity of the new data
    if (NewFeedback1.start == START_FRAME && checksum == NewFeedback1.checksum)
    {
      // Copy the new data
      memcpy(&FeedbackMC1, &NewFeedback1, sizeof(SerialFeedback));
      lastTimeHeardOfMC1 = millis();

      // Print data to built-in Serial
#ifdef USB/*
      //USB.print("1: ");
      //USB.print(FeedbackMC1.cmd1);
      //USB.print(" 2: ");
      //USB.print(FeedbackMC1.cmd2);
      USB.print(" 3: ");
      USB.print(FeedbackMC1.speedR_meas);
      USB.print(" 4: ");
      USB.print(FeedbackMC1.speedL_meas);
      USB.print(" 5: ");
      USB.print(FeedbackMC1.batVoltage);
      //USB.print(" 6: ");
      //USB.print(FeedbackMC1.boardTemp);
      //USB.print(" 7: ");
      //USB.println(FeedbackMC1.cmdLed);*/
#endif
    }
    else
    {
#ifdef USB/*
      USB.println("Non-valid data skipped");*/
#endif
    }
    //USB.println(checksum, BIN);
    //USB.println(NewFeedback1.checksum, BIN);
    //USB.println("");
    idx1 = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev1 = incomingByte1;
}

void ReceiveMC2()
{
  // Check for new data availability in the Serial buffer
  if (MC2.available())
  {
    incomingByte2 = MC2.read();                                          // Read the incoming byte
    bufStartFrame2 = ((uint16_t)(incomingByte2) << 8) | incomingBytePrev2; // Construct the start frame
  }
  else
  {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  USB.print(incomingByte2);
  return;
#endif

  // Copy received data
  if (bufStartFrame2 == START_FRAME)
  { // Initialize if new data is detected
    p2 = (byte *)&NewFeedback2;
    *p2++ = incomingBytePrev2;
    *p2++ = incomingByte2;
    idx2 = 2;
  }
  else if (idx2 >= 2 && idx2 < sizeof(SerialFeedback))
  { // Save the new received data
    *p2++ = incomingByte2;
    idx2++;
  }

  // Check if we reached the end of the package
  if (idx2 == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback2.start ^ NewFeedback2.cmd1 ^ NewFeedback2.cmd2 ^ NewFeedback2.speedR_meas ^ NewFeedback2.speedL_meas ^ NewFeedback2.batVoltage ^ NewFeedback2.boardTemp ^ NewFeedback2.cmdLed); // ,

    // Check validity of the new data
    if (NewFeedback2.start == START_FRAME && checksum == NewFeedback2.checksum)
    {
      // Copy the new data
      memcpy(&FeedbackMC2, &NewFeedback2, sizeof(SerialFeedback));
      lastTimeHeardOfMC2 = millis();

      // Print data to built-in Serial
#ifdef USB/*
      USB.print("\t\t\t");
      USB.print("1: ");
      USB.print(FeedbackMC2.cmd1);
      USB.print(" 2: ");
      USB.print(FeedbackMC2.cmd2);
      USB.print(" 3: ");
      USB.print(FeedbackMC2.speedR_meas);
      USB.print(" 4: ");
      USB.print(FeedbackMC2.speedL_meas);
      USB.print(" 5: ");
      USB.print(FeedbackMC2.batVoltage);
      USB.print(" 6: ");
      USB.print(FeedbackMC2.boardTemp);
      USB.print(" 7: ");
      USB.println(FeedbackMC2.cmdLed);*/
#endif
    }
    else
    {
#ifdef USB/*
      USB.println("Non-valid data skipped");*/
#endif
    }
    idx2 = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev2 = incomingByte2;
}

void readSettings() {
  File settingFile;
  String values = "";
  int a = 0;
  byte x = 0;
  byte i = 0;
  if (SD.begin(chipSelect)) {
    //USB.println("SD.begin check!");
    delay(500);
  }
  if (SD.exists("sett.txt")) {
    settingFile = SD.open("sett.txt");
    while (settingFile.available()) {
      x = settingFile.read();
      if (x == 88) {
        break;
      }
      delay(5);
      values += char(x);

      if (x == 13) {
        values = values.substring(0, 5);
        a = values.toInt();
        settingValues[i] = a;
        values = "";
        i++;
      }
    }
    settingFile.close();
  }
  else {
    //USB.println("File does not exist");
  }
  DeratingL = double(DeratingLint) / 100.0; //#################################################
  DeratingH = double(DeratingHint) / 100.0;
}

void saveSettings() {
  File settingFile;
  if (SD.exists(F("sett.txt"))) {
    SD.remove(F("sett.txt"));
  }
  settingFile = SD.open(F("sett.txt"), FILE_WRITE);
  delay(20); //100
  if (settingFile) {
    for (int i = 0; i <= svLength; i++) {
      settingFile.println(settingValues[i]);
    }
    settingFile.println("X");

    settingFile.close();
    delay(5); //100
  }
}
