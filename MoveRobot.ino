#include <AbsoluteIMU.h>
#include <ACCLNx.h>
#include <AngleSensor.h>
#include <BaseI2CDevice.h>
#include <DISTNx.h>
#include <EV3Color.h>
#include <EV3Gyro.h>
#include <EV3InfraRed.h>
#include <EV3SensorAdapter.h>
#include <EV3Sonar.h>
#include <LineLeader.h>
#include <MagicWand.h>
#include <MsTimer2.h>
#include <NumericPad.h>
#include <NXShield.h>
#include <NXShieldAGS.h>
#include <NXShieldI2C.h>
#include <NXTCam.h>
#include <NXTCurrentMeter.h>
#include <NXTHID.h>
#include <NXTLight.h>
#include <NXTMMX.h>
#include <NXTPowerMeter.h>
#include <NXTServo.h>
#include <NXTTouch.h>
#include <NXTUS.h>
#include <NXTVoltMeter.h>
#include <PFMate.h>
#include <PiLight.h>
#include <PSPNx.h>
#include <RCXLight.h>
#include <RTC.h>
#include <SHDefines.h>
#include <SoftI2cMaster.h>
#include <SumoEyes.h>
#include <Wire.h>
#include <NXShield.h>
#include <NXTUS.h>

#include <SoftwareSerial.h>
//Don't make # symbols
//Bank_A: front-right:1, back-right:2
//Bank_B: front-left: 1, back-left: 2

// The shield
NXShield nxshield;

// 2 line LCD on arduino digital pin #8
SoftwareSerial lcd(2, 8);

// Elevator H-Bridge control pins
int q1 = 13;
int q2 = 11;
int q3 = 10;
int q4 = 9;

//
// declare the i2c devices used on NXShield(s).
//
NXTUS       sonarB; // left
NXTUS       sonarA; // right
NXTUS       sonarBack;
NXTTouch    touchSensor; // used for chain endstops

int updateDelay = 50; // X ms sensor / screen update time
int nudgeTime = 180; // oifnviownerfvwwefvpinm
int scanSpeed = 50;
int scanSideDist = 7; 
boolean cornerScan = false;

void setup(){
  Serial.begin(115200);
  
  lcd.begin(9600); 
  lcd.print("Setup Starting...");
  delay(500);
  
  // setup output pins
  pinMode(q1, OUTPUT);
  pinMode(q2, OUTPUT);
  pinMode(q3, OUTPUT);
  pinMode(q4, OUTPUT);
  
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  
  nxshield.init(SH_HardwareI2C);
  
  float batVolt = (float) nxshield.bank_a.nxshieldGetBatteryVoltage() / 1000;
  if(batVolt < 7.50) {
    clearDisplay();
    beep();
    lcd.print("LOW VOLTAGE!!!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt);
  } else {
    clearDisplay();
    lcd.print("Press GO!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt); 
  }
  
  nxshield.waitForButtonPress(BTN_GO);
  
  nxshield.bank_a.motorReset();
  nxshield.bank_b.motorReset();
  
  //
  // Initialize the i2c sensors.
  //
  sonarB.init( &nxshield, SH_BAS2 );
  sonarA.init( &nxshield, SH_BBS1 );
  sonarBack.init( &nxshield, SH_BBS2 );
  touchSensor.init(&nxshield, SH_BAS1);
  
  
  // sensor diagnostic mode.
if(nxshield.getButtonState(BTN_RIGHT)){
   clearDisplay();
   lcd.print("Diagnostic Mode!");
   delay(5000);
   int backUS = sonarBack.getDist();
   int rightUS = sonarA.getDist();
   int leftUS = sonarB.getDist();
   boolean endStops = touchSensor.isPressed();
   while(!nxshield.getButtonState(BTN_RIGHT)){
     delay(updateDelay);
     backUS = sonarBack.getDist();
     rightUS = sonarA.getDist();
     leftUS = sonarB.getDist();
     endStops = touchSensor.isPressed();
     
     clearDisplay();
     lcd.print("bUS: ");
     lcd.print(backUS);
     lcd.print(" rUS: ");
     lcd.print(rightUS);
     setLCDCursor(16);
     lcd.print("lUS: ");
     lcd.print(leftUS);
     lcd.print(" btn: ");
     lcd.print(endStops);
     
     if(endStops){
       frontWheelsGo();
     }else{
       frontWheelsStop();
     }
   } 
    
}
  //approachHoop();
  //approachBackWall(20, 5);
//  scan(30, 5, 5, 10);  // backWall distance, backThresh, ballThresh, sideThresh
//  elevatorRaise();
//  delay(1500);
//  //elevatorPassiveStop();
//  elevatorPassiveStop();
//  delay(1500);
//  elevatorLower();
//  delay(1500);
//  elevatorPassiveStop();
//  delay(1500);

  //scan(35,3);
//  gyro gyroSensor;
//  gyro testGyro;
//  IMU.readGyro(gyroSensor);
//  delay(2000);
//  IMU.readGyro(testGyro);
//  lcd.print(testGyro.gx==gyroSensor.gx);
//  setLCDCursor(16);
//  lcd.print(testGyro.gy==gyroSensor.gy);
//  delay(2000);
//  while(true){
////    lcd.print(gyroSensor.gx);
////    setLCDCursor(16);
//    delay(updateDelay);
//    clearDisplay();
//    IMU.readGyro(gyroSensor);
//    
//    lcd.print(gyroSensor.gx);
//    setLCDCursor(16);
//    lcd.print(gyroSensor.gz);

//  }
    findCenter(2);
    approachHoop();
    takeAShot();

//approachHoop();\

//  takeAShot();
//  elevatorReadyPosition();
//  delay(500);
//  nudgeElevatorDown();
//  delay(500);
//  while(!touchSensor.isPressed()) {
//  nudgeElevatorUp();
//  delay(200);
//  }
//  elevatorPassiveStop();


  
} // end setup

void
loop(){
    scan(45, 3);
}

void beep() {
 digitalWrite(6, HIGH);
 delay(50);
 digitalWrite(6, LOW); 
}

// CAUTION. DO NOT ACTIVATE RED PIN OF THE LIGHT. LCD IS USING THAT PIN FOR SERIAL COMMUNICATION.
void
coloryLights(){
  //nxshield.ledSetRGB(8,0,0); // guessing that this is the red pin...
  delay(1000);
  nxshield.ledSetRGB(0,8,0);
  delay(1000);
  nxshield.ledSetRGB(0,0,8);
  delay(1000);
  nxshield.ledSetRGB(0,0,0);
  delay(1000);
}

// dist - desired distance (~cm) from the back wall of the field.
// backThresh - the threshold (~cm) before the robot will attempt to correct for drift.
// ballThresh - the threshold (~cm) of sudden distance change that will trigger a ball found state.
// sideDist - the distance (~cm) from the sides of the field that the robot will maintain while scanning.

void scan(int dist, int backThresh) {
  findCenter(3);
  approachBackWall(dist, backThresh);
  
  int backDist = sonarBack.getDist();
  int distSum = 0;
  int averageDist = 0;
  int sensorCount = 0;
  int scanCount = 0;
  boolean  foundBall = false;
  cornerScan = false;
  
  while(!foundBall){
    scanCount++; 
    if(scanCount > 2){
       cornerScan = true;
    } else {
      cornerScan = false;
    }
    
    if(cornerScan){
       frontWheelsGo();
       moveBackward(30);
       delay(2000);
       stopMoving();
       strafeLeft(30);
       delay(2000);
       stopMoving();
       moveForward(30);
       delay(2000);
       stopMoving(); 
       frontWheelsStop();
       
       scanCount = 0;
     }
    
    alignLeft(dist, scanSideDist, backThresh);
    distSum = 0;
    averageDist = 0;
    sensorCount = 0;
    strafeRight(scanSpeed);
   
    while(sonarA.getDist() > scanSideDist){
      delay(updateDelay);
      
      sensorCount++;      
      if(sensorCount > 30){
         sensorCount = 0;
         distSum = 0; 
         averageDist = 0;
      }
      
      backDist = sonarBack.getDist();
      distSum += backDist;
      averageDist = distSum / sensorCount; 
      
      clearDisplay();
      lcd.print(backDist);
      setLCDCursor(16);
      lcd.print(averageDist);
      
      if(backDist > 100) {
        foundBall = true;
        stopMoving();
        grabBall(40);
        //approachHoop();
        takeAShot();
        
        break;
      }
    } //end strafe while
    
    if(!foundBall) {
      delay(1000);
      stopMoving();
      approachBackWall(dist, backThresh);
    }
  } //end find ball while
  
  //void alignLeft(int backDist, int sideDist, int backThresh)
}

void grabBall(int speed) {
 stopMoving();
 delay(100);
 nudgeElevatorDown(300);
 moveBackward(speed);
 delay(2500);
 stopMoving();
 delay(100);
 moveForward(20);
 delay(500);
 nudgeElevatorUp(400);
 delay(100);
 moveBackward(30);
 delay(1500);
 moveForward(50);
 delay(1500);
 stopMoving(); 
 approachHoop();
 }

void
takeAShot(){
  beep();
  elevatorRaise();
  while(!touchSensor.isPressed()){
    lcd.write("Kobe!");
  }
  elevatorPassiveStop();
  delay(500);
  elevatorReadyPosition();
  delay(200);
  moveBackward(40);
  delay(1000);
  stopMoving();
  clearDisplay();
}

// moves the bot to within dist cm, give of take thresh cm, of the back wall
void approachBackWall(int dist, int thresh) {
  clearDisplay();
  lcd.print("approching back wall!");
  int curDist = sonarBack.getDist() - dist;
  
  while(abs(curDist) > thresh)  {
    delay(updateDelay);
    curDist = sonarBack.getDist() - dist;
    
    clearDisplay();
    lcd.print("approachBackWall()"); 
    setLCDCursor(16);
    lcd.print("curDist: ");
    lcd.print(curDist);
    
    if (curDist < 0) {
      moveForward(50);
    } else {
      moveBackward(50);
    }
  }
  stopMoving();
  clearDisplay();
}

// centers the robot, and then moves forward till the bot is ~62cm from the hoop wall.
void approachHoop() {
  //findCenter(3);
  clearDisplay();
  lcd.write("approachHoop()");
  
  delay(500);
  
  moveForward(50);
  while(sonarBack.getDist() < 58) { 
  }
  
  clearDisplay();
  stopMoving();
  if(abs(sonarA.getDist() - sonarB.getDist()) > 6){
    findCenter(3);
  }
}

// a really silly method that just moves the thing left. hopefully. 
void alignLeft(int backDist, int sideDist, int backThresh){
 clearDisplay();
 lcd.write("aligning left...");
 
 int distB = sonarB.getDist() - sideDist;
 int curDist = sonarBack.getDist() - backDist;
 int ballDist = sonarBack.getDist();
 
 while(distB > 0) {
  delay(updateDelay); 
  distB = sonarB.getDist() - sideDist; 
  curDist = abs(sonarBack.getDist() - backDist) ;
  ballDist = sonarBack.getDist();


  strafeLeft(scanSpeed); 
  
  clearDisplay();
  lcd.print("Aligning left...");
  setLCDCursor(16);
  lcd.print("left dist: ");
  lcd.print(distB);
  
    if(ballDist > 100) {
        stopMoving();
        grabBall(40);
        //approachHoop();
        takeAShot();
        break;
    }
//  if(curDist > backThresh) {
//    //stopMoving();
//    clearDisplay();
//    lcd.write("correcting dist. !");
//    delay(100);
//    approachBackWall(backDist, 3);
//  }
 }
 delay(1000);
 stopMoving();
     if(cornerScan){
       frontWheelsGo();
       moveBackward(30);
       delay(2000);
       stopMoving();
       strafeRight(30);
       delay(2000);
       stopMoving();
       moveForward(30);
       delay(2000);
       stopMoving(); 
       frontWheelsStop();
    }
   approachBackWall(backDist, 3);
   clearDisplay();
}

// centers the robot left / right on the field
void findCenter(int threshold) {
  int distA = sonarA.getDist();
  int distB = sonarB.getDist();
  
  clearDisplay();
  lcd.write("findCenter()..");
  
  while(abs(distA - distB) > threshold) {
  delay(updateDelay);
    distA = sonarA.getDist();
    distB = sonarB.getDist();
    
    clearDisplay();
    lcd.print("Finding Center...");
    setLCDCursor(16);
    lcd.print("A: ");
    lcd.print(distA);
    lcd.print(" B: ");
    lcd.print(distB);

    if(distA > distB) {
      strafeRight(50); 
    } else {
      strafeLeft(50);
    }
  }
  
  clearDisplay();
  stopMoving();
}

void breakTime(){
  approachHoop();
  findCenter(3);
  stopMoving();
  clearDisplay();
  lcd.print("taking a break...");
  delay(2000);
  clearDisplay();
}


/////////////////////////// MOVEMENT COMMANDS //////////////////////////

void
moveForward(int speed1){
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
}

void
moveBackward(int speed1){
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
}

void
stopMoving(){
//  nxshield.bank_a.motorStop(SH_Motor_Both, SH_Next_Action_Float);
//  nxshield.bank_b.motorStop(SH_Motor_Both, SH_Next_Action_Float);
  nxshield.bank_a.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
  nxshield.bank_b.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
}
void
strafeRight(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
}

void
strafeLeft(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  
}

void
turnClockwise(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
}
void
turnCounterClockwise(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
}

void
diagonal(){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, 40);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, 90);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, 90);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, 40);
  
}

void nudgeElevatorUp(){
  elevatorRaise();
  delay(nudgeTime);
  elevatorPassiveStop(); 
}

void nudgeElevatorUp(int n){
 elevatorRaise();
 delay(n);
 elevatorPassiveStop();
}

void nudgeElevatorDown(){
  elevatorLower();
  delay(nudgeTime);
  elevatorPassiveStop();
}

void nudgeElevatorDown(int n){
  elevatorLower();
  delay(n);
  elevatorPassiveStop();
}

void elevatorReadyPosition(){
  elevatorRaise();
  while(!touchSensor.isPressed()){
  }
  nudgeElevatorDown();
  delay(200);
  elevatorLower();
  while(!touchSensor.isPressed()){
  }
  nudgeElevatorUp(300);
  elevatorPassiveStop(); 
}

void frontWheelsStop(){
  digitalWrite(5, LOW);
}

void frontWheelsGo(){
  digitalWrite(5, HIGH);  
}


///////////////////////////// ELEVATOR COMMANDS ////////////////////////////
void elevatorLower() {
 digitalWrite(q1, LOW);
 digitalWrite(q2, HIGH);
 digitalWrite(q3, HIGH);
 digitalWrite(q4, LOW);
}

void elevatorRaise() {
 digitalWrite(q1, HIGH);
 digitalWrite(q2, LOW);
 digitalWrite(q3, LOW);
 digitalWrite(q4, HIGH);
}

void elevatorPassiveStop() {
 digitalWrite(q1, LOW);
 digitalWrite(q2, LOW);
 digitalWrite(q3, LOW);
 digitalWrite(q4, LOW);
}

//void elevatorSuddenStop() {
// digitalWrite(q1, HIGH);
// digitalWrite(q2, HIGH);
// digitalWrite(q3, HIGH);
// digitalWrite(q4, HIGH);
//}


///////////////////////////// LCD COMMANDS //////////////////////////////
void clearDisplay() {
  lcd.write(0xFE);
  lcd.write(0x01); 
}

void setLCDCursor(byte cursor_position){
 lcd.write(0xFE); // ready LCD for special command
 lcd.write(0x80); // ready LCD to recieve cursor potition
 lcd.write(cursor_position); // send cursor position 
}
#
