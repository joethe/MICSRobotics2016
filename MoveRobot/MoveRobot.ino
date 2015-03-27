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
#include <NXShield.h>
#include <NXShieldAGS.h>
#include <NXShieldI2C.h>
#include <NXTCam.h>
#include <NXTHID.h>
#include <NXTLight.h>
#include <NXTPowerMeter.h>
#include <NXTServo.h>
#include <NXTTouch.h>
#include <NXTUS.h>
#include <NXTVoltMeter.h>
#include <PFMate.h>
#include <PSPNx.h>
#include <RTC.h>
#include <SoftI2cMaster.h>
#include <SumoEyes.h>
#include <Wire.h>
#include <NXShield.h>
#include <NXTUS.h>

#include <SoftwareSerial.h>
//Don't make # symbols

// The shield
NXShield nxshield;

// 2 line LCD on arduino digital pin #8
SoftwareSerial lcd(2, 8);

// Elevator H-Bridge control pins
// Not using these?
int q1 = 13;
int q2 = 11;
int q3 = 10;
int q4 = 9;

//
// Declare the i2c devices used on NXShield(s).
//
NXTUS       sonarLeft; // left
NXTUS       sonarRight; // right
NXTUS       sonarBack;
//NXTMMX      mmx; // Multiplexer for chain morors
//NXTTouch    touchSensor; // used for chain endstops

//
// Config Values and loop tuneing.
//
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
  
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  
  pinMode(q1, OUTPUT);
  pinMode(q2, OUTPUT);
  pinMode(q3, OUTPUT);
  pinMode(q4, OUTPUT);
  
  nxshield.init(SH_HardwareI2C);
  
  // Check battery voltage on startup. Warn if low.
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
  sonarLeft.init( &nxshield, SH_BAS2 );
  sonarRight.init( &nxshield, SH_BBS1 );
  sonarBack.init( &nxshield, SH_BBS2 );  
//  mmx.init( &nxshield, SH_BAS1);
  
  // sensor diagnostic mode.
if(nxshield.getButtonState(BTN_RIGHT)){
   clearDisplay();
   lcd.print("Diagnostic Mode!");
   delay(5000);
   int backUS = sonarBack.getDist();
   int rightUS = sonarRight.getDist();
   int leftUS = sonarLeft.getDist();
   //boolean endStops = touchSensor.isPressed();
   while(!nxshield.getButtonState(BTN_RIGHT)){
     delay(updateDelay);
     backUS = sonarBack.getDist();
     rightUS = sonarLeft.getDist();
     leftUS = sonarRight.getDist();
     
     clearDisplay();
     lcd.print("bUS: ");
     lcd.print(backUS);
     lcd.print(" rUS: ");
     lcd.print(rightUS);
     setLCDCursor(16);
     lcd.print("lUS: ");
     lcd.print(leftUS);
     lcd.print(" btn: ");
     lcd.print("NA");
     
     if(nxshield.getButtonState(BTN_RIGHT)){
       //chain up
     }else if(nxshield.getButtonState(BTN_LEFT)){
       //chain down
     }else {
       //chain stop  
     } 
   }
    
}
    
 //   approachHoop();
  //  findCenter(2);
 //   takeAShot();

  
} // end setup

//
// Main Loop!
//
void loop(){
  scan(5, 1);
}

//
///////////// Helper Functions
//

// Activates the noise maker
void beep() {
 digitalWrite(6, HIGH);
 delay(50);
 digitalWrite(6, LOW); 
}

// CAUTION. DO NOT ACTIVATE RED PIN OF THE LIGHT. LCD IS USING THAT PIN FOR SERIAL COMMUNICATION.
void coloryLights(){
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
   
    while(sonarLeft.getDist() > scanSideDist){
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


// Moves forward and grabs a ball (assumes ball is there...)

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
 //approachHoop();
 }
 

// Takes a shot.
void takeAShot(){
  beep();
  elevatorRaise();
//  while(!touchSensor.isPressed()){
//    lcd.write("Kobe!");
//  }
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
  
  if(abs(sonarLeft.getDist() - sonarRight.getDist()) > 6){
    findCenter(3);
  }
  
  delay(500);
  
  moveForward(50);
  while(sonarBack.getDist() < 60) { 
  }
  
  clearDisplay();
  stopMoving();
}

// a really silly method that just moves the thing left. hopefully. 
void alignLeft(int backDist, int sideDist, int backThresh){
 clearDisplay();
 lcd.write("aligning left...");
 
 int distB = sonarRight.getDist() - sideDist;
 int curDist = sonarBack.getDist() - backDist;
 int ballDist = sonarBack.getDist();
 
 while(distB > 0) {
  delay(updateDelay); 
  distB = sonarRight.getDist() - sideDist; 
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
  int distA = sonarLeft.getDist();
  int distB = sonarRight.getDist();
  
  clearDisplay();
  lcd.write("findCenter()..");
  
  while(abs(distA - distB) > threshold) {
  delay(updateDelay);
    distA = sonarLeft.getDist();
    distB = sonarRight.getDist();
    
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
moveBackward(int speed1){
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
}

void
moveForward(int speed1){
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
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
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
}

void
strafeLeft(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  
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
 // while(!touchSensor.isPressed()){
  //}
  nudgeElevatorDown();
  delay(200);
  elevatorLower();
 // while(!touchSensor.isPressed()){
  //}
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
void elevatorRaise() {
 digitalWrite(q1, LOW);
 digitalWrite(q2, HIGH);
 digitalWrite(q3, HIGH);
 digitalWrite(q4, LOW);
}

void elevatorLower() {
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

void elevatorBrake() {
 digitalWrite(q1, HIGH);
 digitalWrite(q2, LOW);
 digitalWrite(q3, HIGH);
 digitalWrite(q4, LOW);
}

/*
void elevatorLower(int speed1, long rotations) {
  mmx.runRotations(MMX_Motor_1, MMX_Direction_Forward, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float);
  mmx.runRotations(MMX_Motor_2, MMX_Direction_Reverse, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float); 
}

void elevatorRaise(int speed1, long rotations) {
  mmx.runRotations(MMX_Motor_1, MMX_Direction_Reverse, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float);
  mmx.runRotations(MMX_Motor_2, MMX_Direction_Forward, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float); 
}

void elevatorPassiveStop() {
  mmx.runSeconds(MMX_Motor_Both, MMX_Direction_Reverse, 0, 0, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
  mmx.runSeconds(MMX_Motor_Both, MMX_Direction_Reverse, 0, 0, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
}
*/

// DEPRECATED DO NOT USE WITH CURRENT H-BRIDGE SETUP.
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
