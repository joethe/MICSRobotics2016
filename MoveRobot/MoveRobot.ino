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
#include <NXTMMX.h>s

#include <SoftwareSerial.h>
//Don't make # symbols

//
// Config Values
//
const int HOOP_DIST = 50;
const int UPDATE_DELAY = 50;
const int SCAN_SPEED = 50;
const int SCAN_SIDE_DIST = 7;
const int BACK_DIST = 40;
const int BACK_THRESH = 3;

// The shield
NXShield nxshield;

//
// Declare the i2c devices used on NXShield(s).
//
NXTUS       sonarLeft; // left
NXTUS       sonarRight; // right
NXTUS       sonarBack;
NXTMMX      mmx(0x0A); // Multiplexer for chain morors
NXTTouch    endStop; // used for chain endstops

// 2 line LCD on arduino digital pin #6
SoftwareSerial lcd(2, 5);


///
/////////// SETUP /////////////////////////////////////////////// 
///

void setup(){
  Serial.begin(115200);
  
  lcd.begin(9600); 
  lcd.print("Setup Starting...");
  delay(500);
  
  pinMode(6, OUTPUT);
  
  nxshield.init(SH_HardwareI2C);
  
  // Check battery voltage on startup. Warn if low.
  float batVolt = (float) nxshield.bank_a.nxshieldGetBatteryVoltage() / 1000;
  if(batVolt < 7.50) {
    clearDisplay();
    beep();
    Serial.println("Low Voltage!");
    Serial.println(batVolt);
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
  endStop.init( &nxshield, SH_BAS1 );
  
  // sensor diagnostic mode.
  if(nxshield.getButtonState(BTN_RIGHT)){
     clearDisplay();
     beep();
     lcd.print("Diagnostic Mode!");
     delay(5000);
     int backUS = sonarBack.getDist();
     int rightUS = sonarRight.getDist();
     int leftUS = sonarLeft.getDist();
     //boolean endStops = touchSensor.isPressed();
     while(true){
       delay(UPDATE_DELAY);
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
         elevatorRaise(100);
       }else if(nxshield.getButtonState(BTN_LEFT)){
         elevatorLower(100);
       }else {
         elevatorPassiveStop();
       } 
     }
    
  } // end diagnostics mode
  
  moveForward(50);
  delay(500);
  stopMoving();
  
  elevatorRaise_Degrees(30, 180);

   findCenter(2);
   approachHoop();
   takeAShot();
 
  clearDisplay();
} // end setup

///
/////////// LOOP /////////////////////////////////////////////// 
///

void loop(){
  scan();
}


///
/////////// Helper Functions /////////////////////////////////////////////// 
///

// dist - desired distance (~cm) from the back wall of the field.
// backThresh - the threshold (~cm) before the robot will attempt to correct for drift.
// ballThresh - the threshold (~cm) of sudden distance change that will trigger a ball found state.
// sideDist - the distance (~cm) from the sides of the field that the robot will maintain while scanning.

void scan() {
  int backDist = sonarBack.getDist();
  int distSum = 0;
  int sensorCount = 0;
  int scanCount = 0;
  boolean  foundBall = false;
  //elevatorReadyPosition();
  while(!foundBall){
    alignLeft(SCAN_SIDE_DIST, BACK_THRESH);
    distSum = 0;
    strafeRight(SCAN_SPEED);
   
    while(sonarLeft.getDist() > SCAN_SIDE_DIST){
      delay(UPDATE_DELAY);
      clearDisplay();
            
      backDist = sonarBack.getDist();
      distSum += backDist;
      
      lcd.print("scanning...");
      setLCDCursor(16);
      lcd.print(backDist);
      
      if(backDist > 105 || backDist < 10) {
        foundBall = true;
        stopMoving();
        grabBall(50);
        approachHoop();
       takeAShot();
        
        break;
      }
    } //end strafe while
    
    if(!foundBall) {
      delay(900);
      stopMoving();
      approachBackWall(BACK_DIST, BACK_THRESH);
    }
  } //end find ball while
  //void alignLeft(int backDist, int sideDist, int backThresh);
}



// Moves forward and grabs a ball (assumes ball is there...)
void grabBall(int speed) {
 
 clearDisplay();
 lcd.print("grabBall()...");
 
 stopMoving();
 elevatorToFloor();
 delay(100);
 moveBackward(speed);
 delay(2300);
 stopMoving();
 delay(100);
 moveForward(20);
 delay(500);
 elevatorRaise_Degrees(20, 180);////
 delay(1000);//
 moveBackward(30);
 delay(1500);
 moveForward(50);
 delay(1500);
 stopMoving();

 clearDisplay(); 
 }

// Takes a shot./////////////////////////////////////////////////////////////////////////////////////
void takeAShot(){
  
  clearDisplay();
  lcd.print("Taking the shot!");
  
  beep();
  stopMoving();
  elevatorRaise(100);
  while(!endStop.isPressed()){};
  elevatorReadyPosition();
  
  clearDisplay();
}

// moves the bot to within dist cm, give or take thresh cm, of the back wall
void approachBackWall(int dist, int thresh) {
  
  clearDisplay();
  lcd.print("approching back wall...");
 
  int curDist = sonarBack.getDist() - dist;
  
  while(abs(curDist) > thresh)  {
    delay(UPDATE_DELAY);
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
  findCenter(2);
  clearDisplay();
  lcd.write("approachHoop()");
  
  delay(500);
  
  moveForward(50);
  while(sonarBack.getDist() < HOOP_DIST) { 
  }
  
  clearDisplay();
  stopMoving();
}

// a really silly method that just moves the thing left. hopefully. 
void alignLeft(int sideDist, int backThresh){
 clearDisplay();
 lcd.write("aligning left...");
 
 int distB = sonarRight.getDist() - sideDist;
 int curDist = sonarBack.getDist() - BACK_DIST;
 int ballDist = sonarBack.getDist();
 
 while(distB > 0) {
  delay(UPDATE_DELAY); 
  distB = sonarRight.getDist() - sideDist; 
  curDist = abs(sonarBack.getDist() - BACK_DIST) ;
  ballDist = sonarBack.getDist();

  strafeLeft(SCAN_SPEED); 
  
  clearDisplay();
  lcd.print("Aligning left...");
  setLCDCursor(16);
  lcd.print("left dist: ");
  lcd.print(distB);
  
    if(ballDist > 100 || ballDist < 10) {
        stopMoving();
        grabBall(40);
        approachHoop();
        takeAShot();
        break;
    }
    /*
  if(curDist > backThresh) {
    //stopMoving();
    clearDisplay();
    lcd.write("correcting dist. !");
    delay(100);
    approachBackWall(BACK_DIST, 3);
  }
  */

 } // end while
 delay(1000);
 stopMoving();
 approachBackWall(BACK_DIST, 3);
 clearDisplay();
}

// centers the robot left / right on the field
void findCenter(int threshold) {
  int distA = sonarLeft.getDist();
  int distB = sonarRight.getDist();
  
  clearDisplay();
  lcd.write("findCenter()...");
  
  while(abs(distA - distB) > threshold) {
  delay(UPDATE_DELAY);
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
      strafeRight(40); 
    } else {
      strafeLeft(40);
    }
  }
  
  clearDisplay();
  stopMoving();
}


void breakTime(){
  findCenter(4);
  approachHoop();
  stopMoving();
  clearDisplay();
  lcd.print("taking a break...");
  delay(2000);
  clearDisplay();
}

//////////////////////////// MISC //////////////////////////////////////

// Activates the noise maker
void beep() {
 digitalWrite(6, HIGH);
 delay(50);
 digitalWrite(6, LOW); 
}

void coloryLights(){
  nxshield.ledSetRGB(8,0,0); // guessing that this is the red pin...
  delay(1000);
  nxshield.ledSetRGB(0,8,0);
  delay(1000);
  nxshield.ledSetRGB(0,0,8);
  delay(1000);
  nxshield.ledSetRGB(0,0,0);
  delay(1000);
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



///////////////////////////// ELEVATOR COMMANDS ////////////////////////////

void elevatorRaise_Degrees(int speed1, long deg) {
  mmx.runDegrees(MMX_Motor_1, MMX_Direction_Forward, speed1, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Float);
  mmx.runDegrees(MMX_Motor_2, MMX_Direction_Reverse, speed1, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Float); 
}

void elevatorLower_Degrees(int speed1, long deg) {
  mmx.runDegrees(MMX_Motor_1, MMX_Direction_Reverse, speed1, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Float);
  mmx.runDegrees(MMX_Motor_2, MMX_Direction_Forward, speed1, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Float); 
}

void elevatorRaise(int speed1){
  mmx.runUnlimited(MMX_Motor_1, MMX_Direction_Forward, speed1); 
  mmx.runUnlimited(MMX_Motor_2, MMX_Direction_Reverse, speed1);
}

void elevatorLower(int speed1){
  mmx.runUnlimited(MMX_Motor_1, MMX_Direction_Reverse, speed1); 
  mmx.runUnlimited(MMX_Motor_2, MMX_Direction_Forward, speed1);
}

void elevatorPassiveStop() {
  mmx.runSeconds(MMX_Motor_Both, MMX_Direction_Reverse, 0, 0, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
  mmx.runSeconds(MMX_Motor_Both, MMX_Direction_Reverse, 0, 0, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
}

void elevatorReadyPosition() {
 elevatorLower(50);
 delay(300);
 while(mmx.getMotorStatusByte(MMX_Motor_1) < 16){
    delay(UPDATE_DELAY);
    clearDisplay();
    lcd.print(mmx.getMotorStatusByte(MMX_Motor_1));
 };
 elevatorPassiveStop();
 elevatorRaise_Degrees(50, 180);
}

void elevatorToFloor() {
 elevatorLower(40);
 delay(100);
 while(mmx.getMotorStatusByte(MMX_Motor_1) < 16){
   delay(UPDATE_DELAY);
   clearDisplay();
   lcd.print(mmx.getMotorStatusByte(MMX_Motor_1));
 }
 elevatorPassiveStop();
}

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
