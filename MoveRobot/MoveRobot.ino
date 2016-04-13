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
const int UPDATE_DELAY = 50;

// The shield
NXShield nxshield;

//
// Declare the i2c devices used on NXShield(s).
//
NXTMMX      mmx(0x0A); // Multiplexer for chain morors

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

  // TODO: Hook up the big red go button to use in place of this.
  nxshield.waitForButtonPress(BTN_GO);

  nxshield.bank_a.motorReset();
  nxshield.bank_b.motorReset();

  //
  // Initialize the i2c sensors.
  //

  // sensor diagnostic mode.
  if(nxshield.getButtonState(BTN_RIGHT)){
     clearDisplay();
     beep();
     lcd.print("Diagnostic Mode!");
     delay(5000);

     while(true){
       delay(UPDATE_DELAY);

     }

  } // end diagnostics mode


  clearDisplay();
} // end setup

///
/////////// LOOP ///////////////////////////////////////////////
///

void loop(){
  if(Serial.available()) {
    d = Serial.read(); // reads in a char from serial.

    // TODO: process commands from Pi.

  }

	motorTest();
}


///
/////////// Helper Functions ///////////////////////////////////////////////
///

void motorTest(){
	leftDrive(1, 50, 720);
	rightDrive(1, 50, 720);
	delay(2000);
	leftDrive(0, 50, 720);
	rightDrive(0, 50, 720);
}

void breakTime(){
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
leftDrive(int direction, int speed, long deg){
	if(direction){
  		nxshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, speed, deg, SH_Completion_Dont_Wait);
  		nxshield.bank_b.motorRunDegrees(SH_Motor_2, SH_Direction_Forward, speed, deg, SH_Completion_Dont_Wait);
	} else {
  		nxshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Forward, speed, deg, SH_Completion_Dont_Wait);
  		nxshield.bank_b.motorRunDegrees(SH_Motor_2, SH_Direction_Reverse, speed, deg, SH_Completion_Dont_Wait);
	}
}

void
rightDrive(int direction, int speed, long deg){
	if(direction){
  		mmx.runDegrees(MMX_Motor_1, MMX_Direction_Forward, speed, deg, MMX_Completion_Dont_Wait);
  		mmx.runDegrees(MMX_Motor_2, MMX_Direction_Reverse, speed, deg, MMX_Completion_Dont_Wait);
	} else {
  		mmx.runDegrees(MMX_Motor_1, MMX_Direction_Reverse, speed, deg, MMX_Completion_Dont_Wait);
  		mmx.runDegrees(MMX_Motor_2, MMX_Direction_Forward, speed, deg, MMX_Completion_Dont_Wait);
	}
}

void rotate(int direction, int speed, long deg){
  if(direction){ // CW
    leftDrive(true, speed, deg);
    rightDrive(false, speed, deg);
  } else { // CCW
    leftDrive(false, direction, speed, deg);
    rightDrive(true, direction, speed, deg);
  }
}

void
forward(int speed, long deg){
  mmx.runDegrees(MMX_Motor_1, MMX_Direction_Forward, speed, deg, MMX_Completion_Dont_Wait);
  mmx.runDegrees(MMX_Motor_2, MMX_Direction_Reverse, speed, deg, MMX_Completion_Dont_Wait);
  nxshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, speed, deg, SH_Completion_Dont_Wait);
  nxshield.bank_b.motorRunDegrees(SH_Motor_2, SH_Direction_Forward, speed, deg, SH_Completion_Dont_Wait);
}

void reverse(int speed, long deg){
  mmx.runDegrees(MMX_Motor_1, MMX_Direction_Reverse, speed, deg, MMX_Completion_Dont_Wait);
  mmx.runDegrees(MMX_Motor_2, MMX_Direction_Forward, speed, deg, MMX_Completion_Dont_Wait);
  nxshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Forward, speed, deg, SH_Completion_Dont_Wait);
  nxshield.bank_b.motorRunDegrees(SH_Motor_2, SH_Direction_Reverse, speed, deg, SH_Completion_Dont_Wait);
}

// Forces all motors to stop.
void
stopMoving(){
  mmx.runSeconds(MMX_Motor_Both, MMX_Direction_Reverse, 0, 0, MMX_Completion_Dont_Wait, SH_Next_Action_Brake);
  nxshield.bank_b.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
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
