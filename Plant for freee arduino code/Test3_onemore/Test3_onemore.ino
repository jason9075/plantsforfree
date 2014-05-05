#include <Servo.h> 
#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2); // set the LCD address to 0x27 for a 16 chars and 2 line display
Servo myservo;                    // create servo object to control a servo a maximum of eight servo objects can be created 

#define CM 1 //Centimeter
#define INC 0 //Inch
#define TP 12 //Trig_pin
#define EP 13 //Echo_pin

//BT
#define RxD 2
#define TxD 3
SoftwareSerial blueToothSerial(RxD,TxD);
unsigned char behVal;
unsigned char Receive_val;
unsigned char len;
unsigned char Buffer_temp[4];
unsigned char i;
//BT

//compare variable
int driveMode = 0;
unsigned int middleDistance = 50;
unsigned int leftDistance   = 50;
unsigned int mLeftDistance  = 50;
unsigned int rightDistance  = 50;
unsigned int mRightDistance = 50;
//

//moisture
#define MOISTURE 2
unsigned int moistureVal = 0;
const unsigned int moistureMaxResistor = 700;
const unsigned int moistureMinResistor = 400;
//

//light sensor
#define ADCA0 A0
#define ADCA1 A1
const unsigned int lightResistor = 230;
unsigned char valA0 = 0;
unsigned char valA1 = 0;
unsigned char autoMove = 0;
//

//motor!!!~~~~~~~
const int motorIn1 = 8;
const int motorIn2 = 9;
const int motorIn3 = 10;
const int motorIn4 = 11;
const int checkDelay = 450;
const unsigned char middleAngle     = 80; //( 80 degree is middle )
const unsigned char activeDistance  = 30; 
const unsigned char farDistance     = 50; 
const unsigned char shortDistance   = 10; 
//motor!!!~~~~~~~

void setup() 
{ 
  Serial.begin(9600); // init serial 9600
  
  myservo.attach(7);  // attaches the servo on pin 9 to the servo object 
  pinMode(TP,OUTPUT); // set TP output pin for trigger
  pinMode(EP,INPUT); // set EP input pin for echo
  myservo.write(middleAngle); //set the motor
 
 //LCD
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
//
  
 //motor setup!!!!
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  
 //Set BT
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);   
  Serial.begin(9600);  
  blueToothSerial.begin(9600);
  behVal=0;     
  len=0;
  //Set BT
} 
 
void loop() 
{
  UART_FUN();
  if(driveMode == 0 )
    autoDrive();
  else if(driveMode == 1)
    manualDrive();
}

void showLCD(){
  lcd.clear();
  valA0 = analogRead(ADCA0);
  valA1 = analogRead(ADCA1);
  moistureVal = analogRead(MOISTURE);
  lcd.setCursor(0, 0);
  lcd.print("PhRsis:");
  lcd.print(valA0);
  lcd.print(",");
  lcd.print(valA1);
  lcd.setCursor(0, 1);
  lcd.print("Mos:");
  lcd.print(moistureVal);
  lcd.print(" Sta:");
  if      (  (valA0 < lightResistor && valA1 < lightResistor) &&  (moistureVal < moistureMaxResistor && moistureVal > moistureMinResistor))
    lcd.print("^O^");  // light = good, moisture = good
  else if (  (valA0 < lightResistor && valA1 < lightResistor) && !(moistureVal < moistureMaxResistor && moistureVal > moistureMinResistor))
    lcd.print("X_X");  // light = good, moisture = bad
  else if ( !(valA0 < lightResistor && valA1 < lightResistor) &&  (moistureVal < moistureMaxResistor && moistureVal > moistureMinResistor))
    lcd.print("-.-");  // light = bad, moisture = good
  else if ( !(valA0 < lightResistor && valA1 < lightResistor) && !(moistureVal >= moistureMaxResistor && moistureVal < moistureMinResistor))
    lcd.print("Q_Q");  // light = bad, moisture = bad
}

void UART_FUN()
{
  for(i=0;i<5;i++)
    Buffer_temp[i]=0x0;
    delay(100);
    while(blueToothSerial.available()){                                                
      Receive_val=blueToothSerial.read();
      //Serial.print(Receive_val); 
      // Serial.print("\n"); 
      if(Receive_val==0xAA){
        len=0;
        Buffer_temp[len++]=Receive_val;
      }
      else if(len < 3){
        Buffer_temp[len++]=Receive_val;
      } 
    }   
  //---------------------------- behVal ---------------------------- 
  if(Buffer_temp[0]==0xAA && Buffer_temp[1]==0xBB && Buffer_temp[2]==0xF1){
    behVal=1;
  }
  else if(Buffer_temp[0]==0xAA && Buffer_temp[1]==0xBB && Buffer_temp[2]==0xF2){
    behVal=2;
  }
  else if(Buffer_temp[0]==0xAA && Buffer_temp[1]==0xBB && Buffer_temp[2]==0xF3){
    behVal=3;
  }
  else if(Buffer_temp[0]==0xAA && Buffer_temp[1]==0xBB && Buffer_temp[2]==0xF4){
    behVal=4;
  }
  else if(Buffer_temp[0]==0xAA && Buffer_temp[1]==0xBB && Buffer_temp[2]==0xF5){
    behVal=5;
  }
  //---------------------------- driveMode ---------------------------- 
  else if(Buffer_temp[0]==0xAA && Buffer_temp[1]==0xBB && Buffer_temp[2]==0xA0){
    driveMode = 0;
    goStop();
  }
  else if(Buffer_temp[0]==0xAA && Buffer_temp[1]==0xBB && Buffer_temp[2]==0xA1){
    driveMode = 1;
    goStop();
  }
}

void manualDrive(){
  if(behVal == 1)        
    goLeft(10);           
  else if(behVal == 2)
    goStop();
  else if(behVal == 3)
    goRight(10);
  else if(behVal == 4)
    goForward(10);
  else if(behVal == 5)
    goBack(10);
  if(behVal == 2){
    delay(1000);
    showLCD();
  }
  Serial.print(behVal); 
}

void autoDrive(){
  delay(1000);
  showLCD();
  valA0 = analogRead(ADCA0);
  valA1 = analogRead(ADCA1);
  if(valA0 < lightResistor && valA1 < lightResistor){
    autoMove = 0;
    goStop();
  }
  else 
    autoMove = 1;
  if(autoMove == 1){
    middleDistance = checkDistance(middleAngle);
    if(middleDistance < activeDistance && middleDistance >= shortDistance){                    // have obstacle
      goStop();
      leftDistance   = checkDistance(middleAngle + 65);
      rightDistance  = checkDistance(middleAngle - 65);
      if(leftDistance < activeDistance && rightDistance < activeDistance)  // no way to go
        goTurn();
      else if (leftDistance >= activeDistance && rightDistance < activeDistance)  // go left
        goLeft(1000);
      else if (leftDistance < activeDistance && rightDistance >= activeDistance)  // go right
        goRight(1000);
      else if (leftDistance >= activeDistance && rightDistance >= activeDistance){  //it can go both
        if(leftDistance >= rightDistance)
          goLeft(1000);
        else if(leftDistance < rightDistance)
          goRight(1000);
      }
    }else if(middleDistance < shortDistance){  
      goBack(1500);
      goTurn();
    }else if(middleDistance >= activeDistance){             // no obstacle
      goForward(500);
      if(middleDistance < farDistance ){
        mLeftDistance = checkDistance(middleAngle + 30);
        mRightDistance = checkDistance(middleAngle - 30);
        if(mLeftDistance < farDistance && mRightDistance >=farDistance)      // little obstacle at left
          goRight(300);
        else if(mRightDistance < farDistance && mLeftDistance >=farDistance) // little obstacle at right
          goLeft(300);
      }
    }
  }
}

void goForward(unsigned int delayTime){
 digitalWrite(motorIn1, LOW);
 digitalWrite(motorIn2, HIGH);
 digitalWrite(motorIn3, LOW);
 digitalWrite(motorIn4, HIGH);
 delay(delayTime);
}

void goBack(unsigned int delayTime){
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(delayTime);
}

void goLeft(unsigned int delayTime){
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  delay(delayTime);
}

void goRight(unsigned int delayTime){
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, LOW);
  delay(delayTime);
}

void goStop(){
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, LOW);
}

void goTurn(){
  goBack(500);
  goRight(492);
  goBack(492);
  goRight(492);
  goRight(492);
  goBack(492);
  goRight(492);
  goBack(500);
  goRight(492);
}

unsigned int checkDistance(unsigned char angle){
  myservo.write(angle);      
  delay(checkDelay);
  unsigned int microseconds = TP_init();
  return Distance(microseconds, CM) ; // Calculating the distance
}

unsigned int TP_init(){
  digitalWrite(TP, LOW);
  delayMicroseconds(2);
  digitalWrite(TP, HIGH); // pull the Trig pin to high level for more than 10us impulse
  delayMicroseconds(10);
  digitalWrite(TP, LOW);
  unsigned int microseconds = pulseIn(EP,HIGH); // waits for the pin to go HIGH, and returns the length of the pulse in microseconds
  return microseconds; // return microseconds
}


unsigned int Distance(long time, int flag){
  unsigned int distacne;
  if(flag)
    distacne = time /29 / 2 ; // Distance_CM = ((Duration of high level)*(Sonic :340m/s))/2
  // = ((Duration of high level)*(Sonic :0.034 cm/us))/2
  // = ((Duration of high level)/(Sonic :29.4 cm/us))/2
  else
    distacne = time / 74 / 2; // INC
  return distacne;
}
