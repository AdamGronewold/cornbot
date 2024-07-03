//Motor Driver Code for Teensy 4.1
//Use /dev/ttyACM0 Serial (Teensy 4.1)

#include <SPI.h>
#define CNTR B00100000
#define CLR B00000000

//vehicle parameters
const float wheel_r=0.1092; //wheel radius
const float counts_per_rev=100000; //counts per revolution of wheel //~100000 ticks per revolution, with pwn 255, max is about 1.7 rev/sec, ~102 rpm, 1.2 m/s

//reference linear velocity variables in m/s
static float vref_left; //left side reference speed
static float vref_right; //right side reference speed
static float wref_left;
static float wref_right;
const byte numChars=32;
char receivedChars[numChars]; //array to store incoming reference speeds from serial
boolean newRef=false; //check if ref speeds updated

const float command_lim=255; //pwm command limit

// Motor A connections FL
const int enA = 2; //PWM
const int in1 = 3; //High/low
const int in2 = 4; //High/low
// // Motor B connections BL
const int enB = 5; //PWM
const int in3 = 6; //High/low
const int in4 = 7; //High/low
// // Motor C connections FR
const int enC = 8; //PWM
const int in5 = 9; //High/low
const int in6 = 20; //High/low
// // Motor B connections BR
const int enD = 19; //PWM
const int in7 = 22; //High/low
const int in8 = 23; //High/low

//pins 10, 11, 12, 13 reserved for SPI

// 3-Encoder board pin mapping to available pins on teensy
const int chipSelectPin1=14; //labelled 10 on encoder count shield
const int chipSelectPin2=15; //labelled 9 on encoder count shield
const int chipSelectPin3=16; //labelled 8 on encoder count shield

//Gains for difference equation of speed control
const float K1=1;
const float K2=6.24; //Kp~=K2-mean(K2,K3), Ki~=mean(K2,K3)
const float K3=-4.78;

//function declarations
// long getEncoderValue(int);
// void CheckSpeeds();
// void CommandUpdates();
// void Increment();

//variables for difference equation
long last_left_ticks; //last left count from encoder
long last_right_ticks; //last right count from right encoder
long current_left_ticks; //at current time step
long current_right_ticks; //right tick counts at current time step
long zeroith_left_ticks; //starting value on left encoder
long zeroith_right_ticks; //starting value on right encoder

float rpm_left; //output RPM on left side
float rpm_right; //output RPM on right side
float er_left; //current error on left side in RPM
float er_right; //current error on right side speed in RPM
float last_er_left; //last error on left side in RPM
float last_er_right; //last error on right side in RPM
float uk_left; //current command to left motors
float uk_right; //current comand to right motors
float uk_left_con; //current command after constraining
float uk_right_con; //current right command after constraining
float last_uk_left; //last command to left motor controller
float last_uk_right; //last command to right motor controller

//timer variables
const float T=8000; //stream period; microseconds
unsigned long lastStreamTime=0;
static int count=0;
//const int count_lim=100000000;

//counts to RPM constant to avoid multiple floating calcs
float C=(60*1000000.0)/(counts_per_rev*T);


//__________________________________________________________
void setup() {

  //begin serial for encoder counter board
  Serial.begin(9600);

  while(!Serial) { //wait for serial ports to initialize
    ;
  }
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  
  //Set encoder board pins to outputs
  pinMode(chipSelectPin1, OUTPUT);
  pinMode(chipSelectPin2, OUTPUT);
  pinMode(chipSelectPin3, OUTPUT);
  digitalWrite(chipSelectPin1, HIGH);
  digitalWrite(chipSelectPin2, HIGH);
  digitalWrite(chipSelectPin3, HIGH);

  //initialize the register encoder counter chip
  LS7366_Init();
  delay(100);

  // Turn off motors - Initial state
  turn_off();

  //resetting the counter values inside the encoder board to 0;
  resetEncoder(1);
  resetEncoder(2);
  resetEncoder(3);

  last_left_ticks=getEncoderValue(1); //last left count from encoder
  last_right_ticks=getEncoderValue(2); //last right count from right encoder
  zeroith_left_ticks=getEncoderValue(1); //starting value on left encoder
  zeroith_right_ticks=getEncoderValue(2); //starting value on right encoder
  last_er_left=0;
  last_er_right=0;
  last_uk_left=0;
  last_uk_right=0;
  
  // Serial.print("Last left: ");
  // Serial.println(last_left_ticks);
  // Serial.print("Last right: ");
  // Serial.println(last_right_ticks);
  //   Serial.print("Zeroith left: ");
  // Serial.println(zeroith_left_ticks);
  //   Serial.print("Zeroith right: ");
  // Serial.println(zeroith_right_ticks);
  //   Serial.print("Last left error: ");
  // Serial.println(last_er_left);
  //   Serial.print("Last right error: ");
  // Serial.println(last_er_right);
  //   Serial.print("Last command left: ");
  // Serial.println(last_uk_left);
  //   Serial.print("Last command right: ");
  // Serial.println(last_uk_right);
  // Serial.println();
  // Serial.println("Enter 'G' to continue");
  // while(true) // remain here until told to break
  // {
  // if(Serial.available() > 0) // did something come in?
  //   if(Serial.read() == 'G') // is that something the char G?
  //     break;
  // }
  Serial.println();
  Serial.println();
  Serial.println("Setup complete. Enter new reference speed values into serial monitor\nas <left, right> in m/s (no more than 1.2 m/s).");
} 

//__________________________________________________________
void loop() {
  if ((micros()-lastStreamTime)>T) { //if the time that has passed is larger than the stream period
    lastStreamTime=micros();
    recvRef(); //input reference speed buffer function
    modifyRefSpeeds(); //if new reference received
    checkSpeeds(); //check current speed
    commandUpdates(); //change commands
    if(count%200==0) {
      //debugPrints();
    }
    incrementValues();
    count++;
  }
}

  // static int count=0;
  // time1=micros();
  
  // while (count<count_lim) { 
  //   time2=micros();
  //   if ((time2-time1)>T) { //if the time that has passed is larger than the stream period
  //     time1=time2;
  //     recvRef(); //input reference speed buffer function
  //     modifyRefSpeeds(); //if new reference received
  //     checkSpeeds(); //check current speed
  //     commandUpdates(); //change commands
  //     if(count%200==0) {
  //       //debugPrints();
  //     }

  //     incrementValues();
  //     count++;  
  //   }
  // }
  //   long encoder1Value;
  //   long encoder2Value;
  //   long encoder3Value;
  //   encoder1Value = getEncoderValue(1);  
  //   Serial.print("Encoder X= ");
  //   Serial.print(encoder1Value);

  //   encoder2Value = getEncoderValue(2);  
  //   Serial.print(" Encoder Y= ");
  //   Serial.print(encoder2Value);

  //   encoder3Value = getEncoderValue(2);  
  //   Serial.print(" Encoder Z= ");
  //   Serial.print(encoder3Value);

  //   Serial.print("\r\n");

  //   delay(100); 
  //   count=count+100;
  //   forward(200, 200);
  //   if (encoder1Value>220000) {
  //    // turn_off();
  //     delay(1000);
  //   }
  // }
  //turn_off();
  

  //turn_off();
  //delay(10000);
  //reverse();
  //delay(1000);

//__________________________________________________________
void recvRef() { //receive reference speed buffer function
  static boolean recvInProgress=false;
  static byte ndx =0;
  char beginMarker = '<'; //required start Marker for new speed reference
  char endMarker = '>'; //required end Marker for new speed reference as <x, y> where x and y are linear speeds < 1.2 m/s
  char rc; 
  while (Serial.available() > 0 && newRef == false) {
    rc=Serial.read();
    if (recvInProgress==true) { //if the user has started entering new command
      if (rc != endMarker) { //if that last character entered is not the terminating char
        receivedChars[ndx]=rc; //add the character to our array
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars -1;
        }
      }
      else {
        receivedChars[ndx]='\0'; //terminate the string input
        recvInProgress = false;
        ndx=0; //reset to wait for next reference
        newRef=true; //tell the controller the reference has changed
      }
    }

    else if (rc==beginMarker) {
        recvInProgress=true; //user has started entering new reference value
    }
  }
}

//__________________________________________________________
void modifyRefSpeeds() { //modify reference if a new set of values has been entered
  if (newRef==true) {
    //Serial.println(receivedChars);
    const char delim[3]=", ";
    char *token;

    //get the first token/reference, set as left reference
    token=strtok(receivedChars, delim);
    vref_left=atof(token);
    //second token becomes second reference
    token = strtok(NULL, delim);
    vref_right=atof(token);

    wref_left=(vref_left/wheel_r)/(2*3.14159/60); //reference in RPM
    wref_right=(vref_right/wheel_r)/(2*3.14159/60); //right reference in RPM

    /*Serial.print("Reference Speeds Updated to: <");
    Serial.print(vref_left);
    Serial.print(", ");
    Serial.print(vref_right);
    Serial.println(">");*/

    /*Serial.print("Reference Speeds In RPM: <");
    Serial.print(wref_left);
    Serial.print(", ");
    Serial.print(wref_right);
    Serial.println(">");*/


    newRef=false;
  }
}

//__________________________________________________________
void checkSpeeds() 
{
  current_left_ticks=getEncoderValue(1)-zeroith_left_ticks; //at current time step
  current_right_ticks=getEncoderValue(2)-zeroith_right_ticks; //right tick counts at current time step
  rpm_left=(current_left_ticks-last_left_ticks)*C;
  rpm_right=-(current_right_ticks-last_right_ticks)*C;
  
  //Output to serial console to be picked up and published by ROS system
  
  
  Serial.print(-rpm_right); //on ROS side this is left
  Serial.print(", ");
  Serial.println(rpm_left); //on ROS side this is right //sent over serial to ros node
  
}

//__________________________________________________________
void commandUpdates() 
{ 
  er_left=wref_left-rpm_left;
  er_right=wref_right-rpm_right;
   
  //difference equations
  uk_left=  K1*last_uk_left  +  K2*er_left  +  K3*last_er_left; //K2*er_FL;//
  uk_right=  K1*last_uk_right  +  K2*er_right  +  K3*last_er_right; //K2*er_FR;//
  
  uk_left=constrain(uk_left,-command_lim, command_lim); //constraining to only PWM values
  uk_right=constrain(uk_right,-command_lim, command_lim);
  
  if (wref_left!=0 || wref_right!=0) {
    unsignedDirectional(uk_left, uk_right); //send command update to motor driver
  }
  else if (wref_left==0 && wref_right==0) {
    turn_off();
  }
}

//__________________________________________________________
void incrementValues() 
{
  last_uk_left=uk_left;
  last_uk_right=uk_right;
  last_er_left=er_left;
  last_er_right=er_right;
  last_left_ticks=current_left_ticks;
  last_right_ticks=current_right_ticks;
}

//__________________________________________________________
void debugPrints() 
{
  Serial.print(rpm_left);
  Serial.print("    ");
  Serial.print(rpm_right);
  Serial.print("    ");
  Serial.print(rpm_left*(2*3.14159/60)*wheel_r);
  Serial.print("    ");
  Serial.print(rpm_right*(2*3.14159/60)*wheel_r);
  Serial.print("    ");
  Serial.print(er_left);  
  Serial.print("    ");
  Serial.print(er_right); 
  Serial.print("    ");
  Serial.print(current_left_ticks);
  Serial.print("    ");
  Serial.print(current_right_ticks);
  Serial.print("    ");
  Serial.print(uk_left); 
  Serial.print("    ");
  Serial.println(uk_right);  
}

//__________________________________________________________
void turn_off() {
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
}

//__________________________________________________________
void forward(int left, int right) {
  //left side motors forward
  digitalWrite(in1, HIGH); //forward
  digitalWrite(in2, LOW); //forward
  digitalWrite(in3, LOW); //forward
  digitalWrite(in4, HIGH); //forward


  //right side motors forward
  digitalWrite(in5, LOW); //forward
  digitalWrite(in6, HIGH); //forward
  digitalWrite(in7, HIGH); //forward
  digitalWrite(in8, LOW); //forward

  // for (int i = 0; i < 254; i++) {
  //   analogWrite(enA, i);
  //   analogWrite(enB, i);
  //   analogWrite(enC, i);
  //   analogWrite(enD, i);
  //   delay(20);
  // }

  analogWrite(enA, left); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255. FL
  analogWrite(enB, left); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255. BL
  analogWrite(enC, right); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255. FR
  analogWrite(enD, right); //pin 3 is PWM, 178/255 = (about) 70% speed. Max is 255. BR
}

//__________________________________________________________
void reverse(int left, int right) {
  //left side motors reverse
  digitalWrite(in1, LOW); //reverse
  digitalWrite(in2, HIGH); //reverse
  digitalWrite(in3, HIGH); //reverse
  digitalWrite(in4, LOW); //reverse

  //right side motors reverse
  digitalWrite(in5, HIGH); //reverse
  digitalWrite(in6, LOW); //reverse
  digitalWrite(in7, LOW); //reverse
  digitalWrite(in8, HIGH); //reverse

  analogWrite(enA, left); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. FL
  analogWrite(enB, left); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. FR
  analogWrite(enC, right); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. FR
  analogWrite(enD, right); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. BR
}

void unsignedDirectional(int left, int right) {
  if (left>0) {
    //left side motors forward
    digitalWrite(in1, HIGH); //forward
    digitalWrite(in2, LOW); //forward
    digitalWrite(in3, LOW); //forward
    digitalWrite(in4, HIGH); //forward
  }
  else if (left<0) {
    //left side motors reverse
    digitalWrite(in1, LOW); //reverse
    digitalWrite(in2, HIGH); //reverse
    digitalWrite(in3, HIGH); //reverse
    digitalWrite(in4, LOW); //reverse
  }
  else if (left==0) { //off
      // Turn off motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  if (right>0) {
    //right side motors forward
    digitalWrite(in5, LOW); //forward
    digitalWrite(in6, HIGH); //forward
    digitalWrite(in7, HIGH); //forward
    digitalWrite(in8, LOW); //forward
  }
  else if (right<0) {
      //right side motors reverse
    digitalWrite(in5, HIGH); //reverse
    digitalWrite(in6, LOW); //reverse
    digitalWrite(in7, LOW); //reverse
    digitalWrite(in8, HIGH); //reverse
  }
  else if (right==0) { //off
    digitalWrite(in5, LOW);
    digitalWrite(in6, LOW);
    digitalWrite(in7, LOW);
    digitalWrite(in8, LOW);
  }

  left=abs(left);
  //send to both motors on tank steer
  analogWrite(enA, left); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. FL
  analogWrite(enB, left); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. FR

  right=abs(right);
  analogWrite(enC, right); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. FR
  analogWrite(enD, right); //pin 3 is PWM, 76/255 = (about) 30% speed. Max is 255. BR
}
//__________________________________________________________
long getEncoderValue(int encoder)
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;

    selectEncoder(encoder);

    SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte

    deselectEncoder(encoder);

    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;

    return result;
}//end func

//__________________________________________________________
void selectEncoder(int encoder)
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,LOW);
        break;
     case 2:
       digitalWrite(chipSelectPin2,LOW);
       break;
     case 3:
       digitalWrite(chipSelectPin3,LOW);
       break;    
  }//end switch
}//end func

//__________________________________________________________
void resetEncoder(int encoder) {
    selectEncoder(encoder);
    SPI.transfer(CLR | CNTR);
    deselectEncoder(encoder);
} //end func

//__________________________________________________________
void deselectEncoder(int encoder)
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,HIGH);
        break;
     case 2:
       digitalWrite(chipSelectPin2,HIGH);
       break;
     case 3:
       digitalWrite(chipSelectPin3,HIGH);
       break;    
  }//end switch
}//end func

//__________________________________________________________
// LS7366 Initialization and configuration
void LS7366_Init(void)
{

    // SPI initialization
    SPI.begin(); //sets SCK, MOSI, and SS to outputs; pulls SCK and MOSI low, SS high, pins 10-13 on Teensy
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);

   digitalWrite(chipSelectPin1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin1,HIGH); 

   digitalWrite(chipSelectPin2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin2,HIGH); 

   digitalWrite(chipSelectPin3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin3,HIGH); 

}//end func
