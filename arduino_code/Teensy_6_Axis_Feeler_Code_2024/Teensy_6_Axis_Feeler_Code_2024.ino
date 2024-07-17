//Feeler Driver for 
//Teensy 4.1 (use /dev/ttyACM1 on ROS side)

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#define CNTR B00100000
#define CLR B00000000

//the lines are used by 74HC138 chip to select the cable select lines
int chipSelectPin1 = 14;//C  A2 //labelled 10 on encoder count shield
int chipSelectPin2 = 15; //B  A1  //labelled 9 on encoder count shield
int chipSelectPin3 = 16; //A  A0 //labelled 8 on encoder count shield

float C=(90.00/5000); //encoder ticks to radians to degrees //CPR=5000 TPR=20000
long encoder1Value;
long encoder2Value;
long encoder3Value;
long encoder4Value;
long encoder5Value;
long encoder6Value;
long encoder1zero;
long encoder2zero;
long encoder3zero;
long encoder4zero;
long encoder5zero;
long encoder6zero;

unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 8;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))

const byte numChars=32;
char receivedChars[numChars]; //array to store incoming commands from serial
boolean newRef=false; //check if command updated

//////////////////////////////////////////////
void setup() {
    Serial.begin(9600); //Initialize the Serial Port to view information on the Serial Monitor

    while(!Serial) { //wait for the serial port to initialize
      ;
    }
    pinMode(chipSelectPin1, OUTPUT);
    pinMode(chipSelectPin2, OUTPUT);
    pinMode(chipSelectPin3, OUTPUT);

    digitalWrite(chipSelectPin1, HIGH);
    digitalWrite(chipSelectPin2, HIGH);
    digitalWrite(chipSelectPin3, HIGH);

   
    LS7366_Init(); //initialize the register in all of the 6 chips

    delay(100);

    
    resetEncoder(1); //reseting the counter value inside the encoder chips to 0
    resetEncoder(2);
    resetEncoder(3);
    resetEncoder(4);
    resetEncoder(5);
    resetEncoder(6);

    encoder1zero=getEncoderValue(1);
    encoder2zero=getEncoderValue(2);
    encoder3zero=getEncoderValue(3);
    encoder4zero=getEncoderValue(4);
    encoder5zero=getEncoderValue(5);
    encoder6zero=getEncoderValue(6);
} //end func

//////////////////////////////////////////////
void loop() {
  if ((millis() - lastStreamTime) >= streamPeriod) {
    lastStreamTime = millis();
    measure(); //get encoder values
    sendData(); //send the measurements
    receiveData(); //check for received information
    interpretReceivedData(); //use the data arriving from serial
  }
} //end loop
 //////////////////////////////////////////////
void measure() {
  encoder1Value = (getEncoderValue(1)-encoder1zero); 
  encoder2Value = (getEncoderValue(2)-encoder2zero);  
  encoder3Value = (getEncoderValue(3)-encoder3zero); 
  encoder4Value = (getEncoderValue(4)-encoder4zero);  
  encoder5Value = (getEncoderValue(5)-encoder5zero);  
  encoder6Value = (getEncoderValue(6)-encoder6zero); 
}
///////////////////////////////////////////////
void sendData() {
  Serial.print("FL = ");
  Serial.print(C*encoder1Value);
  Serial.print(", FR = ");
  Serial.print(C*encoder2Value);
  Serial.print(", BL = ");
  Serial.print(C*encoder3Value);
  Serial.print(", BR = ");
  Serial.println(C*encoder4Value);
  //Serial.print(", E5 = ");
  //Serial.print(C*encoder5Value);
  //Serial.print(", E6 = ");
  //Serial.print(C*encoder6Value);        
}
///////////////////////////////////////////////
void receiveData() {
  static boolean recvInProgress=false;
  static byte ndx=0;
  char beginMarker = '<'; //required start Marker for command
  char endMarker = '>'; //required end Marker for command
  char rc; 
  while (Serial.available() > 0 && newRef == false) {
    rc=Serial.read();
    if (recvInProgress==true) { //if the user has started entering new command
      if (rc != endMarker) { //and if that last character entered is not the terminating char
        receivedChars[ndx]=rc; //then add the character to our array
        ndx++; //increase the index
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
        recvInProgress=true; //user has started entering new command
    }
  }
}
///////////////////////////////////////////////
void interpretReceivedData() {
  if (newRef==true) {
    if (strcmp(receivedChars, "RESET")==0 || strcmp(receivedChars, "Reset")==0 || strcmp(receivedChars, "reset")==0 || strcmp(receivedChars, "r")==0) {
      resetEncoder(1); //reseting the counter value inside the encoder chips to 0
      resetEncoder(2);
      resetEncoder(3);
      resetEncoder(4);
      resetEncoder(5);
      resetEncoder(6);
      encoder1zero=getEncoderValue(1);
      encoder2zero=getEncoderValue(2);
      encoder3zero=getEncoderValue(3);
      encoder4zero=getEncoderValue(4);
      encoder5zero=getEncoderValue(5);
      encoder6zero=getEncoderValue(6);
      //Serial.println(receivedChars);
    }
    receivedChars[0]='\0'; //reset the received character array
    newRef=false;
  }
}
 //////////////////////////////////////////////
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
    deselectEncoders();
    result = ((long) count1Value << 24) + ((long) count2Value << 16) + ((long) count3Value << 8) + (long) count4Value;
    return result;
} //end func
//////////////////////////////////////////////
void resetEncoder(int encoder) { //this will control the 74HC138 to set the lines low for SPI cable select
    selectEncoder(encoder);
    SPI.transfer(CLR | CNTR);
    deselectEncoders();
} //end func
//////////////////////////////////////////////
void selectEncoder(int encoder) {
   switch (encoder) {
    case 1:
      digitalWrite(chipSelectPin1, LOW); //C A2  //0 A=LOW B=LOW C=LOW
      digitalWrite(chipSelectPin2, LOW); //B A1 
      digitalWrite(chipSelectPin3, LOW); //A A0
      break;
    case 2:
      digitalWrite(chipSelectPin1, LOW); //C   A2//1 A=HIGH B=LOW C=LOW
      digitalWrite(chipSelectPin2, LOW); //B   A1
      digitalWrite(chipSelectPin3, HIGH); //A  A0
      break;
    case 3:
      digitalWrite(chipSelectPin1, LOW); //C   A2  A=LOW B=HIGH C=LOW
      digitalWrite(chipSelectPin2, HIGH); //B  A1
      digitalWrite(chipSelectPin3, LOW); //A   A0
      break;
    case 4:
      digitalWrite(chipSelectPin1, LOW); //C  A2  A=LOW B=HIGH C=HIGH
      digitalWrite(chipSelectPin2, HIGH); //B  A1
      digitalWrite(chipSelectPin3, HIGH); //A  A0
      break;
    case 5:
      digitalWrite(chipSelectPin1, HIGH); //C  //4 A2  A=HIGH B=LOW C=LOW 
      digitalWrite(chipSelectPin2, LOW); //B       A1
      digitalWrite(chipSelectPin3, LOW); //A       A0
      break;
    case 6:
      digitalWrite(chipSelectPin1, HIGH); //C  //5  A2 A=HIGH B=LOW C=HIGH 
      digitalWrite(chipSelectPin2, LOW); //B        A1
      digitalWrite(chipSelectPin3, HIGH); //A       A0 
      break;
  } //end switch
} //end func
//////////////////////////////////////////////
void deselectEncoders() //select the 8th output low which puts all the cable select lines for the 6 encoder chips high and disables the communication
{
    digitalWrite(chipSelectPin1, HIGH);
    digitalWrite(chipSelectPin2, HIGH);
    digitalWrite(chipSelectPin3, HIGH);
} //end func
//////////////////////////////////////////////
void LS7366_Init(void) // LS7366 Initialization and configuration
{
    int a = 0;
    SPI.begin(); // SPI initialization
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);

    for (a = 1; a < 7; a++) { //initialize the 6 
      selectEncoder(a);
      SPI.transfer(0x88);
      SPI.transfer(0x03);
      deselectEncoders();
    }
} //end func
//////////////////////////////////////////////