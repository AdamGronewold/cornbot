#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

NineAxesMotion mySensor;                 //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 8;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream


const byte numChars=32;
char receivedChars[numChars]; //array to store incoming commands from serial
bool newRef=false; //check if command updated

void setup() { //This code is executed once
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  while(!Serial) { //wait for the serial port to initialize
    ;
  }
  resetSensor();
  updateSensorData = true;
}
///////////////////////////////////////////////
void loop() { //This code is looped forever
  receiveData(); //check for received information regarding reset
  interpretReceivedData(); //use the data arriving from serial to reset sensor if commanded
  if (updateSensorData) { //Keep the updating of data as a separate task
    updateSensor();
    updateSensorData = false;
  }
  if ((millis() - lastStreamTime) >= streamPeriod) {
    lastStreamTime = millis();
    serialPrints();
    updateSensorData = true;
  }
}
///////////////////////////////////////////////
void resetSensor() {
  Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired, 9 degrees of freedom sensor fusion
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
}
///////////////////////////////////////////////
void updateSensor() {
  mySensor.updateAccel();        //Update the Accelerometer data
  mySensor.updateLinearAccel();  //Update the Linear Acceleration data
  mySensor.updateGravAccel();    //Update the Gravity Acceleration data
  mySensor.updateEuler();        //Update the Euler data into the structure of the object
  mySensor.updateCalibStatus();  //Update the Calibration Status
}
///////////////////////////////////////////////
void serialPrints() {  
  Serial.print("<"); //required starting char
  //Serial.print("Time: ");
  Serial.print(lastStreamTime);
  Serial.print(", ");
  //Serial.print("ms ");
  //Serial.print(" aX: ");
  Serial.print(mySensor.readAccelerometer(X_AXIS)); //Accelerometer X-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" aY: ");
  Serial.print(mySensor.readAccelerometer(Y_AXIS));  //Accelerometer Y-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" aZ: ");
  Serial.print(mySensor.readAccelerometer(Z_AXIS));  //Accelerometer Z-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" lX: ");
  Serial.print(mySensor.readLinearAcceleration(X_AXIS)); //Linear Acceleration X-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" lY: ");
  Serial.print(mySensor.readLinearAcceleration(Y_AXIS));  //Linear Acceleration Y-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" lZ: ");
  Serial.print(mySensor.readLinearAcceleration(Z_AXIS));  //Linear Acceleration Z-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" gX: ");
  Serial.print(mySensor.readGravAcceleration(X_AXIS)); //Gravity Acceleration X-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" gY: ");
  Serial.print(mySensor.readGravAcceleration(Y_AXIS));  //Gravity Acceleration Y-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" gZ: ");
  Serial.print(mySensor.readGravAcceleration(Z_AXIS));  //Gravity Acceleration Z-Axis data
  Serial.print(", ");
  //Serial.print("m/s2 ");
  //Serial.print(" H: ");
  Serial.print(mySensor.readEulerHeading()); //Heading data
  Serial.print(", ");
  //Serial.print("deg ");
  //Serial.print(" R: ");
  Serial.print(mySensor.readEulerRoll()); //Roll data
  Serial.print(", ");
  //Serial.print("deg");
  //Serial.print(" P: ");
  Serial.print(mySensor.readEulerPitch()); //Pitch data
  Serial.print(", ");
  //Serial.print("deg ");
  //Serial.print(" A: ");
  Serial.print(mySensor.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)
  Serial.print(", ");
  //Serial.print(" M: ");
  Serial.print(mySensor.readMagCalibStatus());    //Magnetometer Calibration Status (0 - 3)
  Serial.print(", ");
  //Serial.print(" G: ");
  Serial.print(mySensor.readGyroCalibStatus());   //Gyroscope Calibration Status (0 - 3)
  Serial.print(", ");
  //Serial.print(" S: ");
  Serial.print(mySensor.readSystemCalibStatus());   //System Calibration Status (0 - 3)
  Serial.print(">"); //required terminating char
  Serial.println();
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
      Wire.end();
      delay(500);
      resetSensor();
    }
    receivedChars[0]='\0'; //reset the received character array
    newRef=false;
  }
}