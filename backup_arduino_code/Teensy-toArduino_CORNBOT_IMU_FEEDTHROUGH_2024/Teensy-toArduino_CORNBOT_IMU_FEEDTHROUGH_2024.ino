//const byte numChars=32;
char receivedChars[200]; //array to store incoming commands from serial
char receivedChars2[10];
bool newRef=false; //check if command updated
bool newRef2=false;

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop() {
  readArduino();
  readMainSerialforReset();
}

void readArduino() {
  static bool recvInProgress=false;
  static byte ndx=0;
  char beginMarker = '<'; //required start Marker for command
  char endMarker = '>'; //required end Marker for command
  char rc; 
  while (Serial1.available() > 0 && newRef == false) {
    rc=Serial1.read();
    if (recvInProgress==true) { //if the user has started entering new command
      if (rc != endMarker) { //and if that last character entered is not the terminating char
        receivedChars[ndx]=rc; //then add the character to our array
        ndx++; //increase the index
        //if (ndx >= numChars) {
        //  ndx = numChars -1;
        //}
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
  if (newRef==true) {
    Serial.println(receivedChars);
    receivedChars[0]='\0'; //reset the received character array
    newRef=false;
  }
}
void readMainSerialforReset() {
  static boolean recvInProgress2=false;
  static byte ndx2=0;
  char beginMarker2 = '<'; //required start Marker for command
  char endMarker2 = '>'; //required end Marker for command
  char rc2; 
  while (Serial.available() > 0 && newRef2 == false) {
    rc2=Serial.read();
    if (recvInProgress2==true) { //if the user has started entering new command
      if (rc2 != endMarker2) { //and if that last character entered is not the terminating char
        receivedChars2[ndx2]=rc2; //then add the character to our array
        ndx2++; //increase the index
        //if (ndx >= numChars) {
        //  ndx = numChars -1;
        //}
      }
      else {
        receivedChars2[ndx2]='\0'; //terminate the string input
        recvInProgress2 = false;
        ndx2=0; //reset to wait for next reference
        newRef2=true; //tell the controller the reference has changed
      }
    }
    else if (rc2==beginMarker2) {
        recvInProgress2=true; //user has started entering new command
    }
  }
  if (newRef2==true) {
    if (strcmp(receivedChars2, "RESET")==0 || strcmp(receivedChars2, "Reset")==0 || strcmp(receivedChars2, "reset")==0 || strcmp(receivedChars2, "r")==0) {
      Serial1.print("<Reset>"); //send the reset command through to the Arduino
    }
    receivedChars2[0]='\0'; //reset the received character array
    newRef2=false;
  }
}

