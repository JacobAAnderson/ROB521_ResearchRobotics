//----to receive---------
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
char tempChars[numChars];

boolean newData = false;

//----Hold Data-----------
int dataNumber = 0;             // new for this version
char motor[numChars] = {0};


void setup() {
    Serial3.begin(9600);
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
}

void loop() {
    recvWithEndMarker();
}

void recvWithEndMarker() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '$';
    char endMarker = '#';
    char rc;
  
  //Serial[number] for mega
    while (Serial3.available() > 0 && newData == false) {
        rc = Serial3.read();

        if (recvInProgress == true) {
            if (rc != endMarker && rc != startMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;

                strcpy(tempChars, receivedChars);
                char * strtokIndx; // this is used by strtok() as an index

                //-----transfer shit--------
                strtokIndx = strtok(tempChars,",");      // get the first part - the string
                strcpy(motor, strtokIndx); // copy it to motor var

                dataNumber = 0;
                strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
                dataNumber = atoi(strtokIndx);     // convert this part to an integer

                showNewNumber();
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
  
}

void showNewNumber() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        Serial.print("Data as Number ... ");    // new for this version
        Serial.println(dataNumber);     // new for this version
        newData = false;
    }
}
