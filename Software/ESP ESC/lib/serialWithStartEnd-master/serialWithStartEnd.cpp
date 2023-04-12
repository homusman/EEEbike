#include <Arduino.h>
    
class serialWithStartEnd{
    boolean newData = false;
    boolean recvInProgress = false;

    HardwareSerial * SerialCh;

    byte startMarker;
    byte endMarker;
    byte rc;
    byte * receivedChars;
    byte ndx = 0;
    byte numChars = 0;

    public:
    //init
    serialWithStartEnd(HardwareSerial *ptrSerial, byte arrayIn[], byte arrsize, const byte sm = 0xfe,const byte em = 0xff){
    SerialCh = ptrSerial;
    startMarker = sm;
    endMarker = em;
    receivedChars = arrayIn;
    numChars = arrsize;
    }

    void begin(const int bps = 9600){
        SerialCh->begin(bps);
    }

    //loop to get data out of buffer, ideally run at frequency faster than serial bit rate
    void recvWithStartEndMarkers() {

        while (SerialCh->available() > 0 && newData == false) {
            rc = SerialCh->read();

            if (recvInProgress == true) {
                if (rc != endMarker) {
                    receivedChars[ndx] = rc;
                    ndx++;
                    if (ndx > numChars) {
                        Serial.println("message longer than expected");
                    }
                }
                else {
                    receivedChars[ndx] = '\0'; // terminate the string
                    recvInProgress = false;
                    ndx = 0;
                    newData = true;
                }
            }

            else if (rc == startMarker) {
                recvInProgress = true;
            }
        }
    }

    //call to see if new data
    boolean querynewdata() {
        if (newData == true) {
            return true;
        }
        else{
            return false;
        }
    }

    //will only start overwriting received bytes after calling this
    void finishread(){
        newData = false;
    }
};