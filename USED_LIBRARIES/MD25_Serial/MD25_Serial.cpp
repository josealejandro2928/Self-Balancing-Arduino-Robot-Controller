#include "MD25_Serial.h"

MD25_Serial::MD25_Serial(int baudrate,int mode,bool autoregulation){
    Serial1.end();
    Serial1.begin(baudrate);

}
/*void MD25_Serial::setMode(int mode){
    Serial1.write()
}*/