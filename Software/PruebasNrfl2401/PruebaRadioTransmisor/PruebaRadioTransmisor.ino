#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#define CE 9
#define CSN 10
RF24 radio (CE, CSN);
const byte address[6] = "00001";

int dato1 = 0;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  
}

void loop() {
  bool ok = radio.write(dato1,sizeof(dato1));
  if(ok){
    Serial.println(dato1);
    dato1 = dato1 + 1;
  }
  else{
     Serial.println("no se ha podido enviar");
  }
  delay(200);
}
