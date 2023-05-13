#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#define CE 9
#define CSN 10
RF24 radio (CE, CSN);
const byte address[6] = "00001";

int dato1 = 1;
int dato2 = 2;

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  radio.write(&dato1,sizeof(dato1));
  radio.write(&dato2,sizeof(dato2));
  delay(200);
}
