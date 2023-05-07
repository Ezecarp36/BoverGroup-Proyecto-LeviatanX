#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#define CE 8
#define CSN 9
RF24 radio (CE, CSN);
const uint64_t canal = 0xE8E8F0F0E1LL;

int dato1 = 1;
int dato2 = 2;

void setup() {
  radio.begin();
  radio.openWritingPipe(canal);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  radio.write(&dato1,sizeof(dato1));
  radio.write(&dato2,sizeof(dato2));
  delay(200);
}
