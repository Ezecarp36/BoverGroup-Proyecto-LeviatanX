#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#define CE 32
#define CSN 21
RF24 radio (CE, CSN);
#define TICK_PRINT 500
unsigned long currentTime = 0;

const uint64_t canal = 0xE8E8F0F0E1LL;

int dato1;
int dato2;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1,canal);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
}

void loop() {
  if(radio.available())
  {
    radio.read(&dato1, sizeof(dato1));
    radio.read(&dato2, sizeof(dato2));
  } 
  Serial.print("dato1: ");
  Serial.println(dato1);
  Serial.print("dato2: ");
  Serial.println(dato2);
}
