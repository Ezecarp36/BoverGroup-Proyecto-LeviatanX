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
  radio.openWritingPipe(canal); // Con laa función "radio.openWritingPipe(canal)" se abre el canal de comunicación con el receptor.
  radio.setPALevel(RF24_PA_MIN); // Con la función "radio.setPALevel(RF24_PA_MIN)" y su parámetro "RF24_PA_MIN" se configura el módulo en baja potencia.
  radio.stopListening(); // Con la función "radio.stopListening()" se define al módulo como transmisor.
}

void loop() {
  radio.write(&dato1,sizeof(dato1)); // La función "radio.write()" envía la información.
  radio.write(&dato2,sizeof(dato2));
  delay(200);
}
