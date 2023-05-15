#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#define CE 9
#define CSN 10
RF24 radio (CE, CSN);
#define TICK_PRINT 500
unsigned long currentTime = 0;

const uint64_t canal = 0xE8E8F0F0E1LL;

int dato1;
int dato2;

void setup() {
  Serial.begin(9600);
  radio.begin();                  // Se inica el canal de comunicación.
  radio.openReadingPipe(1,canal); // Con la función "radio.openReadingPipe(1,canal)" se abre el canal de comunicación con el transmisor.
  radio.setPALevel(RF24_PA_MIN); // Con la función "radio.setPALevel(RF24_PA_MIN)" y su parámetro "RF24_PA_MIN" se configura el módulo en baja potencia.
  radio.startListening();        // Con la función "radio.starListening()" se define al módulo como receptor.
  
}

void loop() {
  if(radio.available())
  {
    radio.read(&dato1, sizeof(dato1)); // La función "radio.read()" recibe la información.
    radio.read(&dato2, sizeof(dato2));
  } 
  Serial.print("dato1: ");  // Se muestran los datos en pantalla.
  Serial.println(dato1);
  Serial.print("dato2: ");
  Serial.println(dato2);
}