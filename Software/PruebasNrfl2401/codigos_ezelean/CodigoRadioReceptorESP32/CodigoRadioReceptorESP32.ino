#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> 

#define CE 27
#define CSN 15
RF24 radio (CE, CSN);
//const byte address[6] = "00001";

float dato1;
void setup() {
  Serial.begin(9600);                // 
  radio.begin();
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL);   // 
  radio.startListening(); // Con la función "radio.starListening()" se define al módulo como receptor.           // 

}

void loop()
{
  if (radio.available())              // 
  {                
    radio.read(&dato1, sizeof(dato1));    // 
    
    Serial.println(dato1);               // 
  }
  else{
    Serial.println("No hay datos disponibles...");  
  }
  delay(1000);
}