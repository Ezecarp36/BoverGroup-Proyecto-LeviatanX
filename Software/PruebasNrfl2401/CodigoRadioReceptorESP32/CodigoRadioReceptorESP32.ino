#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> 

RF24 radio(27, 15); //
const byte address[6] = "00001";
float dato1;
void setup() {
  Serial.begin(9600);                // 
  radio.begin();
  radio.openReadingPipe(1, address);   // 
  radio.startListening();              // 

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