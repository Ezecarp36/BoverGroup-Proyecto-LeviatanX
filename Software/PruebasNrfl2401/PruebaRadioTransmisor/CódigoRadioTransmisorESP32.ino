#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(17, 16); // pin CE, CSN à connecter sur les GPIO 17 et 16         
const byte address[6] = "00001"; // Adresse d'envoie (bien mettre la même dans le récepteur)
char buf[16];

void setup() {
  radio.begin();                  // Démarrage de la radio
  radio.openWritingPipe(address); // On lui donne son adresse de communication
  radio.setDataRate( RF24_250KBPS );// Vitesse a 250kbps
  radio.setPALevel(RF24_PA_MAX);  // Puissance maximale pour le test de portée
  radio.stopListening();          // Met le module en mode transmission
}

void loop()
{
  char buf[16];
  ltoa(millis(),buf,10);
  radio.write(&buf, sizeof(buf)); // Envoie d'un timestamp toutes les secondes
  delay(1000);
}