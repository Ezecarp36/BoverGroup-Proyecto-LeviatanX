#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h> 

Adafruit_SSD1306 screen(128, 64, &Wire, -1); // Ecran sur port I2C
RF24 radio(17, 16); // pin CE, CSN à connecter sur les GPIO 22 et 21    
const byte address[6] = "00001"; // Adresse d'envoie (bien mettre la même dans l'émetteur)

void setup() {
  Serial.begin(115200);                // Communication série pour afficher le Ping
  radio.begin();
  radio.openReadingPipe(0, address);   // On lui donne son adresse de communication
  radio.setDataRate( RF24_250KBPS );   // Vitesse a 250kbps
  radio.setPALevel(RF24_PA_MAX);       // Puissance maximale pour le test de portée
  radio.startListening();              // Met le module en mode réception

  // Démarrage de la connexion à l'écran
  if(!screen.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println(F("Screen SSD1306 Not found"));
  }
  else
  {
    Serial.println(F("Screen SSD1306 Found"));
    screen.clearDisplay();
    
    screen.setTextSize(3);      // Normal 1:1 pixel scale
    screen.setTextColor(SSD1306_WHITE); // Draw white text
    screen.setCursor(0, 0);     // Start at top-left corner
    screen.cp437(true);         // Use full 256 char 'Code Page 437' font
  }
}

void loop()
{
  if (radio.available())              // A-t-on reçu quelque chose...
  {
    char text[32] = "";                 
    radio.read(&text, sizeof(text));    // Lecture de la donnée recue
    Serial.println(text);               // Affichage sur moniteur serie
    screen.clearDisplay();  
    screen.setCursor(0, 0);
    screen.print(text);                  // Affichage sur écran
    screen.display();
  }
}