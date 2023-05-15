#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(27, 15); // pin CE, CSN a conectar en los GPIO 17 y 16
const byte address[6] = "00001"; // Dirección de envío (asegúrate de que coincida con el receptor)
char buf[16];

void setup() {
radio.begin(); // Inicia la radio
radio.openWritingPipe(address); // Establece la dirección de comunicación
radio.setDataRate(RF24_250KBPS); // Velocidad de datos a 250kbps
radio.setPALevel(RF24_PA_MAX); // Potencia máxima para la prueba de alcance
radio.stopListening(); // Coloca el módulo en modo de transmisión
}

void loop()
{
char buf[16];
ltoa(millis(), buf, 10);
radio.write(&buf, sizeof(buf)); // Envía una marca de tiempo cada segundo
delay(1000);
}