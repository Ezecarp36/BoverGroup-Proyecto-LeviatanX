#include <ESP32Servo.h>

#define PIN_MOTOR 27
Servo esc;  //Inicializo un objeto de la clase Servo para enviarle el pulso al ESC

void setup() {
  esc.attach(PIN_MOTOR);  // Inicializa el servo en el pin correspondiente del motor

void loop() {
  esc.writeMicroseconds(1000);  //Envio un pulso al motor de entre 1ms y 2ms
  delay(5000);
  esc.writeMicroseconds(0);  //Paro el motor
  delay(5000);
}