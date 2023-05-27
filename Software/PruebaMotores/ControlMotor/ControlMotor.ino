#include <ESP32Servo.h>

#define PIN_MOTOR 27
Servo esc;  // Objeto del Servo para controlar el ESC
int velocidad = 1000;

void setup() {
  Serial.begin(9600);     // Inicializa la comunicación serial a 9600 bps
  esc.attach(PIN_MOTOR);  // Inicializa el servo en el pin correspondiente
}

void loop() {
  if (Serial.available()) 
  {
    if (velocidad > 1700) velocidad = 1700; //Pongo 1700 como valor maximo por las dudas
    velocidad = Serial.parseInt();  // Lee el valor de velocidad enviado por el puerto serie
    esc.writeMicroseconds(velocidad);
    
  }
}
