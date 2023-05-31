#include <ESP32Servo.h>
#include "BluetoothSerial.h"

//BLUETOOTH
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
//CONSTANTES
#define PIN_MOTOR 25

//INSTANCIAMOS OBJETOS
BluetoothSerial SerialBT;
Servo esc;  // Objeto del Servo para controlar el ESC

void setup() {
  Serial.begin(115200);     // Inicializa la comunicación serial a 9600 bps
  esc.attach(PIN_MOTOR);  // Inicializa el servo en el pin correspondiente
  SerialBT.begin("ESP32test"); //Bluetooth device name
  delay(3000);
  SerialBT.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
if(SerialBT.available()){
    for (int i = 1000; i < 1800; i= i+25) 
    {
        esc.writeMicroseconds(i);
        SerialBT.print("velocidad: ");
        SerialBT.println(i);
        delay(1500);
        if (i == 1800){
          break;
        } 
    }
}
}
