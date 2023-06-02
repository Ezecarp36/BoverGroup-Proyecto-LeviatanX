#include <RF24.h>
#define CE_PIN  9
#define CSN_PIN 10

const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN);

char dataToSend[10] = "tororo"; // Datos a enviar

void setup()
{
   Serial.begin(9600);
   delay(3000);
   Serial.println("SimpleTx Starting");
   radio.begin();
   radio.setDataRate(RF24_250KBPS);
   radio.openWritingPipe(thisSlaveAddress);
}

void loop()
{
   sendData();
   delay(1000); // Esperar antes de la próxima transmisión
}

void sendData()
{
   radio.write(&dataToSend, sizeof(dataToSend));
   Serial.println("Data sent");
}