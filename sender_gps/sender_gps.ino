/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com
*********/

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>                       

//define the pins used by the transceiver module
#define ss 18
#define rst 23
#define dio0 26

int counter = 0;
TinyGPSPlus gps;  
HardwareSerial GPSSerial1(1);                 

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  GPSSerial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX
  while (!Serial);
  Serial.println("LoRa Sender");
  
  LoRa.setPins(ss, rst, dio0);
  
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");

  smartDelay(1000); 
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));                                     
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // Logging 
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");
  
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.println("**********************");
  LoRa.print("Sending packet: ");
  LoRa.println(counter);
  LoRa.println("(2) #pristavka_nadom");
  LoRa.println(gps.location.lat(), 5);
  LoRa.println(gps.location.lng(), 5);
  LoRa.println("**********************");
  LoRa.endPacket();

  counter++;

  delay(5000);
}

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (GPSSerial1.available())
      gps.encode(GPSSerial1.read());
  } while (millis() - start < ms);
}
