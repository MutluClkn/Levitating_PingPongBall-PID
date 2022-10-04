// KÜTÜPHANELER
#include <Wire.h> 
#include <SharpIR.h>


// TANIMLAMALAR
#define IR A0
#define model 1080

// Sensör Model Seçimi
SharpIR SharpIR(IR, model);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float dis = SharpIR.distance();
  Serial.println(String(dis));
}
