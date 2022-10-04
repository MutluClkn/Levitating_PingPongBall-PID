// KÜTÜPHANELER
#include <Wire.h> 
#include <SharpIR.h>
#include <LiquidCrystal.h>


// TANIMLAMALAR
#define IR A5
#define model 1080
#define PWM_Out 3
#define pot A1
#define buttonPin1 8 // Kırmızı Buton
#define buttonPin2 10 // Yeşil Buton

// Sensör Model Seçimi
SharpIR SharpIR(IR, model);

// LCD Tanımlaması
const int rs = 24, en = 34, d4 = 38, d5 = 40, d6 = 44, d7 = 48;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

  // DEĞİŞKENLER
int stopButtonState = 0; // Durdur butonu (Kırmızı)
int runButtonState = 0; // Çalıştır Butonu (Yeşil)
float ref = 0.0; // Referans Noktası
float dis = 0.0; // Mesafe
float error = 0; // Hata
float errorDiff = 0.0; // Hata değeri
float errorOld = 0.0; // Önceki hata değeri
float P;
float I = 0.0;
float PID;
float Kp = 0.9;
float Ki = 0.0035;
int errorOldLoop = 1;
float temporaryDis;
int buttonState = 0;
int lcdClear = 1;

void setup()
{
  pinMode(PWM_Out, OUTPUT); // PWM çıkışı
  pinMode(pot, INPUT); // Potansiyometre
  pinMode(buttonPin1, INPUT); // Kırmızı Buton Bağlantısı
  pinMode(buttonPin2, INPUT); // Yeşil Buton Bağlantısı
  
  lcd.begin(20,4); // 20x4 LCD başlatılıyor
  Serial.begin(9600); // Seri port ekranı için saniye başı bit
}

void loop()
{
  dis = 0;

  // BAŞLA - DUR BUTONU
  stopButtonState = digitalRead(buttonPin1);
  delay(1000);
  runButtonState = digitalRead(buttonPin2);
  delay(1000);
  Serial.println(String(stopButtonState)+" "+ String(runButtonState));
  
  if (stopButtonState == HIGH){
    buttonState = 0;
  }else if (runButtonState == HIGH){
    buttonState = 1;
  }

  
  if (buttonState == 0) {
    Serial.println("Buton OFF");
    String Kp_String = String(Kp, 2);
    String Ki_String = String(Ki, 4);
    lcd.setCursor(0,0);
    lcd.print("Kp=" + Kp_String + "/" + "Ki=" + Ki_String); // LCD ekranına Kp ve Ki değerleri yansıtılıyor.
    lcd.setCursor(0,1);
    lcd.print("Ref1=50cm/Ref2=10cm"); // LCD ekranına, belirlenen Referans değerleri yansıtılıyor.
    lcd.setCursor(0,2);
    lcd.print(" ");
    lcd.setCursor(6,3);
    lcd.print("HAZIRIM");
    

    // DEĞİŞKENLER 
    dis = 0; // Mesafe
    error = 0; // Hata
    errorDiff = 0; // Hata değeri
    errorOld = 0; // Önceki hata değeri
    I = 0.0;
    lcdClear = 1;
  }
  
  
  else if (buttonState == 1) {
    Serial.println("BUTTON ON");
    // LCD ilk çalışmada temizleniyor.
    while(lcdClear >= 1 ){
        lcd.clear();
        lcdClear = 0;
    }

   
    // MESAFE İŞLEMLERİ
 
    for (int i=0;i<10;i++) {
      temporaryDis = SharpIR.distance(); // Mikroişlemciyi tıkamamak için delayli 10 adet uzaklık ölçümünün ortalaması alınıyor.
      delay(55);
     
     if (temporaryDis < 10 && temporaryDis > 60) {
      temporaryDis = 60 - temporaryDis;
      
     }else if (temporaryDis < 10) {
      temporaryDis = 10;
      temporaryDis = 60 - temporaryDis;
      
     }else if (temporaryDis > 60) {
      temporaryDis = 60;
      temporaryDis = 60 - temporaryDis;
     }
     dis += temporaryDis;
    }
    
    float averageDistance = (dis/10.0); 
    
    String distance = String(averageDistance, 1);
    distance = "Mesafe:" + distance + "cm";
    Serial.println(distance);
    lcd.setCursor(0,0);
    lcd.print(distance); // LCD ekranına ölçülen mesafe yansıtılıyor.
    

    // POT POZİSYONU
    int potPosition =  analogRead(pot);

    if (potPosition >= 1) {
     ref = 10;
    }else if (potPosition == 0) {
     ref = 50;
    }

    // PID HESAPLAMALARI

    error = ref - averageDistance; // Referans noktasından mevcut top uzaklığını çıkararak hata ölçülüyor.
    errorDiff = errorOld - error;

    // İlk çalışmada errorOld olmadığı için errorDiff'i 1 olarak alıyoruz.
     while(errorOldLoop > 0){
      errorOldLoop = 0;
      errorDiff = 1;
    }
    
    errorOld = error;
    
    
    String errorString = String(error, 1);
    lcd.setCursor(0,1);
    lcd.print("Hata: " + errorString); // LCD ekranına mevcut hata değeri yansıtılıyor.

    P = Kp * error;
    I = I + Ki*error*errorDiff;
    PID = P + I;  // GELEN PWM SİNYALİNİ 0-255 ARASINA SIKIŞTIRMAK İÇİN GEREKLİ MAX DEĞERİ ÖĞREN. HOCAYA NASIL YAPILABİLECEĞİNİ SOR.
    
    String P_String = String(P, 1);
    String I_String = String(I, 1);
    String PID_String = String(PID, 1);
    lcd.setCursor(0,2);
    lcd.print("P:" + P_String + " / " + "I:" + I_String); // LCD ekranına P ve I değerleri yansıtılıyor.
    lcd.setCursor(0,3);
    lcd.print("PID:" + PID_String); // LCD ekranına PID (PWM) değeri yansıtılıyor.
    
    analogWrite(PWM_Out, PID); // Motora gönderilen PWM sinyali
    
    
    
    Serial.println("P=" + String(P));
    Serial.println("I=" + String(I));
    Serial.println("PWM=" + String(PID));
    
  }
}
