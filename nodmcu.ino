#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <SimpleTimer.h>

WidgetLCD lcd(V0);
char auth[] = "gv0NMSpXn88V72UmEGkMC9iGFomOMYmj";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Ash";
char pass[] = "1234512345";

SimpleTimer timer;

String myString; // complete message from arduino, which consistors of snesors data
char rdata; // received charactors

int firstVal, secondVal, thirdVal,fourthVal; // sensors
// This function sends Arduino's up time every second to Virtual Pin (1).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V1, millis() / 1000);

}



void setup()
{
  // Debug console
  Serial.begin(9600);
  
  Blynk.begin(auth, ssid, pass);
  
lcd.print(0,0,"Green House");
  delay(5000);
  lcd.print(0,1,"Data Detected");
  delay(3000);
 // lcd.clear();
  timer.setInterval(1000L, sensorvalue1);
  timer.setInterval(1000L, sensorvalue2);
  timer.setInterval(1000L, sensorvalue3);
  timer.setInterval(1000L, sensorvalue4);

  
}

void loop()
{
  
  if (Serial.available() == 0 )
  {
    Blynk.run();
    timer.run(); // Initiates BlynkTimer
  }

  if (Serial.available() > 0 )
  {
    rdata = Serial.read();
    myString = myString + rdata;
    // Serial.print(rdata);
    if ( rdata == '\n')
    {
      //  Serial.println(myString);
      // Serial.println("fahad");
      // new code
      String l = getValue(myString, ',', 0);
      String m = getValue(myString, ',', 1);
      String n = getValue(myString, ',', 2);
      String o = getValue(myString, ',', 3);


      firstVal = l.toInt();
      secondVal = m.toInt();
      thirdVal = n.toInt();
      fourthVal = o.toInt();
      
      myString = "";
      // end new code
    }
  }

}

void sensorvalue1()
{
  int sdata = firstVal;
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, sdata);

}

void sensorvalue2()
{
  int sdata = secondVal;
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V3, sdata);
}

void sensorvalue3()
{
  int sdata = thirdVal;
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V4, sdata);

}
void sensorvalue4()
{
  int sdata = fourthVal;
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, sdata);

}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
