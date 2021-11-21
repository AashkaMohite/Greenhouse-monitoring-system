
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>
#define DHTPIN 5   
#define ldr A0
#define moisture A1

SoftwareSerial nodemcu(2,3);


int ms=0;

int tmp=0;

String cdata; // complete data, consisting of sensors values

 // Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define DHTTYPE DHT11     // DHT 11 

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup() {

  lcd.begin();                      // initialize the lcd 
  //lcd.init();
   lcd.backlight();
  // put your setup code here, to run once:
   pinMode(moisture,INPUT_PULLUP);
   pinMode(ldr,INPUT);
  // lcd.begin();
   Serial.begin(9600);
   nodemcu.begin(9600);


 // Turn on the blacklight and print a message.
 
 // lcd.setcursor(0,1);
     lcd.print("GREEN HOUSE");
  delay(3000);
  lcd.clear();

   dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  
}

void loop() {
   

  
  // put your main code here, to run repeatedly:
  int light=map(analogRead(ldr),0,1023,1,10);
  int ms=map(analogRead(moisture),0,1023,1,100);
 
         // RS = constrain(RainSensor, 150, 440); 
          //RS = map(RainSensor, 150, 440, 1023, 0);
          
          if(ms>80)
          {
         
            lcd.print("Need water!!");
         
          delay(3000);
          lcd.clear();
          }else{
          lcd.print("water is enough!!");
          }
          if(light<=4){
            Serial.println("Night");
               lcd.print("Night");
              
            delay(3000);
          lcd.clear();
            
          }else if(light<7&&light>4){
            Serial.println("Evening");
             lcd.print("Evening");
            delay(3000);
          lcd.clear();
          }
          else if(light>=7)
          {
             lcd.print("Morning");
            delay(3000);
          lcd.clear();
          
         Serial.println("morning");
          }
          delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    lcd.setCursor(2, 1);
     lcd.println("Error reading");
    lcd.setCursor(2, 1);
     lcd.println("temperature!");
            delay(3000);
          lcd.clear();
  }
  else {
    Serial.print("Temperature: ");
    lcd.print("Temp:");
    lcd.print(event.temperature);
    lcd.print(" *C");
    tmp=event.temperature;
    delay(3000);
    lcd.clear();
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if(isnan(event.relative_humidity))
  {
    Serial.println("Error reading humidity!");
     lcd.setCursor(2, 1);
      lcd.println("Error reading");
    lcd.setCursor(3, 1);
     lcd.println("humidity!");
            delay(3000);
          lcd.clear();
  }
  else{
    Serial.print("Humidity: ");
    lcd.print("Humidity:");
    lcd.print(event.relative_humidity);
    lcd.print(" %");
    delay(3000);
    lcd.clear();
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
 Serial.println();
// tmp=event.temperature;
 Serial.println(tmp);
 cdata = cdata +event.relative_humidity+","+tmp+","+light+","+ms; // comma will be used a delimeter
   Serial.println(cdata);
   nodemcu.println(cdata);
   delay(3000); // 100 milli seconds
   cdata = "";
         
}
