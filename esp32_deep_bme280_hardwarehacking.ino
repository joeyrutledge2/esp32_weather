#include <TimeLib.h>
//#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

WiFiMulti wifiMulti;
WiFiUDP Udp;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
#define BME_SCK 5
#define BME_MISO 12
#define BME_MOSI 4
#define BME_CS 10

float tempC;
float pressure;
float humidity;
float tempF;
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

RTC_DATA_ATTR int bootCount = 0;

//IPAddress timeServer(10, 1, 0, 1); // time-b.timefreq.bldrdoc.gov
IPAddress timeServer(162, 213, 2, 253); // time-b.timefreq.bldrdoc.gov

int sensorPin = A13;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
float sensorValue = 0.00;  // variable to store the value coming from the sensor
float voltage = 0.00;
const char carbonHost[] = "grafana.joeyrutledge.com";
unsigned int carbonPort = 2003;

time_t prevDisplay = 0; // when the digital clock was displayed


const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}


void setup() {
  //wifiMulti.addAP("j-vmware", "hellomynameisjoeyrutledge");
  //wifiMulti.addAP("j-vmware");
  //wifiMulti.addAP("i see you", "h1H3nma$@R$@#hhh23mv@");
  wifiMulti.addAP("Joey", "hellomynameisjoey");

  Serial.begin(9600);
  delay(1000);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //pinMode(ledPin, OUTPUT);

    if (! bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }
    
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    Serial.println("Connecting Wifi...");
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }
    print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
  Serial.println("waiting for ntp sync");
  setSyncProvider(getNtpTime);
  bme.takeForcedMeasurement(); // has no effect in normal mode

  printVoltage();
  sendUDPtest();

  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() 
{
    if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    delay(1000);
  }
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      digitalClockDisplay();
      //sendUDPtest();
      delay(60000);

    }
  }

  printVoltage();
  printValues();

}

void printVoltage() 
{
  sensorValue = analogRead(sensorPin);
  voltage = (sensorValue*2/1000);
  Serial.println(voltage);
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  delay(sensorValue);
  // turn the ledPin off:
  digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  delay(sensorValue);
}


void digitalClockDisplay() {
  // digital clock display of the tim
  Serial.print(now());
  Serial.println();
  //Serial.print(hour());
  //printDigits(minute());
  //printDigits(second());
  //Serial.print(" ");
  //Serial.print(day());
  //Serial.print(".");
  //Serial.print(month());
  //Serial.print(".");
  //Serial.print(year());
  //Serial.println();
}

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      Serial.print("Seconds since Jan 1 1900 = ");
      Serial.println(secsSince1900);

      // now convert NTP time into everyday time:
      Serial.print("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      Serial.println(epoch);
      return epoch;
      ////unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      //secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      //secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      //secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      //secsSince1900 |= (unsigned long)packetBuffer[43];
      //return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
      //return secsSince1900;

    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void sendUDPtest()
{
  readBME();  
  Udp.beginPacket(carbonHost, carbonPort);
  Udp.print("esp32.sensor.voltage ");
  Udp.print(voltage);
  Udp.print(" ");
  Udp.println(now());
  Serial.println(voltage);
  digitalWrite(0, LOW);
  Udp.endPacket();

  Udp.beginPacket(carbonHost, carbonPort);
  Udp.print("esp32.sensor.temp.c ");
  Udp.print(tempC);
  Udp.print(" ");
  Udp.println(now());
  //Serial.println(bme.readTemperature());
  Serial.println(tempC);
  digitalWrite(0, LOW);
  Udp.endPacket();

  Udp.beginPacket(carbonHost, carbonPort);
  Udp.print("esp32.sensor.pressure ");
  Udp.print(pressure);
  Udp.print(" ");
  Udp.println(now());
  Serial.println(pressure);
  digitalWrite(0, LOW);
  Udp.endPacket();

  Udp.beginPacket(carbonHost, carbonPort);
  Udp.print("esp32.sensor.humidity ");
  Udp.print(humidity);
  Udp.print(" ");
  Udp.println(now());
  Serial.println(humidity);
  digitalWrite(0, LOW);
  Udp.endPacket();

  Udp.beginPacket(carbonHost, carbonPort);
  Udp.print("esp32.sensor.temp.f ");
  Udp.print(tempF);
  Udp.print(" ");
  Udp.println(now());
  Serial.println(tempF);
  Udp.endPacket();


}



void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void readBME() {
  tempC = bme.readTemperature();
  tempF = tempC * 1.8 + 32;
  pressure = (bme.readPressure() / 100.0F);
  humidity = bme.readHumidity();
}


