#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "ADS1115.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_JSON.h>
#include <WiFiUdp.h>

#define SLEEP_FOR 8000
#define THERMISTORNOMINAL 200000
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 5
#define BCOEFFICIENT 3500
#define SERIESRESISTOR 10000
#define NUMPROBES 4

//oled
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     27 // Reset pin # (or -1 if sharing Arduino reset pin)

ADS1115 adc(ADS1115_DEFAULT_ADDRESS);
WiFiMulti wifiMulti;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//get time from NTP server
WiFiUDP UDP;
IPAddress timeServerIP;
const char* NTPServerName = "time.nist.gov"; //as good as any
const int NTP_PACKET_SIZE = 48;
byte NTPBuffer[NTP_PACKET_SIZE];

float inF[NUMPROBES];
const int adcMax = (3.3 / 6.144) * 32767;

//functions
void getTemps(uint8_t probe);
void outputTemps(char ts[9]);
void init_ads1115();
uint32_t getTime();
void sendNTPpacket(IPAddress& address);
int getSeconds(uint32_t UNIXTime);
int getMinues(uint32_t UNIXTime);
int getHours(uint32_t UNIXTime);
void startUDP();

//ntp
unsigned long intervalNTP = 60000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t timeUNIX = 0;

unsigned long prevActualTime = 0;

void setup() {
  Serial.begin(115200); // initialize serial communication
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("trying WAP");
  delay(500);
  display.display();
  
  Serial.println("setting ap");
  wifiMulti.addAP("", "");
  wifiMulti.addAP("", "!");
  Serial.println("Connecting ...");
  uint16_t x=0,y=0;
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    display.clearDisplay();
    display.setTextSize(2); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    if (x>=128) {
      x=0;
      y++;
    }
    display.setCursor(y,x);
    display.print(".");
    display.display();
    x++;
  }
  startUDP();
  if (!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.restart();
  }
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);

  Serial.println("\r\nSending NTP request ...");
  sendNTPpacket(timeServerIP);


  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("AP: ")); display.println((WiFi.SSID()));
  display.display();
  delay(3000);
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("IP: ")); display.println((WiFi.localIP()));
  display.display();

  Serial.println('\n');
  Serial.print("Connected to "); Serial.println(WiFi.SSID());
  Serial.print("Our IP address:\t"); Serial.println(WiFi.localIP());
  delay(3000);

  //I2Cdev::begin();  // join I2C bus
  Wire.begin();
  init_ads1115();
}

void loop() {
  unsigned long currentMillis = millis();
  char ts[9];

  if (currentMillis - prevNTP > intervalNTP) { // If a minute has passed since last NTP request
    prevNTP = currentMillis;
    Serial.println("\r\nSending NTP request ...");
    sendNTPpacket(timeServerIP);               // Send an NTP request
  }

  if (millis() % SLEEP_FOR == 0) {
    uint32_t time = getTime();                   // Check if an NTP response has arrived and get the (UNIX) time
    if (time) {                                  // If a new timestamp has been received
      timeUNIX = time;
      Serial.print("NTP response:\t");
      Serial.println(timeUNIX);
      lastNTPResponse = currentMillis;
    } else if ((currentMillis - lastNTPResponse) > 3600000) {
      //ESP.reset();
    }
    uint32_t actualTime = timeUNIX + (currentMillis - lastNTPResponse) / 1000;
    if (actualTime != prevActualTime && timeUNIX != 0) { // If a second has passed since last print
      prevActualTime = actualTime;
      snprintf(ts, 9, "%02d:%02d:%02d", getHours(actualTime), getMinutes(actualTime), getSeconds(actualTime));
    }
    for (uint8_t probe = 0; probe < NUMPROBES; probe++) {
      getTemps(probe);
    }
    Serial.println(ts);
    outputTemps(ts);
  }
}

void getTemps(uint8_t probe) {
  float avg;

  //set adc channel for the probe to read
  switch (probe) {
    case 0:
      adc.setMultiplexer(ADS1115_MUX_P0_NG);
      break;
    case 1:
      adc.setMultiplexer(ADS1115_MUX_P1_NG);
      break;
    case 2:
      adc.setMultiplexer(ADS1115_MUX_P2_NG);
      break;
    case 3:
      adc.setMultiplexer(ADS1115_MUX_P3_NG);
      break;
  }

  //get average over 8 readings to adjust for noise
  avg = 0;
  for (uint8_t i = 0; i < 8; i++) {
    avg += adc.getConversion(true);
  }
  avg = avg / 8;
  avg = adcMax / avg - 1;
  avg = SERIESRESISTOR / avg;
  Serial.print("Thermistor resistance p"); Serial.print(String(probe) + ": "); Serial.println(avg);

  //calculate temp in C using steinhart/hart equation
  avg = avg / THERMISTORNOMINAL;               // (R/Ro)
  avg = log(avg);                              // ln(R/Ro)
  avg /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  avg += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  avg = 1.0 / avg;                             // Invert
  avg -= 273.15;                               // convert to C

  Serial.print("p"); Serial.print(String(probe) + ": "); Serial.print(avg); Serial.println("C");
  inF[probe] = avg * 9 / 5 + 32; //convert to F
  Serial.print(F("p")); Serial.print(String(probe) + ": "); Serial.print(inF[probe]); Serial.println(F("F"));
}

void outputTemps(char ts[9]) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (uint8_t i = 0; i < NUMPROBES; i++) {
    display.setCursor(10, i * 8);
    display.print(F("Probe ")); display.print(i); display.print(F(": ")); display.print(inF[i]); display.cp437(true); display.write(7); display.cp437(true);
    display.display();
  }

  //post readings to remote host for web viewing
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    HTTPClient http;
    //Post.begin("http://bbqpi/arduino.php");
    if (http.begin("https://thermi.pro:39780/api/setReading")) {
      JSONVar reading;
      reading["cookid"] = "2";
      reading["probe0"] = inF[0];
      reading["probe1"] = inF[1];
      reading["probe2"] = inF[2];
      reading["probe3"] = inF[3];
      reading["time"] = ts;

      //http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      http.addHeader("Content-Type", "application/json");
      //int httpResponseCode = http.POST("probe0="+String(inF[0])+"&probe1="+String(inF[1])+"&probe2="+String(inF[2])+"&probe3="+String(inF[3]));
      int httpResponseCode = http.POST(JSON.stringify(reading));
      if (httpResponseCode > 0) {
        String response = http.getString();                       //Get the response to the request
        Serial.print(httpResponseCode); //Print return code
        Serial.print(": ");   //Print return code
        Serial.println(response);           //Print request answer
      } else {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
  }
}

void init_ads1115() {
  Serial.println(F("Testing device connections..."));
  Serial.println(adc.testConnection() ? F("ADS1115 connection successful") : F("ADS1115 connection failed"));

  adc.initialize(); // initialize ADS1115 16 bit A/D chip
  //set our ADS1115 config parameters
  adc.setMultiplexer(ADS1115_MUX_P0_NG);                          // single-ended reading on A0
  adc.setGain(ADS1115_PGA_6P144);                                 // set PGA gain to run at 2.048v
  adc.setMode(ADS1115_MODE_SINGLESHOT);                           // set mode to single shot reading
  adc.setRate(ADS1115_RATE_860);                                  // set sample rate to 860SPS, we're going for burst-read then power save
  adc.setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);            // default/ignored because ALERT/RDY
  adc.setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_HIGH);        // ALERT/RDY pin normall hi, go low when conversion is ready to read
  adc.setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);   // default/ignored because ALERT/RDY
  adc.setComparatorQueueMode(ADS1115_COMP_QUE_DISABLE);           // default/ignored because ALERT/RDY
  adc.setConversionReadyPinMode();
}

void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages on port 123
  //Serial.print("Local port:\t");
  //Serial.println(UDP.localPort());
  Serial.println();
}

uint32_t getTime() {
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}

void sendNTPpacket(IPAddress& address) {
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

inline int getSeconds(uint32_t UNIXTime) {
  return UNIXTime % 60;
}

inline int getMinutes(uint32_t UNIXTime) {
  return UNIXTime / 60 % 60;
}

inline int getHours(uint32_t UNIXTime) {
  return UNIXTime / 3600 % 24;
}
