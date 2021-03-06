#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ADS1115.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_JSON.h>

//ADC temp readings
ADS1115 adc(ADS1115_DEFAULT_ADDRESS);
#define SLEEP_FOR 8000
#define THERMISTORNOMINAL 200000 //thermistor nominal resistance
#define TEMPERATURENOMINAL 25    //temp in C for nominal resistance
#define NUMSAMPLES 8             //number of samples to average over to reduce interference skew
#define BCOEFFICIENT 3500
#define SERIESRESISTOR 10000     //fixed resistor value used in Wheatstone bridge
#define NUMPROBES 4
float inF[NUMPROBES];
const int adcMax=(3.3/6.144)*32767;

//128x32 oled
#define SCREEN_WIDTH 128   // OLED display width, in pixels
#define SCREEN_HEIGHT 32   // OLED display height, in pixels
#define OLED_RESET     27  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//wifi connection
WiFiMulti wifiMulti;

//REST post of readings to cloud
HTTPClient http;

//functions
void getTemps(uint8_t probe);
void outputTemps();
void init_ads1115();

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
  display.setCursor(0,0);
  display.print("going WAP");
  delay(1000);
  display.display();
  
  Serial.println("setting ap");
  wifiMulti.addAP("","");
  wifiMulti.addAP("","");
  Serial.println("Connecting...");

  uint8_t idx=0;
  String msg="trying...";
  auto startTime=millis();
  uint8_t constatus;
  while (constatus!=WL_CONNECTED) {
   if (millis()%5000==0) {
    constatus=wifiMulti.run(500);
   }
   if (millis()%250==0) {
    (idx>2) ? idx=0 : idx++;
    display.clearDisplay();
    display.setTextSize(2); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,6);
    display.print(msg.substring(0,((msg.length()-3)+idx)));
    display.display();
   }
  }
  
  /*
  while (wifiMulti.run(1000) != WL_CONNECTED) {
   
   if (millis()%250==0) {
    Serial.print('.');
    (idx>2) ? idx=0 : idx++;
    display.clearDisplay();
    display.setTextSize(2); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,6);
    display.print(msg.substring(0,((msg.length()-3)+idx)));
    display.display();
   }
  }
  */

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
  if (millis()%SLEEP_FOR==0) {
    for (uint8_t probe = 0; probe < NUMPROBES; probe++) {
      getTemps(probe);
    }
    outputTemps();
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

  //get average over NUMSAMPLES readings to adjust for noise
  avg = 0;
  for (uint8_t i=0;i<NUMSAMPLES;i++) {
    avg+=adc.getConversion(true);
  }
  avg=avg/NUMSAMPLES;
  avg=adcMax/avg-1;
  avg=SERIESRESISTOR/avg;
  Serial.print("Thermistor resistance p"); Serial.print(String(probe)+": "); Serial.println(avg);

  //calculate temp in C using steinhart/hart equation
  avg=avg/THERMISTORNOMINAL;                 // (R/Ro)
  avg=log(avg);                              // ln(R/Ro)
  avg/=BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  avg+=1.0/(TEMPERATURENOMINAL+273.15);      // + (1/To)
  avg=1.0/avg;                               // Invert
  avg-=273.15;                               // convert to C

  Serial.print("p"); Serial.print(String(probe)+": "); Serial.print(avg); Serial.println("C");
  inF[probe]=avg*9/5+32; //convert to F
  Serial.print(F("p")); Serial.print(String(probe)+": "); Serial.print(inF[probe]); Serial.println(F("F"));
}

void outputTemps() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (uint8_t i=0; i<NUMPROBES; i++) {
    display.setCursor(10,i*8);
    display.print(F("Probe ")); display.print(i); display.print(F(": ")); display.print(inF[i]); display.cp437(true); display.write(7); display.cp437(true);
    display.display();
  }

  //post readings to remote host for web viewing
  if (WiFi.status()==WL_CONNECTED) { //Check WiFi connection status
    if (http.begin("https://thermi.pro:39780/api/setReading")) {
      JSONVar reading;
      reading["cookid"]="2";
      reading["probe0"]=inF[0];
      reading["probe1"]=inF[1];
      reading["probe2"]=inF[2];
      reading["probe3"]=inF[3];

      http.addHeader("Content-Type","application/json");
      int httpResponseCode=http.POST(JSON.stringify(reading));
      if (httpResponseCode>0) {
        String response=http.getString();                       //Get the response to the request
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
