#include <WiFiNINA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <LCD.h>

#define DHT_PIN 8     // Pin connected to the data pin of the DHT22 sensor
#define DHT_TYPE DHT22 // DHT sensor type (DHT11, DHT21, DHT22)

DHT dht(DHT_PIN, DHT_TYPE);

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define TEMP_THRESHOLD  75
#define Liquid_Detection_Pin 7

float airTemperature; // Variable to store temperature value in degrees Celsius
float humidity;    // Variable to store humidity value in percentage

char ssid[] = "SSID";
char pass[] = "PASS";

int status = WL_IDLE_STATUS;
String temperature;
char server[] = "127.0.0.1";
String postData;
String postTDSData;
String postVariable = "temp=";
String postTDSPPM = "TDS=";
int sensorPin = A1;
float volt;
float ntu;
int waterLvl = 0;

WiFiClient client;

int    HTTP_PORT   = 80;
String HTTP_METHOD = "GET";
char   HOST_NAME[] = "maker.ifttt.com";
String PATH_NAME   = "/trigger/send-email/with/key/WEBHOOKSKEY"; // change your Webhooks key

bool isSent = false;

namespace pin {
const byte tds_sensor = A0;
const byte one_wire_bus = 4; // Dallas Temperature Sensor
}

namespace device {
float aref = 4.3;
}

namespace sensor {
float ec = 0;
unsigned int tds = 0;
float waterTemp = 0;
float tempF = 0;
float ecCalibration = 1;
//float tss = 0;
}

OneWire oneWire(pin::one_wire_bus);
DallasTemperature dallasTemperature(&oneWire);

volatile int flow_frequency; // Measures flow sensor pulsesunsigned 

int l_hour; // Calculated litres/hour
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;

float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}


void flow () // Interrupt function

{
   flow_frequency++;
}

void setup() {
  Serial.begin(115200); // Dubugging on hardware Serial 0
  pinMode(7, INPUT);
  
  lcd.begin(16, 2);
  dallasTemperature.begin();
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  attachInterrupt(0, flow, RISING); // Setup Interrupt
  sei(); // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;

  
  dht.begin(); // Initialize the DHT sensor
}

void loop() {
  readTdsQuick();
  delay(2000);
}

void readTdsQuick() {
  dallasTemperature.requestTemperatures();
  sensor::waterTemp = dallasTemperature.getTempCByIndex(0);
  sensor::tempF = dallasTemperature.getTempFByIndex(0);
  waterLvl = digitalRead(7);

  // Read temperature and humidity from the sensor
  float airTemp = dht.readTemperature();
  float hum = dht.readHumidity();

  // Check if any reading failed
  if (isnan(airTemp) || isnan(hum)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Store the temperature and humidity values in variables
  airTemperature = (airTemp * 9.0 / 5.0) + 32.0;
  humidity = hum;

  
  volt = 0;
    for(int i=0; i<800; i++)
    {
        volt += ((float)analogRead(sensorPin)/1023)*5;
    }
    volt = volt/800;
    volt = round_to_dp(volt,2);
    
    if(volt < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(volt)+5742.3*volt-4353.8; 
    }
//  int tss = 0;
  float rawEc = analogRead(pin::tds_sensor) * device::aref / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float temperatureCoefficient = 1.0 + 0.02 * (sensor::waterTemp - 25.0); // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  sensor::ec = (rawEc / temperatureCoefficient) * sensor::ecCalibration; // temperature and calibration compensation
  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5; //convert voltage value to tds value
  Serial.print(F("TDS:")); Serial.println(sensor::tds);
  Serial.print(F("TSS:")); Serial.println(ntu, 2);
  Serial.print(F("Temperature:")); Serial.println(sensor::tempF,2);
  Serial.println(volt);
  lcd.clear();
  lcd.print("TDS   AIR  TEMP");
  lcd.setCursor(0,1); 
  lcd.print(sensor::tds); 
  lcd.setCursor(4,1); 
  lcd.print(airTemp, 2); 
  lcd.setCursor(11,1); 
  lcd.print(sensor::tempF,2);
  
  currentTime = millis();
   // Every second, calculate and print litres/hour
   if(currentTime >= (cloopTime + 1000))
   {
   cloopTime = currentTime; // Updates cloopTime
      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
      l_hour = (flow_frequency * 60 / 7.5); // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
      flow_frequency = 0; // Reset Counter
      Serial.print(l_hour, DEC); // Print litres/hour
      Serial.println(" L/hour");
   }

  Serial.print("Water Level = ");
  Serial.println(waterLvl,DEC);
  delay(500);
  
  String lvlDisplay = "Bt";
  
  if(waterLvl == 0){
    lvlDisplay = "Lo";
  }
  else{
    lvlDisplay = "Ok";
  }

  // int tss = ntu;
   
  postData = postVariable + sensor::tempF + ' ' + sensor::tds + ' ' + airTemp  + ' ' + hum +  ' ' + lvlDisplay;
 
    if (client.connect(server, 80)) {
    client.println("POST /post.php HTTP/1.1");
    client.println("Host: 127.0.0.1");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(postData.length());
    client.println();
    client.print(postData);
    delay(1000);   
  } 
  
  if (client.connected()) {
    client.stop();
  }
  
  Serial.println(postData);
  
  if (sensor::tempF <= TEMP_THRESHOLD) {
    if (isSent == false) { // to make sure that Arduino does not send duplicated emails
      sendEmail(sensor::tempF);
      isSent = true;
    }
  } else {
    isSent = false; // reset to send if the temperature exceeds threshold again
  }

}

void sendEmail(float tempF) {
  // connect to IFTTT server on port 80:
  if (client.connect(HOST_NAME, HTTP_PORT)) {
    // if connected:
    Serial.println("Connected to server");
    // make a HTTP request:
    String queryString = "?value1=" + String(tempF);
    // send HTTP header
    client.println("GET " + PATH_NAME + queryString + " HTTP/1.1");
    client.println("Host: " + String(HOST_NAME));
    client.println("Connection: close");
    client.println(); // end HTTP header

    while (client.connected()) {
      if (client.available()) {
        // read an incoming byte from the server and print it to serial monitor:
        char c = client.read();
        Serial.print(c);
      }
    }

    // the server's disconnected, stop the client:
    client.stop();
    Serial.println();
    Serial.println("disconnected");
  } else {// if not connected:
    Serial.println("connection failed");
  }
}
