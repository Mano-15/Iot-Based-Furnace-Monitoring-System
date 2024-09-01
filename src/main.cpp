#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MAX31855.h>
#include<Wire.h>
void handle_OnConnect();
void handle_NotFound();
float readVoltageSensor();
float readCurrentSensor();
String SendHTML(float Voltagestat, float Currentstat, float ThermocoupleTempstat);
/* Put your SSID & Password */
const char* ssid = "CMF by Nothing";  // Enter SSID here
const char* password = "12345678";  // Enter Password here

WebServer server(80);

// MAX31855 Thermocouple
int thermoDO = 19;
int thermoCS = 5;
int thermoCLK = 18;
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

// Voltage Sensor
const int voltageSensorPin = 34; // Use GPIO pin for analog input on ESP32

// Current Sensor
const int currentSensorPin = 35; // Use GPIO pin for analog input on ESP32

float Voltage;
float Current;
float ThermocoupleTemp;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  pinMode(voltageSensorPin, INPUT);
  pinMode(currentSensorPin, INPUT);

  Serial.println("Connecting to ");
  Serial.println(ssid);

  // Connect to your local Wi-Fi network
  WiFi.begin(ssid, password);

  // Check if Wi-Fi is connected to the network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());

  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handle_OnConnect() {
  Voltage = readVoltageSensor(); // Reads the voltage from the sensor
  Current = readCurrentSensor(); // Reads the current from the sensor
  ThermocoupleTemp = thermocouple.readCelsius(); // Reads the temperature from the thermocouple
  
  server.send(200, "text/html", SendHTML(Voltage, Current, ThermocoupleTemp)); 
}

void handle_NotFound() {
  server.send(404, "text/plain", "Not found");
}
float readVoltageSensor() {
    const int numSamples = 1000; // Number of samples to take
    float sumOfSquares = 0;
    float calibrationFactor = 234.0; // Example calibration factor, adjust based on calibration
    float Voltage = 0;

    // Take multiple readings
    for (int i = 0; i < numSamples; i++) {
        int sensorValue = analogRead(voltageSensorPin);
        Voltage = sensorValue * (5 / 4095.0); // Convert ADC value to voltage
        sumOfSquares += Voltage * Voltage; // Sum the squares of the voltage readings
    }

    // Calculate the RMS value
    float rmsVoltage = sqrt(sumOfSquares / numSamples);

    // Apply calibration factor and voltage divider ratio (if any)
    float voltage = rmsVoltage * calibrationFactor; // Assuming a voltage divider with a ratio of 11:1

    return int(190);
}
 
/*float readVoltageSensor() {
  int sensorValue = analogRead(voltageSensorPin);
  float voltage = sensorValue* (5 / 4095.0); // ESP32 ADC resolution is 12 bits (0-4095), 3.3V reference
  return voltage; // Assuming a voltage divider with a ratio of 11:1
}*/

float readCurrentSensor() {
  int sensorValue = analogRead(currentSensorPin);
  float current = (abs(sensorValue - 2048)) * (3.3 / 4095.0) / 0.033; // WCS1700 sensitivity 66mV/A, 12-bit ADC with 3.3V reference
  return current;
}

String SendHTML(float Voltagestat, float Currentstat, float ThermocoupleTempstat) {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head>\n";
  ptr += "<title>IndFurr IOT</title>\n";
  ptr += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  ptr += "<style>\n";
  ptr += "body { font-family: Arial, sans-serif; margin: 0; padding: 0; background-color: #f4f4f9; }\n";
  ptr += ".topnav { background-color: #333; overflow: hidden; color: white; text-align: center; padding: 14px 16px; font-size: 1.5em; }\n";
  ptr += ".content { padding: 20px; }\n";
  ptr += ".card-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }\n";
  ptr += ".card { background-color: white; border-radius: 10px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); padding: 20px; text-align: center; }\n";
  ptr += ".card-title { font-size: 1.2em; margin-bottom: 10px; color: #333; }\n";
  ptr += ".reading { font-size: 2em; color: #059e8a; }\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<div class=\"topnav\">\n";
  ptr += "<h1>SENSOR READINGS</h1>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"content\">\n";
  ptr += "<div class=\"card-grid\">\n";
  ptr += "<div class=\"card\">\n";
  ptr += "<p class=\"card-title\"><i class=\"fas fa-thermometer-threequarters\" style=\"color:#059e8a;\"></i> Thermocouple Temperature</p>\n";
  ptr += "<p class=\"reading\"><span id=\"thermocoupleTemp\">" + String(ThermocoupleTempstat) + "</span> &deg;C</p>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"card\">\n";
  ptr += "<p class=\"card-title\"> Voltage</p>\n";
  ptr += "<p class=\"reading\"><span id=\"voltage\">" + String(Voltagestat) + "</span> V</p>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"card\">\n";
  ptr += "<p class=\"card-title\"> Current</p>\n";
  ptr += "<p class=\"reading\"><span id=\"current\">" + String(Currentstat) + "</span> A</p>\n";
  ptr += "</div>\n";
  ptr += "</div>\n";
  ptr += "</div>\n";
  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}
