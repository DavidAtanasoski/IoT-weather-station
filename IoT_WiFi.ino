#include <WiFiNINA.h>
#include <SPI.h>
#include <DHT.h>
#include <array>
#include "ThingSpeak.h"
#include "secrets.h"

#define CHANNEL_NUMBER SECRET_CHANNEL_NUMBER
#define WRITE_API_KEY SECRET_WRITE_API_KEY

#define DHT_PIN 7
#define DHT_SENSOR_TYPE DHT22

#define NUM_MEASUREMENTS 5

DHT dht(DHT_PIN, DHT_SENSOR_TYPE);

IPAddress ip(10, 0, 1, 1);

const char *ssid = STATION_NAME;      // your network SSID (name)
const char *pass = STATION_PASSWORD;  // your network password (use for WPA, or use as key for WEP)

String ssidName = DEFAULT_SSID_NAME;
String ssidPassword = DEFAULD_SSID_PASSWORD;

int status = WL_IDLE_STATUS;
WiFiServer server(80);
WiFiClient networkClient;

unsigned long channelNumber = CHANNEL_NUMBER;
const char *writeApiKey = WRITE_API_KEY;

bool accessPointMode = true;

int ledPin = LED_BUILTIN;
unsigned long previousMillis = 0;  // Store last time the LED was updated
const long interval = 1000;        // Interval for blinking (1 second)

std::array<float, 2> takeMeasurements() {
  int measurementsTaken = 0;
  float temperatureSum = 0.0;
  float humiditySum = 0.0;

  while (measurementsTaken < NUM_MEASUREMENTS) {
    delay(2000);

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read data from the DHT Sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" Celsius, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      temperatureSum += temperature;
      humiditySum += humidity;

      measurementsTaken++;
    }
  }

  return { temperatureSum, humiditySum };
}

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  dht.begin();

  pinMode(ledPin, OUTPUT);

  // Remove this to not wait for serial
  // while (!Serial) {
  //   ;  // wait for serial port to connect. Needed for native USB port only
  // }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  if (connectToNetwork(ssidName, ssidPassword)) {
    Serial.println("Connected in setup!");
    Serial.print("Local IP Address: ");
    Serial.println(WiFi.localIP());

    ThingSpeak.begin(networkClient);
    accessPointMode = false;
  } else {
    Serial.println("Can't connect in setup!");
    accessPointMode = true;
    initAP();
  }
}

void loop() {
  // compare the previous status to the current status
  if (status != WiFi.status() && accessPointMode) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  if (accessPointMode) {

    String networkName = "";
    String networkPass = "";

    clientListen(networkName, networkPass);

    if (networkName != "" && networkPass != "") {
      Serial.print("SSID Name: ");
      Serial.println(networkName);
      Serial.print("Password: ");
      Serial.println(networkPass);

      ssidName = networkName;
      ssidPassword = networkPass;

      accessPointMode = false;
      WiFi.end();
    }
  } else {
    digitalWrite(ledPin, LOW);

    bool connectionStatus = checkWiFiConnection();

    if (!connectionStatus) {

      bool connected = connectToNetwork(ssidName, ssidPassword);
      if (connected) {
        Serial.println("\nConnected!");
        Serial.print("Local IP Address: ");
        Serial.println(WiFi.localIP());

        ThingSpeak.begin(networkClient);
      } else {
        // accessPointMode = true;
        // initAP();
        // loop until you connect to network and blink led
        while (!connectToNetwork(ssidName, ssidPassword)) {
          ;
        }
      }
    } else {
      
      digitalWrite(ledPin, LOW);

      std::array<float, 2> summedMeasures = takeMeasurements();

      float averageTemperature = getAverageTemperature(summedMeasures[0]);
      float averageHumidity = getAverageHumidity(summedMeasures[1]);

      Serial.print("Average Temperature: ");
      Serial.print(averageTemperature);
      Serial.print(" Celsius, Humidity: ");
      Serial.print(averageHumidity);
      Serial.println(" %");

      bool response = writeToCloud(averageTemperature, averageHumidity);

      if (response) {
        Serial.println("Readings were successfully written!");
      } else {
        Serial.println("Connection lost! Readings were not saved.");
        // loop until you connect to the network
        WiFi.end();
        while (!connectToNetwork(ssidName, ssidPassword)) {
          ;
        }
      }
    }
  }
}

void blinkLED(int tInterval) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= tInterval) {
    previousMillis = currentMillis;
    digitalWrite(ledPin, !digitalRead(ledPin));  // Toggle the LED state
  }
}

void initAP() {
  WiFi.config(ip);

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
      ;
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}

void clientListen(String &networkName, String &networkPass) {
  WiFiClient client = server.available();  // listen for incoming clients

  if (client) {                    // if you get a client,
    Serial.println("new client");  // print a message out the serial port
    String currentLine = "";       // make a String to hold incoming data from the client
    String request = "";
    bool currentLineIsBlank = true;

    while (client.connected()) {  // loop while the client's connected
      if (client.available()) {   // if there's bytes to read from the client,
        char c = client.read();   // read a byte, then
        Serial.write(c);          // print it out the serial monitor

        if (c == '\n' && currentLineIsBlank) {
          // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
          // and a content-type so the client knows what's coming, then a blank line:
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();

          // the content of the HTTP response follows the header:
          client.print("<!DOCTYPE html><html>");
          client.print("<head><title>Arduino Station Web Server</title></head>");
          client.print("<body>");
          client.print("<h1>UNDP Arduino Station 1.1</h1>");
          client.print("<h3>Enter WiFi Network Credentials</h3>");
          client.print("<form action=\"/submit\" method=\"GET\">");
          client.print("SSID name: <input type=\"text\" name=\"ssidname\"><br>");  //firstname = 9char; ssidname = 8 char
          client.print("SSID pass: <input type=\"text\" name=\"ssidpass\"><br>");  // lastname = 8 char ssidpass = 8 char
          client.print("<input type=\"submit\" value=\"Submit\"></form>");
          client.print("</body></html>");

          // The HTTP response ends with another blank line:
          client.println();
          break;
        }

        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
          currentLine = "";
        } else if (c == '\r') {
          // you got a carriage return character, ignore it
        } else {
          // you got a character on the current line
          currentLineIsBlank = false;
          currentLine += c;
        }

        // Check to see if the client request was "GET /submit"
        if (currentLine.startsWith("GET /submit")) {
          request = currentLine;
        }
      }
    }

    // Parse request
    if (request.length() > 0) {
      int ssidNamePos = request.indexOf("ssidname=");
      int ssidPassPos = request.indexOf("ssidpass=");
      String ssidName = "";
      String ssidPass = "";

      if (ssidNamePos != -1 && ssidPassPos != -1) {
        ssidNamePos += 9;  // Move past "ssidname="
        int ssidNameEndPos = request.indexOf('&', ssidNamePos);
        if (ssidNameEndPos != -1) {
          ssidName = request.substring(ssidNamePos, ssidNameEndPos);
        }

        ssidPassPos += 9;  // Move past "ssidpass="
        int ssidPassEndPos = request.indexOf(' ', ssidPassPos);
        if (ssidPassEndPos != -1) {
          ssidPass = request.substring(ssidPassPos, ssidPassEndPos);
        }

        // Replace + with space (URL encoding)
        // ssidName.replace('+', ' ');
        // ssidPass.replace('+', ' ');

        ssidName = urlDecode(ssidName);
        ssidPass = urlDecode(ssidPass);

        networkName = ssidName;
        networkPass = ssidPass;
      }
    }

    // Close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

String urlDecode(const String& input) {
  String decoded = "";
  char temp[] = "00";
  unsigned int len = input.length();
  unsigned int i = 0;
  
  while (i < len) {
    char c = input[i];
    if (c == '%') {
      if (i + 2 < len) {
        temp[0] = input[i + 1];
        temp[1] = input[i + 2];
        decoded += (char)strtol(temp, nullptr, 16);
        i += 2;
      }
    } else if (c == '+') {
      decoded += ' ';
    } else {
      decoded += c;
    }
    i++;
  }
  return decoded;
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

bool checkWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Still connected...");
    return true;
  } else {
    Serial.println("Connection was terminated!");
    return false;
  }
}

bool connectToNetwork(String ssidName, String ssidPassword) {

  digitalWrite(ledPin, HIGH);

  Serial.println("I am in connect to network!");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssidName);

    WiFi.begin(ssidName.c_str(), ssidPassword.c_str());
    delay(10000);

    if (WiFi.status() == WL_CONNECTED) {
      return true;
    } else {
      Serial.println("Can't connect!");
      WiFi.end();
      return false;
    }
  }
}

float getAverageTemperature(float sumTemperatures) {
  return sumTemperatures / NUM_MEASUREMENTS;
}

float getAverageHumidity(float sumHumidities) {
  return sumHumidities / NUM_MEASUREMENTS;
}

bool writeToCloud(float averageTemperature, float averageHumidity) {

  if (checkWiFiConnection()) {
    // Connection is still alive. We can write the values.
    ThingSpeak.setField(1, averageTemperature);
    ThingSpeak.setField(2, averageHumidity);
    ThingSpeak.writeFields(channelNumber, writeApiKey);
    // Minimum timeout for requesting data to ThingSpeak is 30s, therefore:
    delay(30000);
    return true;
  } else {
    Serial.println("Connection lost.");
    return false;
  }
}
