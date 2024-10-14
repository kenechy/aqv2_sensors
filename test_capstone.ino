#include <SoftwareSerial.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <DFRobot_OxygenSensor.h>

// Define the pins for the Software Serial for PMS5003
SoftwareSerial pmsSerial(17, 16); // RX, TX for PMS5003

// Define the pin and sensor type for the DHT22
#define DHTPIN 4 // Pin where the DHT22 is connected
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define the I2C address for the DFRobot Oxygen sensor
#define OXYGEN_SENSOR_I2C_ADDRESS 0x73
DFRobot_OxygenSensor oxygenSensor; // Initialize the oxygen sensor object

// Replace with your network credentials
const char* ssid = "FTTx_579D";
const char* password = "9D78579D";

// URLs for your Supabase tables
String sensorsApiUrl = "https://kohjcrdirmvamsjcefew.supabase.co/rest/v1/tblsensors";
// API Key
String apiKey = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImtvaGpjcmRpcm12YW1zamNlZmV3Iiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTcyNzMzMDYxMywiZXhwIjoyMDQyOTA2NjEzfQ.dcjFj_XWSg_Zq8BJQSnI_SfqzjtuG98cu3nZSIzgfBo";
String location = "";
int device_id = 2;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 8 * 3600, 60000); // Use "asia.pool.ntp.org" for Philippines

unsigned long lastRestartCheck = 0;
unsigned long restartInterval = 1800000; // 30 minutes in milliseconds

bool verifyChecksum(byte* packet, int length) {
    unsigned int checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum += packet[i];
    }
    return (checksum == makeWord(packet[length], packet[length + 1]));
}

void setup() {
    Serial.begin(9600);
    pmsSerial.begin(9600);
    dht.begin(); // Initialize DHT22

    // Initialize the oxygen sensor
    if (!oxygenSensor.begin(OXYGEN_SENSOR_I2C_ADDRESS)) {
        Serial.println("Failed to initialize Oxygen sensor!");
        while (1); // Stay in infinite loop if sensor initialization fails
    }

    connectToWiFi();

    timeClient.begin();
    timeClient.setTimeOffset(8 * 3600); // 8 hours * 3600 seconds/hour

    IPAddress dnsServerPrimary(8, 8, 8, 8);  // Google's primary DNS server
    IPAddress dnsServerSecondary(8, 8, 4, 4);  // Google's alternative DNS server
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, dnsServerPrimary, dnsServerSecondary);

    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
    printDNSConfig();

    syncTime();

    fetchLocation();

    lastRestartCheck = millis(); // Initialize the restart timer
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, attempting to reconnect...");
        connectToWiFi();
    }

    timeClient.update();

    if (timeClient.getSeconds() == 0) {
        Serial.print("Current local time: ");
        Serial.print(timeClient.getFormattedTime());
        Serial.println();
    }

    static bool locationFetched = false;

    if (timeClient.getMinutes() % 2 == 0 && timeClient.getSeconds() == 0 && !locationFetched) {
        Serial.println("Fetching location...");
        fetchLocation();

        Serial.println("Sending data...");
        sendData();

        locationFetched = true;
    }

    if (timeClient.getMinutes() % 2 != 0) {
        locationFetched = false;
    }

    if (millis() - lastRestartCheck >= restartInterval) {
        Serial.println("Restarting device...");
        restartDevice();
    }

    delay(1000);
}

void connectToWiFi() {
    WiFi.begin(ssid, password);

    int maxAttempts = 20;
    int attemptCount = 0;

    while (WiFi.status() != WL_CONNECTED && attemptCount < maxAttempts) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
        attemptCount++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to WiFi");
        Serial.print("Connected to WiFi network with IP Address: ");
        Serial.println(WiFi.localIP());
        printDNSConfig();
    } else {
        Serial.println("Failed to connect to WiFi.");
    }
}

void syncTime() {
    Serial.println("Synchronizing time...");
    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    while (!time(nullptr)) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("Time synchronized!");
    delay(1000);
    Serial.print(timeClient.getFormattedTime());
    Serial.println();
}

void fetchLocation() {
    String deviceUrl = "https://kohjcrdirmvamsjcefew.supabase.co/rest/v1/tbldevice?device_id=eq." + String(device_id);
    HTTPClient http;
    http.begin(deviceUrl);
    http.addHeader("apikey", apiKey);
    http.addHeader("Authorization", "Bearer " + apiKey);

    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        String payload = http.getString();
        Serial.print("Location response: ");
        Serial.println(payload);

        DynamicJsonDocument doc(1024);
        deserializeJson(doc, payload);
        
        if (doc.size() > 0) {
            location = doc[0]["location"].as<String>();
        }

        doc.clear();
        Serial.print("Location: ");
        Serial.println(location);

    } else {
        Serial.print("Failed to fetch location. HTTP response code: ");
        Serial.println(httpResponseCode);
    }

    http.end();
}

bool sendDataToServer(const String& data, const String& apiUrl) {
    HTTPClient http;
    http.begin(apiUrl);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", apiKey);
    http.addHeader("Authorization", "Bearer " + apiKey);

    int httpResponseCode = http.POST(data);

    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        Serial.print("Server Response: ");
        Serial.println(http.getString());

        http.end();
        return true;
    } else {
        Serial.print("HTTP Request failed. Error code: ");
        Serial.println(httpResponseCode);
    }

    http.end();
    return false;
}



void sendData() {
    if (WiFi.status() == WL_CONNECTED) {
        String jsonPayload = "{";
        jsonPayload += "\"device_id\":" + String(device_id) + ",";
        jsonPayload += "\"location\":\"" + location + "\",";

        // --- Particulate Matter Data ---
        if (pmsSerial.available() > 0) {
            while (pmsSerial.peek() != 0x42) {
                pmsSerial.read(); // Clear junk data
            }

            byte packet[32];
            pmsSerial.readBytes(packet, 32);

            if (verifyChecksum(packet, 30)) {
                int pm25 = makeWord(packet[10], packet[11]);
                int pm10 = makeWord(packet[12], packet[13]);

                jsonPayload += "\"pm25\":" + String(pm25) + ",";
                jsonPayload += "\"pm25remarks\":\"" + getPM25Remarks(pm25) + "\",";
                jsonPayload += "\"pm10\":" + String(pm10) + ",";
                jsonPayload += "\"pm10remarks\":\"" + getPM10Remarks(pm10) + "\",";
            } else {
                jsonPayload += "\"pm25\":null, \"pm25remarks\":null, \"pm10\":null, \"pm10remarks\":null,";
            }
        }

        // --- Temperature and Humidity Data ---
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        if (!isnan(temperature) && !isnan(humidity)) {
            jsonPayload += "\"temperature\":" + String(temperature) + ",";
            jsonPayload += "\"temperature_remarks\":\"" + getTemperatureRemarks(temperature) + "\",";
            jsonPayload += "\"humidity\":" + String(humidity) + ",";
            jsonPayload += "\"humidity_remarks\":\"" + getHumidityRemarks(humidity) + "\",";
        } else {
            jsonPayload += "\"temperature\":null, \"temperature_remarks\":null,";
            jsonPayload += "\"humidity\":null, \"humidity_remarks\":null,";
        }

        // --- Oxygen Level Data ---
        uint8_t numReadings = 1; // Change this number if you want more readings
        float oxygenLevel = oxygenSensor.getOxygenData(numReadings); // Pass the required argument
        if (!isnan(oxygenLevel)) {
            jsonPayload += "\"oxygen_concentration\":" + String(oxygenLevel) + ",";
            jsonPayload += "\"oxygen_remarks\":\"" + getOxygenRemarks(oxygenLevel) + "\"";
        } else {
            jsonPayload += "\"oxygen\":null, \"oxygen_remarks\":null";
        }

        jsonPayload += "}";

        sendDataToServer(jsonPayload, sensorsApiUrl);
    } else {
        Serial.println("WiFi is not connected. Cannot send data.");
    }
}



void printDNSConfig() {
    Serial.println("DNS Configuration:");
    Serial.println("Primary DNS: " + String(WiFi.dnsIP(0)));
    Serial.println("Secondary DNS: " + String(WiFi.dnsIP(1)));
}


String getPM25Remarks(float pm25) {
  if (pm25 >= 0 && pm25 <= 25) {
    return "Good";
  } else if (pm25 > 25 && pm25 <= 35) {
    return "Fair";
  } else if (pm25 > 35 && pm25 <= 45) {
    return "Unhealthy";
  } else if (pm25 > 45 && pm25 <= 55) {
    return "Very Unhealthy";
  } else if (pm25 > 55 && pm25 <= 90) {
    return "Severely Unhealthy";
  } else if (pm25 > 90) {
    return "Emergency";
  } else {
    return "Invalid PM2.5 Value";
  }
}

String getPM10Remarks(float pm10) {
  if (pm10 >= 0 && pm10 <= 50) {
    return "Good";
  } else if (pm10 > 50 && pm10 <= 100) {
    return "Fair";
  } else if (pm10 > 100 && pm10 <= 150) {
    return "Unhealthy";
  } else if (pm10 > 150 && pm10 <= 200) {
    return "Very Unhealthy";
  } else if (pm10 > 200 && pm10 <= 300) {
    return "Severely Unhealthy";
  } else if (pm10 > 300) {
    return "Emergency";
  } else {
    return "Invalid PM10 Value";
  }
}

String getHumidityRemarks(float humidity) {
  if (humidity >= 30 && humidity <= 60) {
    return "Good";
  } else if (humidity > 60 && humidity <= 70) {
    return "Fair";
  } else if (humidity > 25 && humidity <= 30) {
    return "Fair";
  } else if (humidity > 70) {
    return "Poor";
  } else if (humidity < 25) {
    return "Poor";
  } else {
    return "Invalid humidity Value";
  }
}

String getTemperatureRemarks(float temperature) {
  if (temperature >= 27 && temperature <= 37) {
    return "Good";
  } else if (temperature > 33 && temperature <= 41) {
    return "Caution";
  } else if (temperature > 42 && temperature <= 54) {
    return "Danger";
  } else if (temperature > 55 ) {
    return "Extremely Danger";
  } else {
    return "Invalid temperature Value";
  }
}

String getOxygenRemarks(float oxygenPercentage) {
  if (oxygenPercentage >= 0 && oxygenPercentage <= 50) {
    return "Good";
  } else if (oxygenPercentage > 51) {
    return "Fair";
  } else {
    return "Invalid oxygenPercentage Value";
  }
}

void restartDevice() {
  Serial.println("Restarting device...");
  ESP.restart();  // Restart the ESP32
}
