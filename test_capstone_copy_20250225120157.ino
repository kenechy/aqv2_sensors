#include <SoftwareSerial.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <DFRobot_OxygenSensor.h>


SoftwareSerial pmsSerial(17, 16);


#define DHTPIN 4 // 
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);


#define OXYGEN_SENSOR_I2C_ADDRESS 0x73
DFRobot_OxygenSensor oxygenSensor; 


const char* ssid = "AQV2";
const char* password = "1234567890";


String sensorsApiUrl = "https://kohjcrdirmvamsjcefew.supabase.co/rest/v1/sensors";

String apiKey = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImtvaGpjcmRpcm12YW1zamNlZmV3Iiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTcyNzMzMDYxMywiZXhwIjoyMDQyOTA2NjEzfQ.dcjFj_XWSg_Zq8BJQSnI_SfqzjtuG98cu3nZSIzgfBo";
String location = "";
int locationId = 3;

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
    dht.begin();

    // Initialize the oxygen sensor
    if (!oxygenSensor.begin(OXYGEN_SENSOR_I2C_ADDRESS)) {
        Serial.println("Failed to initialize Oxygen sensor!");
        while (1); 
    }

    connectToWiFi();

    timeClient.begin();
    timeClient.setTimeOffset(8 * 3600); 

    IPAddress dnsServerPrimary(8, 8, 8, 8);  
    IPAddress dnsServerSecondary(8, 8, 4, 4);  
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, dnsServerPrimary, dnsServerSecondary);

    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
    printDNSConfig();

    syncTime();

    fetchLocation();

    lastRestartCheck = millis(); 
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

    if (timeClient.getMinutes() % 5 == 0 && timeClient.getSeconds() == 0 && !locationFetched) {
        Serial.println("Fetching location...");
        fetchLocation();

        Serial.println("Sending data...");
        sendData();

        locationFetched = true;
    }

    if (timeClient.getMinutes() % 5 != 0) {
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
    String deviceUrl = "https://kohjcrdirmvamsjcefew.supabase.co/rest/v1/locations?locationId=eq." + String(locationId);
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
        jsonPayload += "\"locationId\":" + String(locationId) + ",";

        bool hasData = false; 

        // --- Particulate Matter Data ---
        if (pmsSerial.available() > 0) {
            while (pmsSerial.peek() != 0x42) {
                pmsSerial.read();
            }

            byte packet[32];
            pmsSerial.readBytes(packet, 32);

            if (verifyChecksum(packet, 30)) {
                int pm25 = makeWord(packet[10], packet[11]);
                int pm10 = makeWord(packet[12], packet[13]);

                Serial.println("PM2.5: " + String(pm25));  
                Serial.println("PM10: " + String(pm10));    

                
                if (pm25 >= 0 && pm10 >= 0) {
                    jsonPayload += "\"pm25\":" + String(pm25) + ",";
                    jsonPayload += "\"pm25Remarks\":\"" + getPM25Remarks(pm25) + "\",";
                    jsonPayload += "\"pm10\":" + String(pm10) + ",";
                    jsonPayload += "\"pm10Remarks\":\"" + getPM10Remarks(pm10) + "\",";
                    hasData = true;
                } else {
                    Serial.println("Invalid PM data, skipping transmission.");  
                    return; 
                }
            } else {
                Serial.println("Checksum failed, skipping transmission."); 
                return;
            }
        }

        // --- Temperature and Humidity Data ---
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        Serial.println("Temperature: " + String(temperature));  // Debugging
        Serial.println("Humidity: " + String(humidity));        // Debugging

        if (!isnan(temperature) && !isnan(humidity)) {
            jsonPayload += "\"temperature\":" + String(temperature) + ",";
            jsonPayload += "\"temperatureRemarks\":\"" + getTemperatureRemarks(temperature) + "\",";
            jsonPayload += "\"humidity\":" + String(humidity) + ",";
            jsonPayload += "\"humidityRemarks\":\"" + getHumidityRemarks(humidity) + "\",";
            hasData = true;
        } else {
            Serial.println("Invalid temperature/humidity data, skipping transmission."); // Debugging
            return;
        }

        // --- Oxygen Level Data ---
        uint8_t numReadings = 10;
        float oxygenLevel = oxygenSensor.getOxygenData(numReadings); 
        Serial.println("Oxygen Level: " + String(oxygenLevel));  // Debugging

        if (!isnan(oxygenLevel)) {
            jsonPayload += "\"oxygen\":" + String(oxygenLevel) + ",";
            jsonPayload += "\"oxygenRemarks\":\"" + getOxygenRemarks(oxygenLevel) + "\"";
            hasData = true;
        } else {
            Serial.println("Invalid oxygen data, skipping transmission."); // Debugging
            return;
        }

        jsonPayload += "}";

        // Send data only if we have valid data in jsonPayload
        if (hasData) {
            Serial.println("Sending payload: " + jsonPayload); // Debugging

            bool success = sendDataToServer(jsonPayload, sensorsApiUrl);
            if (success) {
                Serial.println("Data sent successfully");
            } else {
                Serial.println("Failed to send data");
            }
        } else {
            Serial.println("No valid data to send.");
        }
    } else {
        Serial.println("WiFi not connected, skipping data send.");
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
  } else if (pm25 > 55 && pm25 <= 91) {
    return "Accutely Unhealthy";
  } else if (pm25 > 91) {
    return "Emergency";
  } else {
    return "Invalid PM2.5 Value";
  }
}

String getPM10Remarks(float pm10) {
  if (pm10 >= 0 && pm10 <= 54) {
    return "Good";
  } else if (pm10 > 54 && pm10 <= 154) {
    return "Fair";
  } else if (pm10 > 154 && pm10 <= 254) {
    return "Unhealthy";
  } else if (pm10 > 254 && pm10 <= 354) {
    return "Very Unhealthy";
  } else if (pm10 > 354 && pm10 <= 424) {
    return "Accutely Unhealthy";
  } else if (pm10 > 424 && pm10 <= 504) {
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
  if (temperature >= 5 && temperature <= 33) {
    return "Good";
  } else if (temperature > 33 && temperature <= 41) {
    return "Caution";
  } else if (temperature > 41 && temperature <= 54) {
    return "Danger";
  } else if (temperature > 54 ) {
    return "Extremely Danger";
  } else {
    return "Invalid temperature Value";
  }
}

String getOxygenRemarks(float oxygenPercentage) {
  if (oxygenPercentage >= 19.5 && oxygenPercentage <= 23.5) {
    return "Good";
  } else if (oxygenPercentage < 19.5) {
    return "Poor";
  } else {
    return "Invalid oxygenPercentage Value";
  }
}


void restartDevice() {
  Serial.println("Restarting device...");
  ESP.restart();  // Restart the ESP32
}