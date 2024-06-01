
// --------------------------------------------------------------------------------
// for nfc
#include <Wire.h>
#include <Adafruit_PN532.h>
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

// Define the keys for sector 2 (both key A and key B)
uint8_t keyA[] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7};
uint8_t accessBits[] = {0x78, 0x84, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t keyB[] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7};

// for wifi
#include <WiFi.h>
#include <HTTPClient.h>
#define WIFI_SSID "free_@ClassicTech"
#define WIFI_PASSWORD "Rajesh77%"

// for gps
#include <TinyGPS++.h>       // library for GPS module
#include <HardwareSerial.h>  // library for hardware serial
HardwareSerial gpsSerial(1); // Create a hardware serial port object
TinyGPSPlus gps;             // The TinyGPS++ object

float latitude, longitude; // Variables to store latitude and longitude

// for server
#define GPS_URL "http://192.168.254.7:8000/vehicle/test/"
#define NFC_URL "http://192.168.254.51:8000/vehicle/test/"

// for each device
int GPS_ID = 1;

// function for nfc reader
String nfc_reader()
{
  Serial.println("Found an NFC card!");
  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  if (success)
  {
    // Authenticate with the key A for sector 2
    success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 2 * 4 + 2, 0, keyA);

    if (success)
    {
      Serial.println("Authentication with Key A successful.");

      // Read data from block 2 of sector 2
      uint8_t data[16];
      unsigned char TextOutput[16];
      success = nfc.mifareclassic_ReadDataBlock(2 * 4 + 2, data);

      if (success)
      {
        String hexString = "";
        for (int i = 0; i < 16; i++)
        {
          char hexBuffer[3];                   // Buffer to hold the hexadecimal representation of each element
          sprintf(hexBuffer, "%02X", data[i]); // Convert data[i] to hexadecimal and store it in hexBuffer
          hexString += hexBuffer;              // Concatenate hexBuffer to hexString
        }
        return hexString; // Return the hex string
      }
      else
      {
        Serial.println("Failed to read custom data.");
      }
    }
    else
    {
      Serial.println("Authentication with Key A failed.");
    }

    delay(50);
  }
  return "error"; // Return an empty string if there's an error or no NFC card found
}

// function to connect network
void connectToWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to WiFi...");
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000)
  {
    delay(1000);
    Serial.println("Connecting...");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Connected to WiFi");
  }
  else
  {
    Serial.println("Connection failed.");
  }
}

// task handleer
TaskHandle_t Task1;
// second loop for gps
void Loop2(void *parameter)
{

  while (1)
  {
    Serial.println("GPS location sent");
    delay(5000);
  }

  // while (gpsSerial.available() > 0)
  // {
  //   gps.encode(gpsSerial.read());

  //   if (gps.location.isUpdated())
  //   {
  //     latitude = gps.location.lat();
  //     longitude = gps.location.lng();

  //     // Serial.print("Latitude: ");
  //     // Serial.println(latitude, 6);
  //     // Serial.print("Longitude: ");
  //     // Serial.println(longitude, 6);

  //     // Create JSON payload
  //     // Send the HTTP POST request
  //     if (WiFi.status() == WL_CONNECTED)
  //     {
  //       String jsonPayload = "{\"gps_id\":\"" + String(GPS_ID) + "\",\"Lat\":\"" + String(latitude, 6) + "\",\"Lng\":\"" + String(longitude, 6) + "\"}";
  //       HTTPClient http;
  //       http.begin(GPS_URL);
  //       http.addHeader("Content-Type", "application/json");

  //       int httpResponseCode = http.POST(jsonPayload);

  //       if (httpResponseCode > 0)
  //       {
  //         Serial.print("HTTP Response code: ");
  //         Serial.println(httpResponseCode);
  //       }
  //       else
  //       {
  //         Serial.print("Error on sending POST: ");
  //         Serial.println(httpResponseCode);
  //       }

  //       http.end();
  //     }
  //     else
  //     {
  //       Serial.println("WiFi Disconnected,connecting again...");
  //       connectToWiFi();
  //     }

  //     // Delay to avoid spamming the server with requests
  //     delay(5000); // Adjust the delay as needed
  //   }
  // }
}

void setup()
{
  Serial.begin(115200);

  // gps
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX

  // Connect to WiFi
  connectToWiFi();

  // nfc
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata)
  {
    Serial.print("Didn't find PN53x board");
    while (1)
      ;
  }

  nfc.SAMConfig();
  Serial.println("Waiting for an NFC card ...");

  // Create a task to handle the GPS data
  xTaskCreatePinnedToCore(
      Loop2,   /* Task function. */
      "Loop2", /* name of task. */
      100000,  /* Stack size of task */
      NULL,    /* parameter of the task */
      1,       /* priority of the task */
      &Task1,  /* Task handle to keep track of created task */
      0);      /* pin task to core 0 */
               /* pin task to core 0 */
}

void loop()
{

  // handle post request for nfc tag
  String nfc_hex_data = nfc_reader();
  if (nfc_hex_data != "error")
  {
    Serial.println("NFC Data: " + nfc_hex_data);
    gps.encode(gpsSerial.read());
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);

    // Create JSON payload
    // String jsonPayload = "{\"nfc_id\":\"" + nfc_hex_data + "\"}";
    String jsonPayload = "{\"nfc_id\":\"" + nfc_hex_data + "\",\"gps_id\":\"" + String(GPS_ID) + "\",\"Lat\":\"" + String(latitude, 6) + "\",\"Lng\":\"" + String(longitude, 6) + "\"}";

    // Send the HTTP POST request
    if (WiFi.status() == WL_CONNECTED)
    {
      HTTPClient http;
      http.begin(NFC_URL);
      http.addHeader("Content-Type", "application/json");

      int httpResponseCode = http.POST(jsonPayload);

      if (httpResponseCode > 0)
      {
        // TODO: REMOVE BELOW LATER
        String response = http.getString();
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        Serial.print("Response: ");
        Serial.println(response);
      }
      else
      {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
      }

      http.end();
    }
    else
    {
      Serial.println("WiFi Disconnected,connecting again...");
      connectToWiFi();
    }
  }

  delay(1000); // Adjust the delay as needed
}
