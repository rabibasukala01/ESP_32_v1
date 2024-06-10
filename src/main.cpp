
// --------------------------------------------------------------------------------
// for nfc
#include <Wire.h>
#include <Adafruit_PN532.h>
#include "SPIFFS.h"
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
String BASE_URL = "http://192.168.254.39:8000";
#define GPS_URL BASE_URL + "/vehicle/update_coords"
#define IN_URL BASE_URL + "/fare/scanned/in"
#define OUT_URL BASE_URL + "/fare/scanned/out"
#define MOBILE_URL_IN BASE_URL + "/fare/mobile_scanned/in"
#define MOBILE_URL_OUT BASE_URL + "/fare/mobile_scanned/out"

// for each device/bus/driver_account_id
int GPS_ID = 7;

// function for nfc reader
String nfc_reader()
{
  Serial.println("Tag found");
  Serial.println("Scanning NFC card...");
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

      // data is in HEX format [encypted by aes]
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
        Serial.println("Failed to Scan.");
      }
    }
    else
    {
      Serial.println("Authentication  failed.");
    }

    delay(1000);
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

void postRequest(String jsonPayload, String URL)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    // Send HTTP POST request
    HTTPClient http;
    http.begin(URL);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode > 0)
    {
      Serial.print("HTTP Response code:");
      Serial.println(httpResponseCode);
      String response = http.getString();
      Serial.print("Response message: ");
      Serial.println(response);
    }
    else
    {
      Serial.print("Error code:");
      Serial.println(httpResponseCode);
      String response = http.getString();
      Serial.print("Response message: ");
      Serial.println(response);
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi Disconnected, connecting again...");
    connectToWiFi(); // Reconnect to WiFi if disconnected
  }
}

// For storing NFC scan data
void storeScanData(String tagData, int scanCount)
{
  File file = SPIFFS.open("/scans.txt", FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.println(tagData + "," + String(scanCount));
  Serial.println("Stored scan data: " + tagData + "," + String(scanCount));
  file.close();
}

// For NFC scan count
int getScanCount(String tagData)
{
  File file = SPIFFS.open("/scans.txt", FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return -1;
  }
  while (file.available())
  {
    String line = file.readStringUntil('\n');
    if (line.startsWith(tagData))
    {
      file.close();
      return line.substring(tagData.length() + 1).toInt();
    }
  }
  file.close();
  return 0; // Tag not found
}

// For deleting NFC scan data
void deleteScanData(String tagData)
{
  File file = SPIFFS.open("/scans.txt", FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Create a temporary file to write data except the specified tag data
  File tempFile = SPIFFS.open("/temp.txt", FILE_WRITE);
  if (!tempFile)
  {
    Serial.println("Failed to open temp file for writing");
    file.close();
    return;
  }

  // Copy data from the original file to the temp file, excluding the specified tag data
  while (file.available())
  {
    String line = file.readStringUntil('\n');
    if (!line.startsWith(tagData))
    {
      tempFile.println(line);
    }
  }

  // Close both files
  file.close();
  tempFile.close();

  // Remove the original file
  SPIFFS.remove("/scans.txt");

  // Rename the temp file to the original file name
  SPIFFS.rename("/temp.txt", "/scans.txt");
}

// For updating TAG-NFC scan count
void updateScanCount(String tagData)
{
  int scanCount = getScanCount(tagData);
  if (scanCount == -1)
  {
    Serial.println("Error reading scan count");
    return;
  }
  if (scanCount == 0)
  {
    // First scan

    storeScanData(tagData, 1);
    Serial.println("First scan for tag: " + tagData);
    gps.encode(gpsSerial.read());
    // float first_lat = gps.location.lat();
    // float first_lng = gps.location.lng();
    // TODO : get the gps data
    float first_lat = 27.675814;
    float first_lng = 85.429938;
    String jsonPayload = "{\"nfc_id\":\"" + tagData + "\",\"gps_id\":\"" + String(GPS_ID) + "\",\"Lat\":\"" + String(first_lat, 6) + "\",\"Lng\":\"" + String(first_lng, 6) + "\"}";
    postRequest(jsonPayload, IN_URL);
  }
  else
  {
    // Second scan

    Serial.println("second scan for tag: " + tagData);

    gps.encode(gpsSerial.read());
    // float second_lat = gps.location.lat();
    // float second_lng = gps.location.lng();
    // TODO : get the gps data
    float second_lat = 27.671371;
    float second_lng = 85.420833;
    String jsonPayload = "{\"nfc_id\":\"" + tagData + "\",\"gps_id\":\"" + String(GPS_ID) + "\",\"Lat\":\"" + String(second_lat, 6) + "\",\"Lng\":\"" + String(second_lng, 6) + "\"}";
    Serial.println("Verifying payment...");
    postRequest(jsonPayload, OUT_URL);
    // delete the data
    deleteScanData(tagData);
  }
}

// FOR UPDATING SCAN COUNT FOR MOBILE DEVICE
void updateScanCountMOBILE(String tagData)
{
  int scanCount = getScanCount(tagData);
  if (scanCount == -1)
  {
    Serial.println("Error reading scan count");
    return;
  }
  if (scanCount == 0)
  // First scan
  {

    storeScanData(tagData, 1);
    Serial.println("First scan for tag: " + tagData);

    // coords in first scan
    gps.encode(gpsSerial.read());
    // float first_lat = gps.location.lat();
    // float first_lng = gps.location.lng();
    // TODO : get the gps data

    float first_lat = 27.675814;
    float first_lng = 85.429938;

    String jsonPayload = "{\"nfc_id\":\"" + tagData + "\",\"gps_id\":\"" + String(GPS_ID) + "\",\"Lat\":\"" + String(first_lat, 6) + "\",\"Lng\":\"" + String(first_lng, 6) + "\"}";
    postRequest(jsonPayload, MOBILE_URL_IN);
  }
  else
  // Second scan
  {
    Serial.println("second scan for tag: " + tagData);

    // coords in second scan
    gps.encode(gpsSerial.read());
    // float second_lat = gps.location.lat();
    // float second_lng = gps.location.lng();
    // TODO : get the gps data

    float second_lat = 27.671371;
    float second_lng = 85.420833;

    String jsonPayload = "{\"nfc_id\":\"" + tagData + "\",\"gps_id\":\"" + String(GPS_ID) + "\",\"Lat\":\"" + String(second_lat, 6) + "\",\"Lng\":\"" + String(second_lng, 6) + "\"}";
    Serial.println("Verifying payment...");
    postRequest(jsonPayload, MOBILE_URL_OUT);
    // delete the data
    deleteScanData(tagData);
  }
}

// this is responsible for reading the data from the mobile device response
// and converting it to plain string format [no hex, no encryptions]
String extract_payload(const byte *data, const long numBytes)
{
  // numBytes is the length of the data array with data and response code [0x90, 0x00=>success]
  String payload = "";
  Serial.print("Received data: ");
  for (int i = 0; i < 10; i++)
  {
    // Serial.print(data[i]);
    payload += data[i];
  }
  Serial.println(payload);
  return payload;
}

// task handleer
TaskHandle_t Task1;
// second loop for gps
void Loop2(void *parameter)
{
  for (;;)
  {
    // Read GPS data if available
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
    }

    // Check if GPS data is updated
    if (gps.location.isUpdated())
    {
      // Retrieve latitude and longitude
      latitude = gps.location.lat();
      longitude = gps.location.lng();

      // post
      String jsonPayload = "{\"gps_id\":\"" + String(GPS_ID) + "\",\"Lat\":\"" + String(latitude, 6) + "\",\"Lng\":\"" + String(longitude, 6) + "\"}";

      postRequest(jsonPayload, GPS_URL);

      // Delay to avoid spamming the server with requests
      vTaskDelay(pdMS_TO_TICKS(10000)); // Adjust the delay as needed
    }
    else
    {
      // GPS data not updated, wait for a while before checking again
      vTaskDelay(pdMS_TO_TICKS(5000)); // Adjust the delay as needed
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // Serial.println(GPS_URL);

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

  // for SPIFFS storage
  if (!SPIFFS.begin(true))
  {
    Serial.println("Failed to mount SPIFFS. Formatting...");
    SPIFFS.format();
    if (!SPIFFS.begin(true))
    {
      Serial.println("Failed to mount SPIFFS even after formatting.");
      return;
    }
    Serial.println("SPIFFS mounted after formatting.");
  }
  Serial.println("SPIFFS mounted successfully.");

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

  bool success;
  uint8_t responseLength = 32;

  success = nfc.inListPassiveTarget();

  if (success)
  {

    uint8_t selectApdu[] = {0x00,                                     /* CLA */
                            0xA4,                                     /* INS */
                            0x04,                                     /* P1  */
                            0x00,                                     /* P2  */
                            0x07,                                     /* Length of AID  */
                            0xF0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, /* AID defined on Android App */
                            0x00 /* Le  */};

    uint8_t response[32]; // Buffer to store the response
    success = nfc.inDataExchange(selectApdu, sizeof(selectApdu), response, &responseLength);
    if (success)
    // handle for mobile device
    {
      delay(25);
      Serial.print("Mobile device found");

      // extyract the payload
      String payload_tag = extract_payload(response, responseLength);

      // TODO: handle payload here
      updateScanCountMOBILE(payload_tag);
    }
    else
    // handle request for nfc TAG
    {
      delay(25);

      String nfc_hex_data = nfc_reader();
      if (nfc_hex_data != "error")
      {
        Serial.println("NFC Data: " + nfc_hex_data);
        // Call for update scan count
        updateScanCount(nfc_hex_data);
      }
    }
  }
  else
  {
    Serial.println("Scan Here!");
  }

  delay(1000); // Adjust the delay as needed
}
