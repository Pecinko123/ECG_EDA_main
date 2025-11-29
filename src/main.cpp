#include "defines.h"
#include "protocentral_max30001.h"

#include <atomic>
#include <Esp.h>
#include <LSM6.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <mutex>
#include <semphr.h>
#include <ESP32Time.h>
#include <queue.h>
#include <FS.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#define SAMPLE_RATE MAX30001_RATE_256 // 512 samples per second

MAX30001 ecgSensor(VSPI_SS);


// ---- TASKS ----  creating 4 FreeRTOS tasks 
TaskHandle_t Task1ecgAcquisition;
TaskHandle_t Task2biozAcquisition;
TaskHandle_t Task3secondaryDataAcquisition;
TaskHandle_t Task4dataSending;

QueueHandle_t dataQueue; //queue for writing and reading primary and secondary data (tasks 1-3) and sending task 4 to influx or pc server

// ----TIME----
ESP32Time ESP32_time;


volatile bool ecgIntHappened = false;
volatile uint32_t intCouter = 0;
uint32_t dataCount = 0;

//using two separate interrupts... one for when ecg data is ready and other for bioz 
//(ready when ECG(BIOZ) FIFO is full up to EFIT(BFIT) samples set in MNGR_INT register)
void IRAM_ATTR ecgIntCall()
{
  intCouter++;
  ecgIntHappened = true;

}

void IRAM_ATTR biozIntCall(void *arg) //so far only want to read ecg then add bioz
{
  (void)arg; // Explicitly ignore parameter
  // ISR intentionally left blank
}

void setup() 
{
  Serial.begin(115200);
  delay(500); //has to wait a little for serial to connect... otherwise doesnt print some prints
  Serial.println();

  // Wait for a serial connection for up to 5 seconds.
  // This is crucial for ESP32-S3 and other boards with native USB.
  // It prevents logs from being cleared if no monitor is attached.
  for (int i = 0; i < 50 && !Serial; i++)
  {
    delay(100);
  }

  //Serial.setDebugOutput(true); //dont know what this does

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);


  //INITIALISE SPI
  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  //SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);           // CPOL = 0, CPHA = 0
  //SPI.setClockDivider(SPI_CLOCK_DIV16); // Selecting 1Mhz clock for SPI


   //INITIALISE THE SENSOR
  Serial.println("Initializing MAX30001...");
  max30001_error_t result = ecgSensor.begin();
  attachInterrupt(INTB_ECG_PIN, ecgIntCall, FALLING);

  pinMode(VSPI_SS, OUTPUT); 
  digitalWrite(VSPI_SS, HIGH);

  if (result != MAX30001_SUCCESS) {
      Serial.print("✗ Failed to initialize MAX30001. Error code: ");
      Serial.println(result);
      Serial.println("\nCheck connections:");
      Serial.println("  - SPI wiring (MISO, MOSI, SCLK, CS)");
      Serial.println("  - Power supply (VCC, GND)");
      Serial.println("  - CS pin configuration");
      while (1) {
          delay(1000);
      }
  }
  Serial.println("✓ MAX30001 initialized successfully");
    
    // Check if device is responding
    if (!ecgSensor.isConnected()) {
        Serial.println("✗ Device not responding on SPI bus");
        Serial.println("  Check CS pin and SPI connections");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ Device connected and responding");
    
    // Get device information
    max30001_device_info_t deviceInfo;
    ecgSensor.getDeviceInfo(&deviceInfo);
    Serial.print("✓ Device Info - Part ID: 0x");
    Serial.print(deviceInfo.part_id, HEX);
    Serial.print(", Revision: 0x");
    Serial.println(deviceInfo.revision, HEX);
    
    Serial.println("\nStarting ECG + BioZ acquisition (interrupt-driven)...");


    // Start combined ECG and BioZ monitoring
    result = ecgSensor.startECGBioZ(SAMPLE_RATE);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start acquisition. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ ECG and BioZ acquisition started");

    ecgSensor.max30001SetInterrupts(EN_EINT | 0x01); //musí byť explixitne povolené v setupe... nestačí vo funkcii beginECGBioz
}

void loop() 
{
  // if (ecgIntHappened == true)
  // {
  //   ecgIntHappened = false;
  //   max30001_ecg_sample_t ecgSample;
  //   int32_t ecg_value = 0;

  //   // Get ECG sample
  //   ecgSensor.getECGSample(&ecgSample);
  //   ecgSensor.clearFIFO();
  //   ecg_value = ecgSample.ecg_sample;
  //   int32_t ecg_microvolts = ecgSensor.convertECGToMicrovolts(ecg_value, MAX30001_ECG_GAIN_160);

  //   // Print sample data (sparse output to avoid overwhelming serial)
  //   if (dataCount % 1 == 0) {  // Print every 16th sample (~8 per second at 128 SPS)
  //       Serial.print("ECG: ");
  //       Serial.print(ecg_microvolts);
  //       Serial.print(" | Interrupts: ");
  //       Serial.println(intCouter);

  //   }

  // }

  if (ecgIntHappened) {
    ecgSensor.max30001ServiceAllInterrupts();  // Fills s32ECGData[] buffer
    ecgIntHappened = false;
    
    // Process ALL buffered samples
    for (int i = 0; i < ecgSensor.ecgSamplesAvailable; i++) {
      int32_t raw = ecgSensor.s32ECGData[i];
      int32_t uv = ecgSensor.convertECGToMicrovolts(raw, MAX30001_ECG_GAIN_160);
      
      dataCount++;
      if (dataCount % 1 == 0) {
        Serial.print("ECG: ");
        Serial.print(uv);
        Serial.print(" | Count: ");
        Serial.println(dataCount);
      }
    }
    ecgSensor.clearFIFO();
    
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield CPU
  }
}

