#include "defines.h"
#include "protocentral_max30001.h"
#include "config_helpers.h"
#include "log_utils.h"

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

#define SAMPLE_RATE MAX30001_RATE_256

using namespace std;
/// INSERT DEVICE ID HERE
#define DEVICE_ID 4

//conditional headers
#ifdef ESP32          //"Only compile this section IF the ESP32 board is selected"
#include <SPIFFS.h>   //SPI Flash File System — a lightweight filesystem that stores files directly in the ESP32's flash memory
#endif
#include <HTTPClient.h>

MAX30001 ecgSensor(VSPI_SS);
ConfigHelpers configHelpers = ConfigHelpers();
LSM6 imu;

static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED; //creates a spinlock mutex in UNLOCKED mode for thread-safe access to shared data on the ESP32.

char ap_name[50] = {0};        // zero-initialize
char server_address[20] = {0}; // same for these
char participant[50] = {0};
char position[50] = {0};

// ---- WIFI ----
int serverUnavailableRestartCounter;
WiFiManagerParameter custom_server_address("server", "sever ip address", server_address, 20);
WiFiManagerParameter custom_participant("participant", "participant", participant, 50);
WiFiManagerParameter custom_position("position", "position", position, 50);

WiFiManager wifiManager;
WiFiClient client;

Preferences preferences;

// ---- TASKS ----  creating 4 FreeRTOS tasks 
TaskHandle_t Task1ecgBioZAcquisition;
TaskHandle_t Task2secondaryDataAcquisition;
TaskHandle_t Task3dataSending;

QueueHandle_t dataQueue; //queue for writing and reading primary and secondary data (tasks 1-3) and sending task 4 to influx or pc server

SemaphoreHandle_t adcReadySem = xSemaphoreCreateBinary();
SemaphoreHandle_t readSupportDataSem = xSemaphoreCreateBinary();

// Atomic variables - thread-safe without explicit locks
// Used to track write positions in data buffers across multiple tasks
atomic<int16_t> supportDataWriteIndex(0);  // Index for IMU/compass buffer
atomic<int16_t> ecgDataWriteIndex(0);      // Index for ECG/EMG buffer
// These prevent race conditions when Task1 and Task2 update simultaneously

struct DataStruct dataStructNow; //structure to which you write the data which is then sent to queue (queue for reading)
struct InitialDataStructure initialData;

// ----TIME----
ESP32Time ESP32_time;

bool startMeasuring = false;
int port = 8888;

bool initialDataSent = false;
bool shouldSaveConfig = false;

volatile bool ecgIntHappened = false;
volatile uint32_t intCouter = 0;
uint32_t dataCount = 0;

/// Sends directly to serial if availible, if not save to log file
void generalPrint(const char *format, ...)
{
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (Serial)
  {
    Serial.println(buf);
  }
  else
  {
    // logToFile expects a format string and variable arguments
    logToFile("%s", buf);
  }
}

void saveConfigCallback()
{
  configHelpers.writeConfigFile(&preferences, server_address, participant, position, &custom_server_address, &custom_participant, &custom_position);
}

void getApName()
{
  snprintf(ap_name, sizeof(ap_name), "ESP32AP_V2_%d_B", DEVICE_ID);
}

// ADJUST FOR MAX1704X fuel gauge
// uint8_t readBatteryLevel()
// {
//   return map(analogRead(1), 1400, 2600, 0, 100);
// }

void dataReadingSecondaryTask(void *pvParameters);
void dataReadingTask(void *pvParameters);
void dataSendingTask(void *pvParameters);
void maxConnectionCheck(max30001_error_t result);
void ecgEdaStartCheck(max30001_error_t result);
void printChunkDebug(const DataStruct& d);
void printChunkDebugEcg(const DataStruct& d);



// ---- INTERRUPTS ----
//using two separate interrupts... one for when ecg data is ready and other for bioz 
//(ready when ECG(BIOZ) FIFO is full up to EFIT(BFIT) samples set in MNGR_INT register)
void IRAM_ATTR ecgIntCall()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(adcReadySem, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    Serial.setDebugOutput(true);

  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
  }
  else
  {
    if (Serial)
    { // Only print and clear logs if a serial monitor is connected
      printAndClearLogs();
    }
    else
    {
      logToFile("No serial monitor connected, skipping log print.");
    }
  }

  generalPrint("Begin initialization");
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
  dataQueue = xQueueCreate(10, sizeof(struct DataStruct)); // Queue to hold data batches
  preferences.begin("main", false);

  configHelpers.readConfigFile(&preferences, server_address, participant, position);

  custom_server_address.setValue(server_address, sizeof(server_address));
  custom_participant.setValue(participant, sizeof(participant));
  custom_position.setValue(position, sizeof(position));

  WiFi.mode(WIFI_STA);

  // reset settings - for testing
  // wifiManager.resetSettings();

  // ----- Reset prefs for testing ----
  // Uncomment to erase stored wifi and positions
  // Do only once, then comment it again and run, esp will forget wifi and position
  // wifiManager.erase();
  // preferences.clear();

  wifiManager.setConnectTimeout(5);
  wifiManager.setConfigPortalTimeout(60);

  // set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setBreakAfterConfig(true);

  // add all your parameters here
  wifiManager.addParameter(&custom_server_address);
  wifiManager.addParameter(&custom_participant);
  wifiManager.addParameter(&custom_position);
  getApName();

  // This is used when only the server is missing, but wifi is still available
  bool startWithAP = preferences.getBool(START_CONFIG_SERVER, false);
  if (startWithAP)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(500);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(500);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(500);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(500);
    preferences.putBool(START_CONFIG_SERVER, false);
    generalPrint("starting config portal");
    wifiManager.startConfigPortal(ap_name);
  }

  configHelpers.readConfigFile(&preferences, server_address, participant, position);
  if (server_address[0] == '\0')
  {
    generalPrint("No server address found, clearing wifi manager");
    wifiManager.erase();
  }

  if (!wifiManager.autoConnect(ap_name))
  {
    generalPrint("failed to connect and hit timeout");
    delay(500);
    // reset and try again, or maybe put it to deep sleep
    preferences.end();
    ESP.restart();
  }

  /// ConnectedToWifi
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(GREEN_LED_PIN, LOW);

  initialData.device_id = DEVICE_ID;
  strcpy(initialData.participant, participant);
  strcpy(initialData.position, position);

  //---- TIME ----  //
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  struct tm timeInfo;

  if (getLocalTime(&timeInfo))
  {
    ESP32_time.setTimeStruct(timeInfo);
  }
  else
  {
    generalPrint("Failed to obtain time");
  }

  generalPrint("socketIO setup on: %s:%d\n", server_address, port);
  generalPrint("data size: %u", sizeof(DataStruct));

  generalPrint("I2C imu and compass setup");

  Wire.begin(SDA_pin, SCL_pin);

  imu.init();
  imu.enableDefault();

  //INITIALISE SPI
  generalPrint("SPI and MAX30001");
  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);           // CPOL = 0, CPHA = 0
  SPI.setClockDivider(SPI_CLOCK_DIV16); // Selecting 1Mhz clock for SPI


   //INITIALISE THE SENSOR
  Serial.println("Initializing MAX30001...");
  max30001_error_t result = ecgSensor.begin();
  maxConnectionCheck(result);

  pinMode(VSPI_SS, OUTPUT); 
  digitalWrite(VSPI_SS, HIGH);
  attachInterrupt(INTB_ECG_PIN, ecgIntCall, FALLING);
  

    // Start combined ECG and BioZ monitoring
  result = ecgSensor.startECGBioZ(SAMPLE_RATE);
  ecgEdaStartCheck(result);
  


  ecgSensor.max30001SetInterrupts(EN_EINT | 0x01); //musí byť explixitne povolené v setupe... nestačí vo funkcii beginECGBioz //changed setinterrupts to also change mngr bit

  Serial.println("Start ADS measuring thread");

  // ---- THREAD DEFINITION ----
  xTaskCreatePinnedToCore(
      dataReadingTask,          /* Task function. */
      "dataReadingTask",        /* name of task. */
      10000,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      configMAX_PRIORITIES - 1, /* priority of the task */
      &Task1ecgBioZAcquisition,            /* Task handle to keep track of created task */
      1);

  Serial.println("Start secondary data reading thread");

  xTaskCreatePinnedToCore(
      dataReadingSecondaryTask,   /* Task function. */
      "dataReadingSecondaryTask", /* name of task. */
      10000,                      /* Stack size of task */
      NULL,                       /* parameter of the task */
      configMAX_PRIORITIES - 2,   /* priority of the task */
      &Task2secondaryDataAcquisition,       /* Task handle to keep track of created task */
      1);

  Serial.println("Start data sending thread");

  xTaskCreatePinnedToCore(
      dataSendingTask,          /* Task function. */
      "dataSendingTask",        /* name of task. */
      10000,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      configMAX_PRIORITIES - 3, /* priority of the task */
      &Task3dataSending,            /* Task handle to keep track of created task */
      0);

  digitalWrite(RED_LED_PIN, LOW);

  generalPrint("Initialization is done");


  //DEBUG... printing registers at the end of a setup
  Serial.println("=== MAX30001 Registers ===");
  ecgSensor.printRegister24( "CNFG_GEN",  CNFG_GEN);
  ecgSensor.printRegister24( "CNFG_ECG",  CNFG_ECG);
  ecgSensor.printRegister24( "CNFG_BIOZ", CNFG_BIOZ);
  ecgSensor.printRegister24( "MNGR_INT",  MNGR_INT);
  ecgSensor.printRegister24( "EN_INT",    EN_INT);
  Serial.println("==========================");

}

void loop() 
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}


void ecgEdaStartCheck(max30001_error_t result)
{
  if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start acquisition. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ ECG and BioZ acquisition started");
}

void dataReadingTask(void *pvParameters)
{
  generalPrint("Data reading task started");
  
  // Debug timing variables
  static uint32_t lastQueueTime = 0; 
  
  // Counts total ECG samples to keep the 8:1 rhythm across different interrupts
  static uint32_t totalEcgSampleCount = 0; 
  int bioZBufferIndex = 0;

  // Interrupt Setup: Fire when 16 ECG samples are ready.
  // 0xF... = EFIT 15 (16 samples).
  // 0x.0.. = BFIT 0 (1 sample threshold).
  // This reduces CPU wakeups to approx 32Hz (512 / 16).

  while (true)
  {
    // Wait for the interrupt (Fires approx every 31ms at 512Hz)
    if (xSemaphoreTake(adcReadySem, portMAX_DELAY) == pdTRUE)
    {
      // We process the batch of 10 samples waiting in the FIFO.
      // Since we set EFIT=9, we know at least 10 samples are ready.
      for (int i = 0; i < 10; i++) 
      {
        max30001_ecg_sample_t ecgSample;
        
        // 1. Get ECG Sample
        // High-level API: Handles SPI read and FIFO pointer advance automatically.
        ecgSensor.getECGSample(&ecgSample);

        if (ecgSample.sample_valid) 
        {
           // Store ECG
           int32_t ecgVal = ecgSensor.convertECGToMicrovolts(ecgSample.ecg_sample, MAX30001_ECG_GAIN_80);
           
           if (ecgDataWriteIndex < CHUNK_SEND_SIZE) {
              dataStructNow.ecg_data_arr[ecgDataWriteIndex] = ecgVal;
           }

           // 2. Handle BioZ (Ratio 8:1)
           // We check if this specific ECG sample is the "8th" one
           if (totalEcgSampleCount % BIOZ_RATIO == 0)
           {
               max30001_bioz_sample_t biozSample;
               
               // Attempt to read BioZ (one SPI read)
               ecgSensor.getBioZSample(&biozSample);
               
               if (biozSample.sample_valid) {
                   if (bioZBufferIndex < (CHUNK_SEND_SIZE / BIOZ_RATIO)) {
                       dataStructNow.bioz_data_arr[bioZBufferIndex] = biozSample.bioz_sample;
                       bioZBufferIndex++;
                   }
               }
           }

           // 3. IMU Trigger (matches your snippet)
           // Trigger every 10th sample for external support data
           if (ecgDataWriteIndex % 10 == 0) {
              xSemaphoreGive(readSupportDataSem);
           }
           
           // Increment Counters
           totalEcgSampleCount++;
           ecgDataWriteIndex++;

           // 4. Check Chunk Complete (100 samples)
           if (ecgDataWriteIndex == CHUNK_SEND_SIZE)
           {
              // --- DEBUG & TIMING LOGIC (From your snippet) ---
              uint32_t now = millis();
              uint32_t timeSinceLastQueue = now - lastQueueTime; 
              
              generalPrint("MAX READ 100th sample: %lu ms", timeSinceLastQueue); // Debug timing
              //printChunkDebugEcg(dataStructNow); // Debug data dump
              
              // Send Data
              if (xQueueSend(dataQueue, (void *)&dataStructNow, portMAX_DELAY) != pdPASS) {
                generalPrint("ERROR: Data could not queue");
              }
              
              // Reset Indices
              ecgDataWriteIndex = 0;
              bioZBufferIndex = 0;
              supportDataWriteIndex = 0;
              lastQueueTime = now; // Update timestamp
           }
        }
      }
      
      // Clear FIFO tracking (Safety measure, matches your snippet logic)
      ecgSensor.clearFIFO(); 
    }
  }
}


void dataReadingSecondaryTask(void *pvParameters)
{
  struct timeval tv;
  while (true)
  {
    if (xSemaphoreTake(readSupportDataSem, portMAX_DELAY) == pdTRUE)
    {
      //compass.read();
      imu.read();
      tm timeStruct = ESP32_time.getTimeStruct();
      gettimeofday(&tv, NULL);

      // dataStructNow.compass_x[supportDataWriteIndex] = compass.getX();
      // dataStructNow.compass_y[supportDataWriteIndex] = compass.getY();
      // dataStructNow.compass_z[supportDataWriteIndex] = compass.getZ();
      dataStructNow.imu_gyro_x[supportDataWriteIndex] = imu.g.x;
      dataStructNow.imu_gyro_y[supportDataWriteIndex] = imu.g.y;
      dataStructNow.imu_gyro_z[supportDataWriteIndex] = imu.g.z;
      dataStructNow.imu_acc_x[supportDataWriteIndex] = imu.a.x;
      dataStructNow.imu_acc_y[supportDataWriteIndex] = imu.a.y;
      dataStructNow.imu_acc_z[supportDataWriteIndex] = imu.a.z;
      dataStructNow.time[supportDataWriteIndex] = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);

      supportDataWriteIndex++;
    } 
  }
}
void dataSendingTask(void *pvParameters)
{
  struct SocketSendStruct structureToSend; // {0} wasnt there before... maybe garmabage data due to that
  generalPrint("Data sending task started");
  generalPrint("Connecting to server %s : %d", server_address, port);
  generalPrint("With device  %d,participant: %s position: %s", DEVICE_ID, participant, position);
  int connectionFailCounter = 0;
  IPAddress ip;

  while (true)
  {
    if (!ip.fromString(server_address))
    {
      generalPrint("Bad server IP: '%s'", server_address);
      // decide: start portal or restart
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (!client.connect(ip, port))
    {
      generalPrint("Connection to %s:%d failed.", server_address, port);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(200);
      digitalWrite(RED_LED_PIN, LOW);
      delay(100);
      connectionFailCounter++;
      if (connectionFailCounter > SERVER_CONNECTION_FAILS_MAX_COUNT)
      {
        int restartCounter = preferences.getInt(SERVER_UNAVAILABLE_RESTART_COUNTER, 0);
        generalPrint("Server not found restart counter: %d", restartCounter);

        if (restartCounter > MAX_RESTART_COUNT_TO_SETUP_AP)
        {
          digitalWrite(RED_LED_PIN, HIGH);
          digitalWrite(GREEN_LED_PIN, HIGH);
          delay(1000);
          preferences.putInt(SERVER_UNAVAILABLE_RESTART_COUNTER, 0);
          preferences.putBool(START_CONFIG_SERVER, true);
          generalPrint("Connection failed too many times, restarting and forcing AP start.");
          preferences.end();
          ESP.restart();
        }
        else
        {
          restartCounter += 1;
          generalPrint("Incrementing server unavailable counter to %d", restartCounter);
          preferences.putInt(SERVER_UNAVAILABLE_RESTART_COUNTER, restartCounter);
        }

        generalPrint("Connection failed too many times, restarting.");
        preferences.end();
        ESP.restart();
      }
      continue;
    }
    if (initialDataSent == false)
    {
      generalPrint("Sending initial data.");
      structureToSend.isInitialData = 1; // 1 for initial data
      structureToSend.dataUnion.initialData = initialData;

      client.write((char *)&structureToSend, sizeof(SocketSendStruct));
      client.flush();
      initialDataSent = true;
      char ack[50] = {0};
      client.readBytes(ack, sizeof(ack) - 1);
      generalPrint("Received initial data ACK: %s", ack);
    }

    generalPrint("Connected to server, starting data transmission.");
    digitalWrite(GREEN_LED_PIN, HIGH);
    if (client)
      structureToSend.isInitialData = 0; // 0 for false
    // Send data to the server
    while (client.connected())
    { // loop while the client's connected
      if (xQueueReceive(dataQueue, &structureToSend.dataUnion.mainData, portMAX_DELAY) == pdPASS)
      {
        //structureToSend.batteryLevel = readBatteryLevel();
        printChunkDebugEcg(structureToSend.dataUnion.mainData); //debug function for printing data which is the sent
        client.write((char *)&structureToSend, sizeof(SocketSendStruct));
        client.flush();
      }
    }
    digitalWrite(GREEN_LED_PIN, LOW);
    generalPrint("Client disconnected.");
  }
}

void maxConnectionCheck(max30001_error_t result)
{
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
}

void printChunkDebug(const DataStruct& d) {
    // One single println statement, unrolled loops, using '\n' for new lines
    Serial.println(
        "---- SENDING CHUNK ----\n" + 
        
        // 1. ECG Data (Indices 0-11)
        String("ECG: ") + 
        String(d.ecg_data_arr[0]) + " " + String(d.ecg_data_arr[1]) + " " + String(d.ecg_data_arr[2]) + " " + 
        String(d.ecg_data_arr[3]) + " " + String(d.ecg_data_arr[4]) + " " + String(d.ecg_data_arr[5]) + " " + 
        String(d.ecg_data_arr[6]) + " " + String(d.ecg_data_arr[7]) + " " + String(d.ecg_data_arr[8]) + " " + 
        String(d.ecg_data_arr[9]) + " " + String(d.ecg_data_arr[10]) + " " + String(d.ecg_data_arr[11]) + 
        "\n" +

        "BIOZ: " +
        String(d.bioz_data_arr[0]) + " " + String(d.bioz_data_arr[1]) + " " + String(d.bioz_data_arr[2]) + " " + 
        String(d.bioz_data_arr[3]) + " " + String(d.bioz_data_arr[4]) + " " + String(d.bioz_data_arr[5]) + " " + 
        String(d.bioz_data_arr[6]) + " " + String(d.bioz_data_arr[7]) + " " + String(d.bioz_data_arr[8]) + " " + 
        String(d.bioz_data_arr[9]) + " " + String(d.bioz_data_arr[10]) + " " + String(d.bioz_data_arr[11]) +
        "\n" +
    
        // 2. IMU ACC Data (Indices 0-11, formatted as (x,y,z))
        "IMU ACC: " + 
        "(" + String(d.imu_acc_x[0]) + "," + String(d.imu_acc_y[0]) + "," + String(d.imu_acc_z[0]) + ") " +
        "(" + String(d.imu_acc_x[1]) + "," + String(d.imu_acc_y[1]) + "," + String(d.imu_acc_z[1]) + ") " +
        "(" + String(d.imu_acc_x[2]) + "," + String(d.imu_acc_y[2]) + "," + String(d.imu_acc_z[2]) + ") " +
        "(" + String(d.imu_acc_x[3]) + "," + String(d.imu_acc_y[3]) + "," + String(d.imu_acc_z[3]) + ") " +
        "(" + String(d.imu_acc_x[4]) + "," + String(d.imu_acc_y[4]) + "," + String(d.imu_acc_z[4]) + ") " +
        "(" + String(d.imu_acc_x[5]) + "," + String(d.imu_acc_y[5]) + "," + String(d.imu_acc_z[5]) + ") " +
        "(" + String(d.imu_acc_x[6]) + "," + String(d.imu_acc_y[6]) + "," + String(d.imu_acc_z[6]) + ") " +
        "(" + String(d.imu_acc_x[7]) + "," + String(d.imu_acc_y[7]) + "," + String(d.imu_acc_z[7]) + ") " +
        "(" + String(d.imu_acc_x[8]) + "," + String(d.imu_acc_y[8]) + "," + String(d.imu_acc_z[8]) + ") " +
        "(" + String(d.imu_acc_x[9]) + "," + String(d.imu_acc_y[9]) + "," + String(d.imu_acc_z[9]) + ") " +
        "(" + String(d.imu_acc_x[10]) + "," + String(d.imu_acc_y[10]) + "," + String(d.imu_acc_z[10]) + ") " +
        "(" + String(d.imu_acc_x[11]) + "," + String(d.imu_acc_y[11]) + "," + String(d.imu_acc_z[11]) + ") " +
        "\n" +

        // 3. Time Data (Indices 0-11)
        "TIME: " + 
        String(d.time[0]) + " " + String(d.time[1]) + " " + String(d.time[2]) + " " + 
        String(d.time[3]) + " " + String(d.time[4]) + " " + String(d.time[5]) + " " + 
        String(d.time[6]) + " " + String(d.time[7]) + " " + String(d.time[8]) + " " + 
        String(d.time[9]) + " " + String(d.time[10]) + " " + String(d.time[11]) + 
        "\n-----------------------"
    );
}

void printChunkDebugEcg(const DataStruct& d) {
    // Single println, printing first 12 samples of ECG and BIOZ
    Serial.println(
        String("ECG: ") + 
        String(d.ecg_data_arr[0]) + " " + String(d.ecg_data_arr[1]) + " " + String(d.ecg_data_arr[2]) + " " + 
        String(d.ecg_data_arr[3]) + " " + String(d.ecg_data_arr[4]) + " " + String(d.ecg_data_arr[5]) + " " + 
        String(d.ecg_data_arr[6]) + " " + String(d.ecg_data_arr[7]) + " " + String(d.ecg_data_arr[8]) + " " + 
        String(d.ecg_data_arr[9]) + " " + String(d.ecg_data_arr[10]) + " " + String(d.ecg_data_arr[11]) +
        "\nBIOZ: " +
        String(d.bioz_data_arr[0]) + " " + String(d.bioz_data_arr[1]) + " " + String(d.bioz_data_arr[2]) + " " + 
        String(d.bioz_data_arr[3]) + " " + String(d.bioz_data_arr[4]) + " " + String(d.bioz_data_arr[5]) + " " + 
        String(d.bioz_data_arr[6]) + " " + String(d.bioz_data_arr[7]) + " " + String(d.bioz_data_arr[8]) + " " + 
        String(d.bioz_data_arr[9]) + " " + String(d.bioz_data_arr[10]) + " " + String(d.bioz_data_arr[11])
    );
}


