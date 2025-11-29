#include <cstdint>

#define SDA_pin 6
#define SCL_pin 5

#define VSPI_MISO 14
#define VSPI_MOSI 13
#define VSPI_SCLK 11
#define VSPI_SS 10

#define INTB_ECG_PIN 15
#define INTB2_bioz_PIN 16

#define GREEN_LED_PIN 36
#define RED_LED_PIN 37

#define CHUNK_SEND_SIZE 100
#define CHUNK_SEND_SIZE_SECONDARY CHUNK_SEND_SIZE / 10


// struct DataStruct
// {
//     int32_t ecg_data_arr[CHUNK_SEND_SIZE];
//     int32_t bioz_data_arr[CHUNK_SEND_SIZE];
// };

// union DataUnion
// {
//     struct DataStruct mainData;
// };

// struct SocketSendStruct
// {
//     uint8_t isInitialData; // 1 if is initial data
//     uint8_t batteryLevel;
//     DataUnion dataUnion;
// };