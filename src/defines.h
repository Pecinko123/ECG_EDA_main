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

#define AP_NAME "ESP32_AP"
#define SERVER_UNAVAILABLE_RESTART_COUNTER "restartCounter"
#define START_CONFIG_SERVER "startConfig"

// TIME
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 3600
#define DAYLIGHT_OFFSET_SEC 3600

// SOCKETS
#define CHUNK_SEND_SIZE 100
#define CHUNK_SEND_SIZE_SECONDARY CHUNK_SEND_SIZE / 10
#define SERVER_CONNECTION_FAILS_MAX_COUNT 5
#define MAX_RESTART_COUNT_TO_SETUP_AP 3

#define BIOZ_RATIO 4

#pragma pack(1)
struct DataStruct
{
    int32_t ecg_data_arr[CHUNK_SEND_SIZE];
    int32_t bioz_data_arr[CHUNK_SEND_SIZE / BIOZ_RATIO];
    int16_t imu_acc_x[CHUNK_SEND_SIZE_SECONDARY];
    int16_t imu_acc_y[CHUNK_SEND_SIZE_SECONDARY];
    int16_t imu_acc_z[CHUNK_SEND_SIZE_SECONDARY];
    int16_t imu_gyro_x[CHUNK_SEND_SIZE_SECONDARY];
    int16_t imu_gyro_y[CHUNK_SEND_SIZE_SECONDARY];
    int16_t imu_gyro_z[CHUNK_SEND_SIZE_SECONDARY];
    uint64_t time[CHUNK_SEND_SIZE_SECONDARY];
};

struct InitialDataStructure
{
    char participant[50];
    char position[50];
    uint16_t device_id;
};

union DataUnion
{
    struct DataStruct mainData;
    struct InitialDataStructure initialData;
};

struct SocketSendStruct
{
    uint8_t isInitialData; // 1 if is initial data
    uint8_t batteryLevel;
    DataUnion dataUnion;
};
#pragma pack()