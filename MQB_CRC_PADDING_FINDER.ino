#include <FlexCAN_T4.h>

// Define the CRC algorithm to use
#define USE_CRC8_AUTOSAR
// #define USE_CRC8_SAE_J1850

#include "CRC.h"

#ifdef USE_CRC8_AUTOSAR
#define CRC8_FUNC calculateCRC8_AUTOSAR
#elif defined(USE_CRC8_SAE_J1850)
#define CRC8_FUNC calculateCRC8_SAE_J1850
#endif


// Define the platform (uncomment the one you are using)
//#define PQ46_PLATFORM
#define MQB_PLATFORM

// Set up CAN Bus
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> CANbus;

// Known CAN IDs for PQ46 platform
#ifdef PQ46_PLATFORM
const int IMMOBILIZER_ID = 0x3D0;        // Immobilizer status and control
const int INDICATORS_ID = 0x470;         // General indicator lights on the dashboard
const int LIGHTS_ID = 0x531;             // Control and status of lights and blinkers
const int DIESEL_ENGINE_ID = 0x480;      // Diesel engine specific messages
const int RPM_ID = 0x280;                // Engine RPM
const int SPEED_ID = 0x5A0;              // Vehicle speed data
const int ABS_ID = 0x1A0;                // ABS (Anti-lock Braking System) status and control
const int AIRBAG_ID = 0x050;             // Airbag system status
const int GEAR_ID = 0x540;               // Gear shifter position and status
const int CRUISE_CONTROL_ID = 0x288;     // Cruise control status and settings
const int WINDOW_CONTROL_FL_ID = 0x381;  // Front left window control
const int WINDOW_CONTROL_FR_ID = 0x3B5;  // Front right window control
const int WINDOW_CONTROL_RL_ID = 0x4BD;  // Rear left window control
const int WINDOW_CONTROL_RR_ID = 0x3B5;  // Rear right window control (same as FR window)
const int CLOCK_ID = 0x623;              // Current time (hour, minute, second)
const int DOOR_LOCK_ID = 0x281;          // Door lock/unlock status
const int STEERING_WHEEL_ID = 0x5BF;     // Multi-function steering wheel controls
#endif

// Known CAN IDs for MQB platform
#ifdef MQB_PLATFORM
const int AIRBAG_01_ID = 0x40;           // Airbag system status
const int KLEMMEN_STATUS_01_ID = 0x3C0;  // Ignition status
const int DIMMUNG_01_ID = 0x5F0;         // Backlight dimming
const int GATEWAY_72_ID = 0x3db;         // Gateway diagnostics
const int GATEWAY_76_ID = 0x3df;         // Gateway diagnostics
const int ESP_02_ID = 0x101;             // ESP (Electronic Stability Program) sensor data
const int ESP_10_ID = 0x116;             // ESP status and events
const int ESP_20_ID = 0x65D;             // Tire circumference adjustment
const int ESP_21_ID = 0xFD;              // Speed and distance data
const int KOMBI_01_ID = 0x30B;           // Instrument cluster (Kombi) status
const int MOTOR_04_ID = 0x107;           // Engine RPM
const int MOTOR_07_ID = 0x640;           // Oil temperature
const int MOTOR_09_ID = 0x647;           // Coolant temperature
const int MOTOR_14_ID = 0x3BE;           // Engine torque
const int MOTOR_18_ID = 0x670;           // Engine status
const int MOTOR_26_ID = 0x3c7;           // Engine data
const int MOTOR_CODE_01_ID = 0x641;      // Engine code
const int ESP_24_ID = 0x31B;             // Vehicle speed on instrument cluster
const int TSK_07_ID = 0x31E;             // Transmission status
const int LH_EPS_01_ID = 0x32A;          // Electric power steering status
const int RKA_01_ID = 0x663;             // Tire pressure monitoring system (TPMS) status
const int OBD_01_ID = 0x391;             // OBD diagnostics
const int WBA_03_ID = 0x394;             // Gear position
const int BLINKMODI_02_ID = 0x366;       // Turn signals (blinkers) status
const int MFSW_ID = 0x5BF;               // Multi-function steering wheel controls
const int TPMS_ID = 0x64A;               // TPMS data (ID 0x5F9 is also related to TPMS)
const int SWA_01_ID = 0x30F;             // Lane Change Assist (Spurwechselassistent)
const int PARKBRAKE_ID = 0x30d;          // Electronic parking brake status
const int LWR_AFS_01_ID = 0x395;         // Adaptive front lighting system (AFS) status
const int ESP_05_ID = 0x106;             // ESP control
const int LICHT_VORNE_01_ID = 0x658;     // Front lights status
const int LICHT_HINTEN_01_ID = 0x3D6;    // Rear lights status
const int LICHT_ANF_ID = 0x3D5;          // General lights status
const int DOOR_STATUS_ID = 0x583;        // Door status
const int OUTDOOR_TEMP_ID = 0x5e1;       // Outdoor temperature
const int DATE_ID = 0x17331110;          // Date and time information
const int ENGINE_04_ID = 0x107;          // Engine Control Module (ECM) data
const int IMMOBILIZER_ID = 0x3D0;        // Immobilizer
const int INDICATORS_ID = 0x470;         // General indicator lights
const int LIGHTS_ID = 0x531;             // Lights and blinkers
const int DIESEL_ENGINE_ID = 0x480;      // Diesel engine specific messages
const int RPM_ID = 0x280;                // RPM
const int SPEED_ID = 0x5A0;              // Speed
const int ABS_ID = 0x1A0;                // ABS
const int AIRBAG_ID = 0x050;             // Airbag system
const int GEAR_ID = 0x540;               // Gear shifter
const int CRUISE_CONTROL_ID = 0x288;     // Cruise control
#endif

// Define the CAN IDs to monitor
#ifdef PQ46_PLATFORM
const int CAN_IDs[] = { TEST_ID };
#elif defined(MQB_PLATFORM)
const int CAN_IDs[] = { MOTOR_04_ID, ESP_21_ID, ESP_24_ID, KOMBI_01_ID };
#endif

const int MESSAGE_LENGTH = 8;
uint8_t paddingByteTable[16] = {0};
bool foundPaddingBytes[16] = {false};

uint8_t findKey(const uint8_t *msgBody, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 1; i < len; i++) {
        crc ^= msgBody[i];
        uint8_t temp[1] = { crc };
        crc = CRC8_FUNC(temp, 1);
    }
    uint8_t firstByte[1] = { static_cast<uint8_t>(msgBody[0] ^ 0xFF) };
    uint8_t key = CRC8_FUNC(firstByte, 1) ^ crc;
    return key;
}

void setup() {
    Serial.begin(115200);
    CANbus.begin();
    CANbus.setBaudRate(500000);
    Serial.println("CAN BUS init ok!");
}

int foundCount = 0; // Counter for found padding bytes
bool allFound = false; // Flag to indicate all padding bytes are found

void loop() {
    if (allFound) {
        return;
    }

    CAN_message_t msg;
    if (CANbus.read(msg)) {
        for (int i = 0; i < sizeof(CAN_IDs) / sizeof(CAN_IDs[0]); i++) {
            if (msg.id == CAN_IDs[i] && msg.len == MESSAGE_LENGTH) {
                uint8_t counter = msg.buf[1] & 0x0F;
                if (!foundPaddingBytes[counter]) {
                    uint8_t key = findKey(msg.buf, MESSAGE_LENGTH);
                    paddingByteTable[counter] = key;
                    foundPaddingBytes[counter] = true;
                    foundCount++; // Increment the found count

                    // Print found information
                    Serial.print("Found padding byte from CAN ID: 0x");
                    Serial.print(msg.id, HEX);
                    Serial.print(" for counter ");
                    Serial.print(counter, HEX);
                    Serial.print(": 0x");
                    Serial.println(key, HEX);

                    // Check if all padding bytes are found
                    if (foundCount >= 16) {
                        allFound = true;
                        Serial.println("All padding bytes found. Complete padding byte table:");
                        displayPaddingBytes(); // Display the entire padding byte table
                        return;
                    }
                }
            }
        }
    }
}

void displayPaddingBytes() {
    Serial.println("Complete padding byte table:");
    for (int i = 0; i < 16; i++) {
        Serial.print("[");
        if (foundPaddingBytes[i]) {
            Serial.print("0x");
            Serial.print(paddingByteTable[i], HEX);
        } else {
            Serial.print("None");
        }
        Serial.print("]");
        if (i < 15) {
            Serial.print(", ");
        }
    }
    Serial.println();
}
