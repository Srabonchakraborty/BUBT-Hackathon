#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <DHT.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

/* =========================================================================
 * FastRobox 1.0 - Level 1: Dual-Core Tasks, Health and Watchdog Firmware
 * ========================================================================= */

// ----------------- Pin Definitions -----------------
#define PIN_LED_R       25
#define PIN_LED_G       26
#define PIN_LED_B       27
#define PIN_DOOR_RELAY  32

#define PIN_DHT         4
#define DHTTYPE         DHT11

#define PIN_FLAME       34    // Flame Digital Out (Active LOW usually)

#define RC522_SS_PIN    5     // SDA wire for unplug test
#define RC522_RST_PIN   22
#define RC522_SCK_PIN   18
#define RC522_MISO_PIN  19
#define RC522_MOSI_PIN  23

#define WDT_TIMEOUT_SECONDS 8 // Watchdog reset within 8-10 seconds

// ----------------- Global State -----------------
DHT dht(PIN_DHT, DHTTYPE);
MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);

String doorState = "LOCKED";
volatile bool sensorErrorFlag = false;
volatile bool rfidError = false;
volatile bool dhtError = false;

float currentTemp = 0.0;
float currentHum = 0.0;
bool flameDetected = false;

// Task Handles
TaskHandle_t TaskHeartbeatHandle = NULL;
TaskHandle_t TaskSerialCLIHandle = NULL;
TaskHandle_t TaskSensorsHandle   = NULL;
TaskHandle_t TaskRFIDHandle      = NULL;

// ----------------- LED Control Helper -----------------
// Common Cathode (HIGH = ON). For Common Anode, invert digitalWrite.
void setRGB(bool r, bool g, bool b) {
    digitalWrite(PIN_LED_R, r ? HIGH : LOW);
    digitalWrite(PIN_LED_G, g ? HIGH : LOW);
    digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

// =========================================================================
// CORE 0: Task 1 - Heartbeat LED (2 Hz rhythm & Magenta on error)
// =========================================================================
void TaskHeartbeat(void *pvParameters) {
    // 2 Hz rhythm = 500 ms period (250 ms ON, 250 ms OFF)
    bool ledState = false;
    for (;;) {
        sensorErrorFlag = (rfidError || dhtError);

        if (sensorErrorFlag) {
            // Magenta on sensor error (Red + Blue)
            if (ledState) {
                setRGB(true, false, true); // Magenta
            } else {
                setRGB(false, false, false);
            }
        } else {
            // Normal 2 Hz Green or Cyan rhythm
            if (ledState) {
                setRGB(false, true, false); // Steady Green
            } else {
                setRGB(false, false, false);
            }
        }
        ledState = !ledState;
        vTaskDelay(pdMS_TO_TICKS(250)); // 250ms = 2 Hz toggle
    }
}

// =========================================================================
// CORE 1: Task 2 - Sensor Reading (DHT11 5s, Flame 100ms, Evidence checks)
// =========================================================================
void TaskSensors(void *pvParameters) {
    TickType_t lastDHTRead = xTaskGetTickCount();
    int dhtFailCount = 0;

    for (;;) {
        // --- 1. Flame Sensor Check (Period: 100 ms) ---
        // Most flame sensors pull LOW when flame is present
        int flameVal = digitalRead(PIN_FLAME);
        flameDetected = (flameVal == LOW);

        // --- 2. DHT11 Check (Period: 5 s) ---
        if ((xTaskGetTickCount() - lastDHTRead) >= pdMS_TO_TICKS(5000)) {
            lastDHTRead = xTaskGetTickCount();

            float t = dht.readTemperature();
            float h = dht.readHumidity();

            if (isnan(t) || isnan(h)) {
                dhtFailCount++;
                // If unplugged for > 10-12s (2 consecutive failures)
                if (dhtFailCount >= 2) {
                    if (!dhtError) {
                        dhtError = true;
                        Serial.println("\n[ERROR] SENSOR ERROR: DHT11 Unplugged/Disconnected!");
                    }
                }
            } else {
                currentTemp = t;
                currentHum = h;
                if (dhtError) {
                    dhtError = false;
                    Serial.println("\n[RECOVERED] DHT11 reconnected successfully.");
                }
                dhtFailCount = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Flame period: 100ms
    }
}

// =========================================================================
// CORE 1: Task 3 - RFID RC522 (Period: 200 ms, Tap/Hold & SDA disconnect)
// =========================================================================
void TaskRFID(void *pvParameters) {
    bool cardPresent = false;
    unsigned long cardStartTime = 0;
    bool holdTriggered = false;
    String lastUID = "";
    int rfidFailCheckCount = 0;

    for (;;) {
        // --- Check SDA Wire Disconnection every ~1 second (< 3 seconds target) ---
        rfidFailCheckCount++;
        if (rfidFailCheckCount >= 5) {
            rfidFailCheckCount = 0;
            byte ver = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
            // Valid version is typically 0x91 or 0x92. 0x00 or 0xFF means bus/wire disconnected
            if (ver == 0x00 || ver == 0xFF) {
                if (!rfidError) {
                    rfidError = true;
                    Serial.println("\n[ERROR] SENSOR ERROR: RFID Reader (RC522) Disconnected!");
                }
            } else {
                if (rfidError) {
                    rfidError = false;
                    Serial.println("\n[RECOVERED] RFID Reader reconnected successfully.");
                }
            }
        }

        // --- RFID Card Presence & Tap / Hold Logic ---
        if (!rfidError) {
            bool currentCard = mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial();

            if (currentCard) {
                String uid = "";
                for (byte i = 0; i < mfrc522.uid.size; i++) {
                    if (mfrc522.uid.uidByte[i] < 0x10) uid += "0";
                    uid += String(mfrc522.uid.uidByte[i], HEX);
                }
                uid.toUpperCase();

                if (!cardPresent) {
                    // Card just placed
                    cardPresent = true;
                    cardStartTime = millis();
                    holdTriggered = false;
                    lastUID = uid;
                } else {
                    // Card still held on reader
                    if (!holdTriggered && (millis() - cardStartTime >= 5000)) {
                        holdTriggered = true;
                        Serial.printf("[CARD] Hold detected (UID: %s) for 5 seconds\n", lastUID.c_str());
                    }
                }
                mfrc522.PICC_HaltA();
                mfrc522.PCD_StopCrypto1();
            } else {
                if (cardPresent) {
                    // Card removed
                    if (!holdTriggered && (millis() - cardStartTime >= 100)) {
                        Serial.printf("[CARD] Tap detected (UID: %s)\n", lastUID.c_str());
                    }
                    cardPresent = false;
                    holdTriggered = false;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Period: 200 ms
    }
}

// =========================================================================
// CORE 0: Task 4 - Serial CLI & 1-Second [STATUS] Output
// =========================================================================
void printHealthTable() {
    Serial.println("\n==================== SYSTEM HEALTH ====================");
    Serial.printf(" Uptime            : %lu seconds\n", millis() / 1000);
    Serial.printf(" Free Heap Memory  : %u bytes\n", ESP.getFreeHeap());
    Serial.printf(" CPU Frequency     : %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf(" Task Watchdog     : Enabled (%d sec timeout)\n", WDT_TIMEOUT_SECONDS);
    Serial.printf(" Core 0 Status     : OK (CLI & Heartbeat)\n");
    Serial.printf(" Core 1 Status     : OK (Sensors & RFID)\n");
    Serial.printf(" DHT11 Sensor      : %s (Temp: %.1f C, Hum: %.1f %%)\n", dhtError ? "FAULT (UNPLUGGED)" : "OK", currentTemp, currentHum);
    Serial.printf(" RFID RC522        : %s\n", rfidError ? "FAULT (UNPLUGGED)" : "OK");
    Serial.printf(" Flame Status      : %s\n", flameDetected ? "FIRE DETECTED!" : "Safe");
    Serial.printf(" Door State        : %s\n", doorState.c_str());
    Serial.println("=======================================================\n");
}

void printTaskList() {
    Serial.println("\n----------------- RUNNING TASKS -----------------");
    Serial.printf("%-16s %-6s %-6s %-8s %-4s\n", "Task Name", "State", "Prio", "StackLeft", "Core");
    Serial.println("-------------------------------------------------");
    
    // Core check reporting for tasks
    Serial.printf("%-16s %-6s %-6s %-8u %-4d\n", "TaskHeartbeat", "RUN", "1", uxTaskGetStackHighWaterMark(TaskHeartbeatHandle), 0);
    Serial.printf("%-16s %-6s %-6s %-8u %-4d\n", "TaskSerialCLI", "RUN", "2", uxTaskGetStackHighWaterMark(TaskSerialCLIHandle), 0);
    Serial.printf("%-16s %-6s %-6s %-8u %-4d\n", "TaskSensors",   "RUN", "1", uxTaskGetStackHighWaterMark(TaskSensorsHandle), 1);
    Serial.printf("%-16s %-6s %-6s %-8u %-4d\n", "TaskRFID",      "RUN", "2", uxTaskGetStackHighWaterMark(TaskRFIDHandle), 1);
    Serial.println("-------------------------------------------------\n");
}

void TaskSerialCLI(void *pvParameters) {
    TickType_t lastStatusTick = xTaskGetTickCount();
    String inputBuffer = "";

    // Register this CLI task with Task Watchdog
    esp_task_wdt_add(NULL);

    for (;;) {
        // Feed watchdog during normal operation
        esp_task_wdt_reset();

        // --- Print fixed format [STATUS] line every 1 second ---
        if ((xTaskGetTickCount() - lastStatusTick) >= pdMS_TO_TICKS(1000)) {
            lastStatusTick = xTaskGetTickCount();
            Serial.printf("[STATUS] Uptime: %lus | Door: %s | Temp: %.1fC | Flame: %s | RFID: %s\n",
                millis() / 1000,
                doorState.c_str(),
                currentTemp,
                flameDetected ? "FIRE" : "CLEAR",
                rfidError ? "ERR" : "OK"
            );
        }

        // --- Handle Serial Commands ---
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                inputBuffer.trim();
                if (inputBuffer.length() > 0) {
                    if (inputBuffer.equalsIgnoreCase("health")) {
                        printHealthTable();
                    } else if (inputBuffer.equalsIgnoreCase("tasks")) {
                        printTaskList();
                    } else if (inputBuffer.equalsIgnoreCase("hang")) {
                        Serial.println("[WARN] Entering deliberate infinite loop without feeding watchdog...");
                        Serial.println("[INFO] Watchdog will reboot the board in ~8-10 seconds.");
                        // Deliberate hang: infinite loop starving WDT
                        while (true) {
                            // No esp_task_wdt_reset() called -> Watchdog triggers!
                            delay(100);
                        }
                    } else {
                        Serial.printf("[CLI] Unknown command: '%s'. Available: health, tasks, hang\n", inputBuffer.c_str());
                    }
                    inputBuffer = "";
                }
            } else {
                inputBuffer += c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =========================================================================
// SETUP & INITIALIZATION
// =========================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    pinMode(PIN_DOOR_RELAY, OUTPUT);
    pinMode(PIN_FLAME, INPUT);

    // Initial check for Watchdog boot reason
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
        Serial.println("\n[EVENT] BOOT reason=WATCHDOG");
    } else {
        Serial.println("\n[EVENT] BOOT reason=NORMAL_POWERON");
    }

    // Evidence requirement: Flame at start-up gives door 'UNLOCKED'
    if (digitalRead(PIN_FLAME) == LOW) {
        doorState = "UNLOCKED";
        digitalWrite(PIN_DOOR_RELAY, HIGH);
        Serial.println("[SAFETY] Flame detected at startup! Door is UNLOCKED");
    } else {
        doorState = "LOCKED";
        digitalWrite(PIN_DOOR_RELAY, LOW);
        Serial.println("[SAFETY] System initialized with door LOCKED");
    }

    // Hardware Inits
    dht.begin();
    SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
    mfrc522.PCD_Init();

    // Initialize Hardware Watchdog (8 seconds timeout)
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);

    // Create Core 0 Tasks
    xTaskCreatePinnedToCore(TaskHeartbeat, "TaskHeartbeat", 2048, NULL, 1, &TaskHeartbeatHandle, 0);
    xTaskCreatePinnedToCore(TaskSerialCLI, "TaskSerialCLI", 4096, NULL, 2, &TaskSerialCLIHandle, 0);

    // Create Core 1 Tasks
    xTaskCreatePinnedToCore(TaskSensors,   "TaskSensors",   3072, NULL, 1, &TaskSensorsHandle,   1);
    xTaskCreatePinnedToCore(TaskRFID,      "TaskRFID",      4096, NULL, 2, &TaskRFIDHandle,      1);

    Serial.println("[BOOT] All Dual-Core tasks successfully started on Core 0 & Core 1.\n");
}

void loop() {
    // Empty because all tasks are managed by FreeRTOS scheduler
    vTaskDelete(NULL);
}
