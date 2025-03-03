#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <mcp_can.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

// Wi-Fi Credentials
const char* ssid = "Triton9";
const char* password = "88888888";

// MCP2515 CAN Bus Configuration
#define CAN_CS 4   // GPIO3
#define CAN_INT 3  // GPIO4
MCP_CAN CAN(CAN_CS);

// OLED Configuration (I2C)
#define SDA_PIN 5
#define SCL_PIN 6
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Web Server & WebSocket
AsyncWebServer server(80);
WebSocketsServer webSocket(81);

// Function: Update Status on OLED
void updateStatus(const String& message) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, message.c_str());
    u8g2.sendBuffer();
}

// Setup Function
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    u8g2.begin();
    updateStatus("OLED Ready");

    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        
        String ip = WiFi.localIP().toString();
        int secondDot = ip.indexOf('.', ip.indexOf('.') + 1);
        String shortIP = ip.substring(secondDot);
        updateStatus(shortIP);
    } else {
        updateStatus("WiFi Failed");
        Serial.println("WiFi Connection Failed.");
    }

    // Initialize SPI with correct pins
    SPI.begin(10, 8, 7, CAN_CS);  // SCK=D10, MISO=D8, MOSI=D7, CS=D3
    pinMode(CAN_CS, OUTPUT);
    digitalWrite(CAN_CS, HIGH);

    // Initialize CAN Module
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        updateStatus("CAN Bus Failed");
        while (1);
    }
    CAN.setMode(MCP_NORMAL);
    updateStatus("CAN Ready");
    Serial.println("CAN Bus Initialized.");

    // Web Server Route (Basic Test)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.println("Serving Web Page...");
        request->send(200, "text/html", "<h1>Hello from ESP32</h1>");
    });

    // Start Web Server
    server.begin();
    Serial.println("Web Server Started!");

    // Start WebSocket Server
    webSocket.begin();
    Serial.println("WebSocket Started!");

    webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        if (type == WStype_TEXT) {
            String msg = String((char*)payload);
            msg = msg.substring(0, length);
            Serial.print("WebSocket Message: ");
            Serial.println(msg);

            int motor_id;
            float angle;
            if (sscanf(msg.c_str(), "M%d A%f", &motor_id, &angle) == 2) {
                updateStatus("Motor " + String(motor_id) + " -> " + String(angle) + "°");
            } else if (msg == "STOP_ALL") {
                updateStatus("All Motors Stopped");
            } else if (msg == "CLEAR_ENCODERS") {
                updateStatus("Encoders Cleared");
            }
            webSocket.sendTXT(num, "ACK");
        }
    });
}

// Main Loop
void loop() {
    webSocket.loop();
}
