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

// Motor Parameters
const int motor_ids[] = {1, 2, 3, 4, 5, 6};
const float gear_ratios[] = {6.75, 75, 75, 24, 33.91, 33.91};
const float k_factor[] = {166.67, 166.67, 166.67, 166.67, 63.27, 63.27};
const int steps_per_revolution = 200;

// HTML and CSS for the web page
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Motor Control</title>
    <style>
        body { font-family: 'Arial', sans-serif; background-color: #282c34; color: white; }
        h1 { text-align: center; }
        .control { margin: 20px; }
        input { width: 100px; }
        .status { font-size: small; color: #61dafb; }
    </style>
</head>
<body>
    <h1>Motor Control</h1>
    <div class="control">
        <label for="motor_id">Motor ID:</label>
        <input type="number" id="motor_id" min="1" max="6" value="1">
        <label for="angle">Angle (°):</label>
        <input type="number" id="angle" value="0">
        <button onclick="moveMotor()">Move Motor</button>
        <button onclick="readEncoder()">Read Encoder Position</button>
    </div>
    <div class="status" id="status"></div>
    <script>
        var connection = new WebSocket('ws://' + window.location.hostname + ':81/');
        connection.onmessage = function (event) {
            document.getElementById('status').innerHTML += event.data + '<br>';
        };
        function moveMotor() {
            var motorId = document.getElementById('motor_id').value;
            var angle = document.getElementById('angle').value;
            connection.send('M' + motorId + ' A' + angle);
            document.getElementById('status').innerHTML += 'Moving Motor ' + motorId + ' to ' + angle + '°<br>';
        }
        function readEncoder() {
            var motorId = document.getElementById('motor_id').value;
            connection.send('READ_ENCODER ' + motorId);
            document.getElementById('status').innerHTML += 'Reading Encoder for Motor ' + motorId + '<br>';
        }
    </script>
</body>
</html>
)rawliteral";

// Update OLED Display
void updateStatus(const String& message) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, message.c_str());
    u8g2.sendBuffer();
    webSocket.broadcastTXT(message.c_str());
}

// Function to convert angle to pulses
int angleToPulses(uint8_t motor_id, float angle) {
    float pulses_per_degree = (gear_ratios[motor_id - 1] * k_factor[motor_id - 1] * steps_per_revolution) / 360.0;
    return static_cast<int>(angle * pulses_per_degree);
}

// Function to send CAN message with CRC
void sendCanMessage(uint8_t motor_id, const uint8_t* data, size_t length) {
    uint8_t crc = calculate_crc(motor_id, data, length);
    uint8_t message[length + 1]; // +1 for CRC
    memcpy(message, data, length);
    message[length] = crc; // Append CRC

    CAN.sendMsgBuf(motor_id, 0, length + 1, message);
}

// Move Motor to Absolute Position
void moveMotorAbsolute(uint8_t motor_id, float angle) {
    int pulses = angleToPulses(motor_id, angle);
    uint8_t data[3] = {(pulses >> 8) & 0xFF, pulses & 0xFF, 0x00}; // Adjust data format as needed
    sendCanMessage(motor_id, data, sizeof(data));
    updateStatus("Motor " + String(motor_id) + " -> " + String(angle) + "°");
}

// Stop All Motors
void stopAllMotors() {
    for (int i = 0; i < 6; i++) {
        uint8_t data[1] = {0xF7};
        sendCanMessage(motor_ids[i], data, sizeof(data));
    }
    updateStatus("All Motors Stopped");
}

// Clear All Encoders
void clearAllEncoders() {
    for (int i = 0; i < 6; i++) {
        uint8_t data[1] = {0x92}; // Assuming 0x92 is the command to reset encoders
        sendCanMessage(motor_ids[i], data, sizeof(data));
    }
    updateStatus("Encoders Cleared");
}

// Function to calculate CRC
uint8_t calculate_crc(uint8_t can_id, const uint8_t* data, size_t length) {
    uint8_t crc = 0; // Initialize CRC
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // Simple XOR for CRC calculation
    }
    return crc; // Return calculated CRC
}

// Function to read encoder position
void readEncoderPosition(uint8_t motor_id) {
    uint8_t command = 0x30; // Command to read encoder position
    sendCanMessage(motor_id, &command, 1); // Send command with a DLC of 1 byte

    // Buffer for incoming data
    uint8_t len = 0;
    uint8_t response[8]; // Max CAN message size is 8 bytes
    uint32_t rxId;       // Variable to store received message ID

    // Wait for response with timeout handling
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) { // 1-second timeout
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
            CAN.readMsgBuf(&rxId, &len, response); // Read message
            if (rxId == motor_id && len == 8) { // Ensure message is for our motor & correct size

                // Extract carry value (bytes 1-2 as signed int16)
                int16_t carry = (response[1] | (response[2] << 8));

                // Extract encoder value (bytes 3-6 as signed int32)
                int32_t value = (response[3] | (response[4] << 8) | (response[5] << 16) | (response[6] << 24));

                updateStatus("M" + String(motor_id) + " Enc: " + String(value) + " Carry: " + String(carry));
                return;
            }
        }
    }
    updateStatus("No response from M" + String(motor_id));
}


// WebSocket Handler
void onWebSocketMessage(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_TEXT) { // Only process text messages
        String msg = String((char*)payload);
        msg = msg.substring(0, length);
        Serial.print("WebSocket Message: ");
        Serial.println(msg);

        int motor_id;
        float angle;
        if (sscanf(msg.c_str(), "M%d A%f", &motor_id, &angle) == 2) {
            moveMotorAbsolute(motor_id, angle);
            webSocket.sendTXT(num, "Motor " + String(motor_id) + " moved to " + String(angle) + "°");
        } else if (msg == "STOP_ALL") {
            stopAllMotors();
            webSocket.sendTXT(num, "All Motors Stopped");
        } else if (msg == "CLEAR_ENCODERS") {
            clearAllEncoders(); // Assuming this function is defined
            webSocket.sendTXT(num, "Encoders Cleared");
        } else if (msg.startsWith("READ_ENCODER")) {
            motor_id = msg.charAt(13) - '0'; // Assuming motor_id is sent as a single digit
            readEncoderPosition(motor_id);
            webSocket.sendTXT(num, "Reading Encoder for Motor " + String(motor_id));
        }
    }
}

// Setup Function
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    u8g2.begin();
    updateStatus("OLED Ready");
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    updateStatus(WiFi.localIP().toString());

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

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", htmlPage);
    });
    server.begin();
    webSocket.begin();
    webSocket.onEvent(onWebSocketMessage);
}

// Main Loop
void loop() {
    webSocket.loop();
}
