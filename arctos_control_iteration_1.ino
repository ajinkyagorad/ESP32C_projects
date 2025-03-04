#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <mcp_can.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <Arduino.h> // Ensure you include Arduino.h for String and Serial

// Wi-Fi Credentials
const char* ssid = "Triton9";
const char* password = "88888888";

// MCP2515 CAN Bus Configuration
#define CAN_CS 4   // GPIO4
#define CAN_INT 3  // GPIO3
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
        <button onclick="scanCanBus()">Scan CAN Bus</button>
        <button onclick="enableMotors()">Enable Motors</button>
    </div>
    <div class="status" id="status"></div>
    <script>
        var connection = new WebSocket('ws://' + window.location.hostname + ':81/');

        connection.onopen = function () {
            console.log("WebSocket Connected");
        };

        connection.onmessage = function (event) {
            document.getElementById('status').innerHTML += event.data + '<br>';
        };

        function sendMessage(message) {
            if (connection.readyState === WebSocket.OPEN) {
                connection.send(message);
            } else {
                console.error("WebSocket not open");
            }
        }

        function moveMotor() {
            var motorId = document.getElementById('motor_id').value;
            var angle = document.getElementById('angle').value;
            sendMessage('M' + motorId + ' A' + angle);
            document.getElementById('status').innerHTML += 'Moving Motor ' + motorId + ' to ' + angle + '°<br>';
        }

        function readEncoder() {
            var motorId = document.getElementById('motor_id').value;
            sendMessage('READ_ENCODER ' + motorId);
            document.getElementById('status').innerHTML += 'Reading Encoder for Motor ' + motorId + '<br>';
        }

        function scanCanBus() {
            sendMessage('SCAN_CAN_BUS');
            document.getElementById('status').innerHTML += 'Scanning CAN Bus...<br>';
        }

        function enableMotors() {
            sendMessage('ENABLE_MOTORS');
            document.getElementById('status').innerHTML += 'Enabling Motors...<br>';
        }
    </script>

</body>
</html>
)rawliteral";

// Function to convert a byte array to a hex string
String bytesToHexString(const uint8_t* bytes, size_t length) {
    String hexString = "";
    for (size_t i = 0; i < length; i++) {
        if (bytes[i] < 0x10) {
            hexString += "0"; // Add leading zero for single digit hex
        }
        hexString += String(bytes[i], HEX);
        if (i < length - 1) {
            hexString += " "; // Add space between bytes
        }
    }
    return hexString;
}

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

// Function to calculate CRC
uint8_t calculate_crc(uint8_t can_id, const uint8_t* data, size_t length) {
    uint8_t crc = can_id; // Initialize CRC with the CAN ID
    for (size_t i = 0; i < length; i++) {
        crc += data[i]; // Sum each byte of data with the CRC
    }
    return crc & 0xFF; // Return calculated CRC, ensuring it's within 0-255
}

bool sendCanMessage2(uint8_t motor_id, const uint8_t* data, size_t length, uint8_t* response, size_t response_length) {
    uint8_t crc = calculate_crc(motor_id, data, length);
    uint8_t message[length + 1]; // +1 for CRC
    memcpy(message, data, length);
    message[length] = crc; // Append CRC

    CAN.sendMsgBuf(motor_id, 0, length + 1, message); // Send the message

    // Wait for response with timeout handling
    unsigned long startTime = millis();
    while (millis() - startTime < 200) { // timeout ms
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
            uint8_t len = 0;
            uint32_t rxId;
            CAN.readMsgBuf(&rxId, &len, response); // Read message
            if (rxId == motor_id && len == response_length) {
                return true; // Response received successfully
            }
        }
    }
    return false; // No response received
}

// Function to convert a byte array to a hex string with 0x prefix and uppercase
String bytesToHexStringWithPrefix(const uint8_t* bytes, size_t length) {
    String hexString = "";
    for (size_t i = 0; i < length; i++) {
        hexString += "0x"; // Add prefix
        if (bytes[i] < 0x10) {
            hexString += "0"; // Add leading zero for single digit hex
        }
        hexString += String(bytes[i], HEX); // Convert byte to hex
        if (i < length - 1) {
            hexString += " "; // Add space between bytes
        }
    }
    return hexString;
}

bool sendCanMessage(uint8_t motor_id, const uint8_t* data, size_t length, uint8_t* response, size_t* response_length) {
    uint8_t crc = calculate_crc(motor_id, data, length);
    uint8_t message[length + 1]; // +1 for CRC
    memcpy(message, data, length);
    message[length] = crc; // Append CRC

    // Print the message being sent in hex format
    // Print the motor_id in hex format before the message
    Serial.print("Sending CAN message: 0x");
    Serial.print(motor_id, HEX); // Print motor_id in hexadecimal format
    Serial.print(" "); // Add a space for readability
    String hexMessage = bytesToHexStringWithPrefix(message, length + 1);
    Serial.print("Sending CAN message: ");
    Serial.println(hexMessage);
    updateStatus("Sending CAN message: " + hexMessage); // Update HTML status

    CAN.sendMsgBuf(motor_id, 0, length + 1, message); // Send the message

    // Wait for response with timeout handling
    unsigned long startTime = millis();
    while (millis() - startTime < 2) { // timeout ms
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
            uint8_t len = 0;
            uint32_t rxId; // Variable to store the received message ID
            CAN.readMsgBuf(&rxId, &len, response); // Read message

            // Prepend rxId to the response
            // Assuming response has enough space, we can directly set the first byte
            memmove(response + 1, response, len); // Shift existing response data to the right
            response[0] = rxId; // Store rxId at the start of the response
            *response_length = len + 1; // Update the response length to include rxId


            // Print the received message in hex format
            String hexResponse = bytesToHexStringWithPrefix(response, *response_length);
            Serial.print("Received CAN response: ");
            Serial.println(hexResponse);
            updateStatus("Received CAN response: " + hexResponse); // Update HTML status

            return true; // Response received successfully
        }
    }
    return false; // No response received
}


void enableMotor(uint8_t motor_id, bool enable) {
    // Construct the command: [motor_id, command, enable_flag]
    uint8_t data[2] = {0xF3, enable ? 0x01 : 0x00}; // Command to enable/disable motor
    uint8_t response[10]; // Buffer for response
    size_t response_length; // Variable to hold the length of the response

    if (sendCanMessage(motor_id, data, sizeof(data), response, &response_length)) {
        // Check if the response matches the expected format
        if (response_length > 0 && response[0] == motor_id && response[1] == 0xF3) {
            updateStatus("Motor " + String(motor_id) + " enabled: " + String(enable));
        } else {
            updateStatus("Unexpected response for Motor " + String(motor_id));
        }
    } else {
        updateStatus("Failed to enable Motor " + String(motor_id));
    }
}


void scanCanDevices() {
    updateStatus("Scanning CAN bus...");
    Serial.println("Scanning CAN bus...");

    // Broadcast test message
    uint8_t testMessage[1] = {0x30}; // Example command
    CAN.sendMsgBuf(0x7DF, 0, 1, testMessage); // Send broadcast message

    Serial.println("Broadcast sent. Waiting for responses...");

    // Listen for responses for a limited time
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) { // 2-second timeout
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
            uint8_t len = 0;
            uint32_t rxId;
            uint8_t response[8];
            CAN.readMsgBuf(&rxId, &len, response); // Read message

            Serial.print("Device found at ID: ");
            Serial.println(rxId);
            updateStatus("Device found at ID: " + String(rxId));
        }
    }

    Serial.println("CAN scan complete.");
    updateStatus("CAN scan done.");
}

// Move Motor to Absolute Position
void moveMotorAbsolute(uint8_t motor_id, float angle) {
    int pulses = angleToPulses(motor_id, angle);
    uint8_t speed = 1000; // Set speed (RPM)
    uint8_t acceleration = 10; // Set acceleration

    // Prepare CAN data
    uint8_t speed_high = (speed >> 8) & 0xFF; // High byte of speed
    uint8_t speed_low = speed & 0xFF; // Low byte of speed

    // Convert pulses to 3 bytes
    uint8_t abs_axis_bytes[3];
    abs_axis_bytes[0] = (pulses >> 16) & 0xFF; // High byte
    abs_axis_bytes[1] = (pulses >> 8) & 0xFF;  // Middle byte
    abs_axis_bytes[2] = pulses & 0xFF;          // Low byte

    // Prepare the data array
    uint8_t data[7] = {0xF5, speed_high, speed_low, acceleration, abs_axis_bytes[0], abs_axis_bytes[1], abs_axis_bytes[2]};


    uint8_t response[10]; // Buffer for response
    size_t response_length; // Variable to hold the length of the response

    if (sendCanMessage(motor_id, data, sizeof(data), response, &response_length)) {
        // Check if the response matches the expected format
        if (response_length > 0 && response[0] == motor_id) {
            updateStatus("Motor " + String(motor_id) + " moved to " + String(angle) + "° (" + String(pulses) + " pulses)");
        } else {
            updateStatus("Unexpected response for Motor " + String(motor_id));
        }
    } else {
        updateStatus("Failed to move Motor " + String(motor_id));
    }
}


// Stop All Motors
void stopAllMotors() {
    for (int i = 0; i < 6; i++) {
        uint8_t data[1] = {0xF7}; // Command to stop the motor
        uint8_t response[10]; // Buffer for response
        size_t response_length; // Variable to hold the length of the response

        if (sendCanMessage(motor_ids[i], data, sizeof(data), response, &response_length)) {
            // Process the response if needed
            if (response_length > 0) {
                // Optionally log or handle the response
                String hexResponse = bytesToHexString(response, response_length);
                Serial.print("Response from Motor " + String(motor_ids[i]) + ": ");
                Serial.println(hexResponse);
            }
        } else {
            updateStatus("Failed to stop Motor " + String(motor_ids[i]));
        }
    }
    updateStatus("All Motors Stopped");
}

// Clear All Encoders
void clearAllEncoders() {
    for (int i = 0; i < 6; i++) {
        uint8_t data[1] = {0x92}; // Assuming 0x92 is the command to reset encoders
        uint8_t response[10]; // Buffer for response
        size_t response_length; // Variable to hold the length of the response

        if (sendCanMessage(motor_ids[i], data, sizeof(data), response, &response_length)) {
            // Process the response if needed
            if (response_length > 0) {
                // Optionally log or handle the response
                String hexResponse = bytesToHexString(response, response_length);
                Serial.print("Response from Motor " + String(motor_ids[i]) + ": ");
                Serial.println(hexResponse);
            }
        } else {
            updateStatus("Failed to clear encoder for Motor " + String(motor_ids[i]));
        }
    }
    updateStatus("Encoders Cleared");
}


// Function to read encoder position
void readEncoderPosition(uint8_t motor_id) {
    // Construct the command: [command]
    uint8_t command[1] = {0x30}; // Command to read encoder position
    uint8_t response[10]; // Buffer for response
    size_t response_length; // Variable to hold the length of the response

    if (sendCanMessage(motor_id, command, sizeof(command), response, &response_length)) {
        // Process the response
        if (response_length >= 8) { // Ensure we have enough data (1 byte for rxId + 7 bytes for data)
            // The first byte is the rxId, so we start extracting from the second byte
            int16_t carry = (response[2] | (response[3] << 8)); // Extract carry value
            int32_t value = (response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24)); // Extract encoder value
            updateStatus("M" + String(response[0]) + " Enc: " + String(value) + " Carry: " + String(carry));
        } else {
            updateStatus("Invalid response length from M" + String(motor_id));
        }
    } else {
        updateStatus("No response from M" + String(motor_id));
    }
}


void setWorkingCurrent(uint8_t motor_id, int current_ma) {
    if (current_ma > 5200) {
        updateStatus("Invalid Current Value for Motor " + String(motor_id));
        return;
    }
    // Construct the command: [command, current_high_byte, current_low_byte]
    uint8_t data[3] = { 0x83, (current_ma >> 8) & 0xFF, current_ma & 0xFF}; // Command to set current
    uint8_t response[8]; // Buffer for response
    size_t response_length; // Variable to hold the length of the response

    if (sendCanMessage(motor_id, data, sizeof(data), response, &response_length)) {
        // Check if the response matches the expected format
        if (response_length > 0 && response[0] == motor_id && response[1] == 0x83) {
            updateStatus("Motor " + String(motor_id) + " current set to " + String(current_ma) + "mA");
        } else {
            updateStatus("Unexpected response for Motor " + String(motor_id));
        }
    } else {
        updateStatus("Failed to set current for Motor " + String(motor_id));
    }
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
        } else if (msg == "SCAN_CAN_BUS") {
            scanCanDevices();
            webSocket.sendTXT(num, "Scanning CAN Bus...");
        } else if (msg == "ENABLE_MOTORS") {
            for (int i = 0; i < 6; i++) {
                enableMotor(motor_ids[i], true); // Enable each motor
                setWorkingCurrent(motor_ids[i], 3000); // Set working current to 3000mA
            }
            webSocket.sendTXT(num, "All motors enabled and working current set to 3000mA.");
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
