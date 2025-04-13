#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Adafruit_LSM6DSOX.h>
#include <WebSocketsClient.h>

Adafruit_LSM6DSOX lsm6ds;

const char* ssid = "BitHacks";
const char* password = "BitHacks2025!";

const char* laptopIP = "169.234.126.4";//"169.234.114.68";  // Replace with your laptop's IP
const uint16_t laptopPort = 1234;        // Port to send data to

WiFiUDP udp;
const char* websocket_server_host = "169.234.88.152";//"127.0.0.1";  // <-- Replace with your laptop's local IP
const uint16_t websocket_server_port = 8000;
const char* websocket_path = "/ws";

WebSocketsClient webSocket;

const int GPIO1 = 2; // GPIO pin for LED (originally GPIO2)
const int GPIO38 = 38; // GPIO pin for LED (originally GPIO38)

unsigned int count = 0;
const int VRxPin = 4;    // X-axis analog
const int VRyPin = 5;    // Y-axis analog

static unsigned long lastSend = 0;

// void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
//   switch(type) {
//     case WStype_DISCONNECTED:
//       Serial.println("WebSocket disconnected");
//       break;
//     case WStype_CONNECTED:
//       Serial.println("WebSocket connected");
//       webSocket.sendTXT("ESP32 joined!");
//       break;
//     case WStype_TEXT:
//       Serial.printf("Server says: %s\n", payload);
//       break;
//     default:
//       Serial.printf("Unknown event: %d\n", type);
//       break;
//   }
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // while (!Serial) delay(10);  // Wait until Serial is ready (works on some boards)
  delay(5000);
  Serial.println("Hello, world!");
  Serial.println("This is a test of the serial communication.");
  pinMode(GPIO1, OUTPUT);
  pinMode(GPIO38, OUTPUT);
  pinMode(VRxPin, INPUT);
  pinMode(VRyPin, INPUT);

  //------------- Connect to WiFi -------------//
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // webSocket.begin(websocket_server_host, websocket_server_port, websocket_path);
  // webSocket.onEvent(webSocketEvent);
  // webSocket.setReconnectInterval(5000); // auto-reconnect after 5s

  //------------- Accelerometer Setup -------------//
  // Custom I2C pins: SDA = GPIO 8, SCL = GPIO 9
  Wire.begin(8, 9);

  if (!lsm6ds.begin_I2C()) {
    Serial.println("❌ Failed to find LSM6DSOX. Check wiring.");
    while (1) delay(10);
  }

  Serial.println("✅ LSM6DSOX initialized.");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("Hello, world! " + String(count));
  // digitalWrite(GPIO1, HIGH);
  // digitalWrite(GPIO38, HIGH); // Turn on the LED
  // // delay(1000); // Wait for a second
  // digitalWrite(GPIO1, LOW);
  // digitalWrite(GPIO38, LOW); // Turn off the LED
  // // delay(1000); // Wait for a second
  // count++;

  int xValue = analogRead(VRxPin);     // 0 - 4095 (12-bit ADC)
  int yValue = analogRead(VRyPin);     // 0 - 4095
  Serial.print("X: ");
  Serial.print(xValue);
  Serial.print(" | Y: ");
  Serial.print(yValue);
  // delay(1000); // Delay for 100 milliseconds

  // -------------- Accelerometer Data -------------- //
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  // Print acceleration data
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  Serial.print("Accel [m/s^2] => X: ");
  Serial.print(ax, 2);
  Serial.print(" Y: ");
  Serial.print(ay, 2);
  Serial.print(" Z: ");
  Serial.println(az, 2);

  // Example: basic jump detection (spike in Z-axis)
  if (az < 5.0) {
    Serial.println("🕴 Jump detected!");
  }

  delay(200);
  // delay(1000); // Send ~10 packets/sec

  // Send data to laptop
  String message = String(xValue) + ";" + String(yValue) + ";" + String(az);  // Example motion data
  udp.beginPacket(laptopIP, laptopPort);
  udp.print(message);
  udp.endPacket();


  webSocket.loop(); // Maintain WebSocket connection
  

// if (millis() - lastSend > 1000) {
//     lastSend = millis();
//     String fakeJumpData = "xValue: " + String(xValue) + "; yValue: " + String(yValue) + "; az: " + String(az);
//     // Send fake jump data to WebSocket server
//     webSocket.sendTXT(fakeJumpData);
// }
}

