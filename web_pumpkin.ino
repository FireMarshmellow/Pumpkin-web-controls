#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>
#include <HardwareSerial.h>

// ---------------------------
// WiFi Configuration
// ---------------------------
const char* ssid = "";
const char* password = "";


// ---------------------------
// Web Server Setup
// ---------------------------
WebServer server(80);

// ---------------------------
// LED Configuration
// ---------------------------
#define NUM_LEDS 20           // Number of LEDs in the strip
#define DATA_PIN 7            // GPIO 7 for LED strip data
CRGB leds[NUM_LEDS];

// ---------------------------
// Ultrasonic Sensor Configuration (Optional)
// ---------------------------
const int trigPin = 5;        // GPIO 5 for trig
const int echoPin = 4;        // GPIO 4 for echo

// ---------------------------
// GPIO Configuration
// ---------------------------
const int gpioPin = 6;        // GPIO 6 for control

// ---------------------------
// MP3 Module Configuration
// ---------------------------
#define RXD2 16               // GPIO 16 (RX) for Serial2
#define TXD2 17               // GPIO 17 (TX) for Serial2
HardwareSerial mySerial(1);  // Use UART1 (Serial2)

// ---------------------------
// MP3 Track Configuration
// ---------------------------
const int numSounds = 5;      // Number of MP3 tracks

// ---------------------------
// Function Prototypes
// ---------------------------
void handleRoot();
void handlePlay();
void handleGPIOOn();
void handleSetColor();

// MP3 Module Functions
void play_track(uint8_t track_number);
void send_command(uint8_t command[], size_t length);

// ---------------------------
// Setup Function
// ---------------------------
void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  //while (!Serial) { ; } // Wait for Serial to initialize

  // Initialize LED strip
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  // Initialize GPIO pin
  pinMode(gpioPin, OUTPUT);
  digitalWrite(gpioPin, LOW); // Start LOW

  // (Optional) Initialize Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize MP3 module communication
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2); // Adjust baud rate if necessary

  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Define web server routes
  server.on("/", handleRoot);
  server.on("/gpio/on", handleGPIOOn);
  server.on("/setColor", handleSetColor);

  // Use server.onNotFound to handle dynamic URIs like /play/1, /play/2, etc.
  server.onNotFound(handlePlay);

  // Start the web server
  server.begin();
  Serial.println("Web server started.");
}

// ---------------------------
// Loop Function
// ---------------------------
void loop() {
  server.handleClient();
}

// ---------------------------
// Web Server Handlers
// ---------------------------

// Root Handler - Serves the Control Web Page
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>ESP32 Controller</title></head><body>";
  html += "<h1>ESP32 MP3 & LED Controller</h1>";

  // MP3 Controls
  html += "<h2>Play MP3 Tracks</h2>";
  for (int i = 1; i <= numSounds; i++) {
    html += "<button onclick=\"playTrack(" + String(i) + ")\">Play Track " + String(i) + "</button><br><br>";
  }

  // GPIO Controls
  html += "<h2>GPIO Control</h2>";
  html += "<button onclick=\"toggleGPIO()\">Toggle GPIO " + String(gpioPin) + "</button><br><br>";

  // LED Color Control
  html += "<h2>LED Color Selector</h2>";
  html += "<input type='color' id='ledColor' name='ledColor' value='#FFFFFF'>";
  html += "<button onclick=\"setColor()\">Set Color</button><br><br>";

  // JavaScript for making asynchronous requests
  html += "<script>";
  
  // Play Track Function
  html += "function playTrack(track) {";
  html += "  fetch('/play/' + track)";
  html += "    .then(response => response.text())";
  html += "    .then(data => console.log(data));";
  html += "}";

  // Toggle GPIO Function
  html += "function toggleGPIO() {";
  html += "  fetch('/gpio/on')";
  html += "    .then(response => response.text())";
  html += "    .then(data => console.log(data));";
  html += "}";

  // Set LED Color Function
  html += "function setColor() { ";
  html += "  var color = document.getElementById('ledColor').value.substring(1);";
  html += "  fetch('/setColor?color=' + color)";
  html += "    .then(response => response.text())";
  html += "    .then(data => console.log(data));";
  html += "}";
  
  html += "</script>";
  
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Play Track Handler
void handlePlay() {
  String uri = server.uri(); // e.g., /play/1

  if (uri.startsWith("/play/")) {
    String trackStr = uri.substring(strlen("/play/")); // Extract the number after /play/
    int trackNumber = trackStr.toInt();

    if (trackNumber >= 1 && trackNumber <= numSounds) {
      play_track(trackNumber);
      String response = "Playing track " + String(trackNumber);
      Serial.println(response);
      server.send(200, "text/plain", response);
    } else {
      server.send(400, "text/plain", "Invalid track number");
    }
  } else {
    server.send(404, "text/plain", "Not found");
  }
}


// GPIO ON Handler
void handleGPIOOn() {
  digitalWrite(gpioPin, HIGH);
  delay(50);
  digitalWrite(gpioPin, LOW);
  String response = "GPIO " + String(gpioPin) + " toggle";
  Serial.println(response);
  server.send(200, "text/plain", response);
}

// Set LED Color Handler
void handleSetColor() {
  if (server.hasArg("color")) {
    String colorValue = server.arg("color"); // Hex color without '#'
    long intColor = strtol(colorValue.c_str(), NULL, 16);
    uint8_t red = (intColor >> 16) & 0xFF;
    uint8_t green = (intColor >> 8) & 0xFF;
    uint8_t blue = intColor & 0xFF;

    // Set the LED strip to the selected color
    fill_solid(leds, NUM_LEDS, CRGB(red, green, blue));
    FastLED.show();

    String response = "LED color set to #" + colorValue;
    Serial.println(response);
    server.send(200, "text/plain", response);
  } else {
    server.send(400, "text/plain", "No color specified");
  }
}

/*
// (Optional) Ultrasonic Sensor Distance Handler
void handleGetDistance() {
  long distance = getDistance();
  String response = "Distance: " + String(distance) + " cm";
  Serial.println(response);
  server.send(200, "text/plain", response);
}
*/

// ---------------------------
// MP3 Module Functions
// ---------------------------

// Function to play a specific track
void play_track(uint8_t track_number) {
  Serial.print("Playing track: ");
  Serial.println(track_number);
  uint8_t command[] = {0x7E, 0x03, 0x00, 0x02, 0x00, track_number, 0xEF};
  send_command(command, sizeof(command));
}



// Function to send a command to the MP3 module
void send_command(uint8_t command[], size_t length) {
  Serial.print("Sending command: ");
  for (size_t i = 0; i < length; i++) {
    Serial.print(command[i], HEX);
    Serial.print(" ");
    mySerial.write(command[i]);
  }
  Serial.println();
}