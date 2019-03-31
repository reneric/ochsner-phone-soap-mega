#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// Update these with values suitable for the hardware/network.
byte mac[] = { 0x30, 0x8E, 0xB9, 0xC4, 0x47, 0x16 };
IPAddress ip(192, 168, 1, 97);

// Presence States
#define ACTIVE_STATE 0
#define IDLE_STATE 1

#define NUM_STATIONS 4                // The number of stations

// The Client ID for connecting to the MQTT Broker
const char CLIENT_ID = 'phoneSoap1';

// Station Pins
const int PS1_sensorPin = 23;          // The phone soap input pin for PS1 (INPUT)
const int PS1_activePin = 22;          // The ACTIVE state pin to trigger the PS1 matrix/teensy (OUTPUT)

const int PS2_sensorPin = 25;          // The phone soap input pin for PS2 (INPUT)
const int PS2_activePin = 24;          // The ACTIVE state pin to trigger the PS2 matrix/teensy (OUTPUT)

const int PS3_sensorPin = 27;          // The phone soap input pin for PS3 (INPUT)
const int PS3_activePin = 26;          // The ACTIVE state pin to trigger the PS3 matrix/teensy (OUTPUT)

const int PS4_sensorPin = 29;          // The phone soap input pin for PS4 (INPUT)
const int PS4_activePin = 28;          // The ACTIVE state pin to trigger the PS4 matrix/teensy (OUTPUT)

// Initialize the current state for each station
int PS1_currentState;
int PS2_currentState;
int PS3_currentState;
int PS4_currentState;

// Initialize the ethernet library
EthernetClient net;
// Initialize the MQTT library
PubSubClient mqttClient(net);

const char* mqttServer = "192.168.1.69";

// Station names, used as MQTT Topics
const char stations[NUM_STATIONS][10] = {"PS1", "PS2", "PS3", "PS4"};

// Station states, used as MQTT Messages
const char states[2][10] = {"ACTIVE", "IDLE"};

// Put the current states into an array for indexing
int currentStates[NUM_STATIONS] = {PS1_currentState, PS2_currentState, PS3_currentState, PS4_currentState, R2_currentState, R3_currentState};

// Put the pins into arrays for indexing
const int sensorPins[NUM_STATIONS] = {PS1_sensorPin, PS2_sensorPin, PS3_sensorPin, PS4_sensorPin};
const int activePins[NUM_STATIONS] = {PS1_activePin, PS2_activePin, PS3_activePin, PS4_activePin};

// Reconnect to the MQTT broker when the connection is lost
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect with the client ID
    if (mqttClient.connect(CLIENT_ID)) {
        Serial.println("Connected!");
        // Once connected, publish an announcement...
        mqttClient.publish(CLIENT_ID, "CONNECTED");

        // Subscribe to each station topic
        for (int i = 0; i < NUM_STATIONS; i++) {
          mqttClient.subscribe(stations[i]);
        }
    } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
    }
  }
}

void messageReceived(char* topic, byte* payload, unsigned int length) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void setup() {
  // Initialize serial communication:
  Serial.begin(9600);

  // Initialize the ethernet connection
  Ethernet.begin(mac, ip);
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(messageReceived);
  
  // Initialize all pins and set currentStates to IDLE
  for (int i = 0; i < NUM_STATIONS; i++) {
    pinMode(sensorPins[i], INPUT);
    pinMode(activePins[i], OUTPUT);
    currentStates[i] = IDLE_STATE;
  }
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Run each statin through the state machine
  for (int i = 0; i < NUM_STATIONS; i++) {
    stateMachine(i);
  }
  // Small delay to keep things stable
  delay(100);
}

int lastTempState[NUM_STATIONS];
void stateMachine (int pos) {
  // Get the current sensor state
  lastTempState[pos] = getState(pos);
  
  /*
   * Only send the state update on the first loop.
   *
   * IF the current (temporary) PS sensor state is not equal to the actual current broadcasted state,
   * THEN we can safely change the actual state and broadcast it.
   *
   */
  if (lastTempState[pos] != currentStates[pos]) {
#if DEBUG == 1    
    Serial.print("Sensor State Changed: ");
    Serial.println(stations[pos]);
    Serial.print("Last State: ");
    Serial.println(currentStates[pos]);
    Serial.print("New State: ");
    Serial.println(lastTempState[pos]);
#endif
    currentStates[pos] = lastTempState[pos];
    // Publish the message for this station. i.e. client.publish("PS1", "ACTIVE")
    client.publish(stations[pos], states[currentStates[pos]]);
  }
  switch (currentStates[pos]) {
    case ACTIVE_STATE:
      setActive(pos);
      break;
    default:
      setIdle(pos);
      break;
  }
}

void setActive (int pos) {
  digitalWrite(activePins[pos], HIGH);
}

void setIdle (int pos) {
  digitalWrite(activePins[pos], LOW);
}

int getState(int pos) {
  return digitalRead(sensorPins[pos]) == HIGH ? ACTIVE_STATE : IDLE_STATE;
}
