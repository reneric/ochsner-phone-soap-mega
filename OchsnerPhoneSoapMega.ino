#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

/************* PRESENCE STATES *************/
#define ACTIVE_STATE 0
#define IDLE_STATE 1


#define NUM_STATIONS 4

/************* MQTT CONNECTION *************/
const int mqttPort = 1883;
const char* mqttServer = "192.168.2.10";

/************ NETWORK CONNECTION ***********/
IPAddress PS_T_IP(192, 168, 1, 97);
byte PS_T_MAC[] = { 0x75, 0xF0, 0x62, 0xC2, 0xAD, 0x09 };
IPAddress PS_W_IP(192, 168, 1, 98);
byte PS_W_MAC[] = { 0xAB, 0x93, 0x0A, 0xDF, 0x7B, 0x81 };
IPAddress PS_O_IP(192, 168, 1, 99);
byte PS_O_MAC[] = { 0x44, 0xA0, 0x99, 0x11, 0xFA, 0x93 };

IPAddress ip;
byte mac[sizeof(PS_T_MAC)];

/*************** MQTT TOPICS ***************/
const char* T1_TOPIC = "phoneSoap/PS_T1";
const char* T2_TOPIC = "phoneSoap/PS_T2";
const char* T3_TOPIC = "phoneSoap/PS_T3";
const char* T4_TOPIC = "phoneSoap/PS_T4";

const char* W1_TOPIC = "phoneSoap/PS_W1";
const char* W2_TOPIC = "phoneSoap/PS_W2";
const char* W3_TOPIC = "phoneSoap/PS_W3";
const char* W4_TOPIC = "phoneSoap/PS_W4";

const char* O1_TOPIC = "phoneSoap/PS_O1";
const char* O2_TOPIC = "phoneSoap/PS_O2";
const char* O3_TOPIC = "phoneSoap/PS_O3";
const char* O4_TOPIC = "phoneSoap/PS_O4";

char* STATION;
char PS1_TOPIC;
char PS2_TOPIC;
char PS3_TOPIC;
char PS4_TOPIC;

/******** MQTT PHONE SOAP STATIONS *********/
/*
 * 
 * *** SET STATION BELOW ***
 * ****** THIS IS REQUIRED ******
 *
 */
const char* STATION_T = "PS_T";
// const char* STATION_W = "PS_W";
// const char* STATION_O = "PS_O";

#if defined(STATION_T)
  STATION = STATION_T;
  ip = PS_T_IP;
  mac[] = PS_T_MAC;
  PS1_TOPIC = T1_TOPIC;
  PS2_TOPIC = T2_TOPIC;
  PS3_TOPIC = T3_TOPIC;
  PS4_TOPIC = T4_TOPIC;
#elif defined(STATION_W)
  STATION = STATION_W;
  ip = PS_W_IP;
  mac[] = PS_W_MAC;
  PS1_TOPIC = W1_TOPIC;
  PS2_TOPIC = W2_TOPIC;
  PS3_TOPIC = W3_TOPIC;
  PS4_TOPIC = W4_TOPIC;
#elif defined(STATION_O)
  STATION = STATION_O;
  ip = PS_O_IP;
  mac[] = PS_O_MAC;
  PS1_TOPIC = O1_TOPIC;
  PS2_TOPIC = O2_TOPIC;
  PS3_TOPIC = O3_TOPIC;
  PS4_TOPIC = O4_TOPIC;
#endif



/************** STATION PINS **************/
const int PS1_sensorPin = 23;          // The phone soap input pin for PS1 (INPUT)
const int PS1_activePin = 22;          // The ACTIVE state pin to trigger the PS1 matrix/teensy (OUTPUT)

const int PS2_sensorPin = 25;          // The phone soap input pin for PS2 (INPUT)
const int PS2_activePin = 24;          // The ACTIVE state pin to trigger the PS2 matrix/teensy (OUTPUT)

const int PS3_sensorPin = 27;          // The phone soap input pin for PS3 (INPUT)
const int PS3_activePin = 26;          // The ACTIVE state pin to trigger the PS3 matrix/teensy (OUTPUT)

const int PS4_sensorPin = 29;          // The phone soap input pin for PS4 (INPUT)
const int PS4_activePin = 28;          // The ACTIVE state pin to trigger the PS4 matrix/teensy (OUTPUT)


/******** INITIALIZE CURRENT STATE ********/
int PS1_currentState;
int PS2_currentState;
int PS3_currentState;
int PS4_currentState;


/******* INITIALIZE ETHERNET LIBRARY *******/
EthernetClient net;
/********* INITIALIZE MQTT LIBRARY *********/
PubSubClient mqttClient(net);

// Station names, used as MQTT Topics
const char stations[NUM_STATIONS][10] = {PS1_TOPIC, PS2_TOPIC, PS3_TOPIC, PS4_TOPIC};

// Station states, used as MQTT Messages
const char states[2][10] = {"ACTIVE", "IDLE"};

// Put the current states into an array for indexing
int currentStates[NUM_STATIONS] = {PS1_currentState, PS2_currentState, PS3_currentState, PS4_currentState};

// Put the pins into arrays for indexing
const int sensorPins[NUM_STATIONS] = {PS1_sensorPin, PS2_sensorPin, PS3_sensorPin, PS4_sensorPin};
const int activePins[NUM_STATIONS] = {PS1_activePin, PS2_activePin, PS3_activePin, PS4_activePin};

// Reconnect to the MQTT broker when the connection is lost
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect with the client ID
    if (mqttClient.connect(STATION)) {
        Serial.println("Connected!");
        // Once connected, publish an announcement...
        mqttClient.publish(STATION, "CONNECTED");

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

boolean reconnect_non_blocking() {
  if (mqttClient.connect(STATION)) {
    Serial.println("Connected!");
    // Once connected, publish an announcement...
    mqttClient.publish(STATION, "CONNECTED");

    // Subscribe to each station topic
    for (int i = 0; i < NUM_STATIONS; i++) {
      mqttClient.subscribe(stations[i]);
    }
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
  }
  return mqttClient.connected();
}

void messageReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);  
  Serial.println("] ");
  char payloadArr[length+1];
  
  for (unsigned int i=0;i<length;i++)
  {
    payloadArr[i] = (char)payload[i];
  }
  payloadArr[length] = 0;

  Serial.println(payloadArr);  // null terminated array
}

long lastReconnectAttempt = 0;
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
  lastReconnectAttempt = 0;
}

void loop() {
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect_non_blocking()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    mqttClient.loop();
  }

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
    mqttClient.publish(stations[pos], states[currentStates[pos]]);
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