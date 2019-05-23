#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#define DEBUG 0

// #define LOCAL true

/********************* PRESENCE STATES ********************/
#define ACTIVE_STATE 1
#define IDLE_STATE 0

/*
 *
 ** SET THE NUMBER OF PHONE SOAP STATIONS IN THE CLUSTER **
 *
 */
#define NUM_STATIONS 4

/********************* MQTT CONNECTION ********************/
#ifdef LOCAL
const char* mqttServer = "192.168.1.97";
#else
const char* mqttServer = "192.168.2.10";
#endif
const int mqttPort = 1883;


/*********************** MQTT TOPICS **********************/
#define T1_TOPIC "PS_T1"
#define T2_TOPIC "PS_T2"
#define T3_TOPIC "PS_T3"
#define T4_TOPIC "PS_T4"
#define PS_T_STATE_TOPIC "command/PS_T"

#define W1_TOPIC "PS_W1"
#define W2_TOPIC "PS_W2"
#define W3_TOPIC "PS_W3"
#define W4_TOPIC "PS_W4"
#define PS_W_STATE_TOPIC "command/PS_W"

#define O1_TOPIC "PS_O1"
#define O2_TOPIC "PS_O2"
#define O3_TOPIC "PS_O3"
#define O4_TOPIC "PS_O4"
#define PS_O_STATE_TOPIC "command/PS_O"


/*
 * MQTT PHONE SOAP STATIONS
 *
 * Set station below (REQUIRED)
 *
 */
#define STATION_T "PS_T"
// #define STATION_W "PS_W"
// #define STATION_O "PS_O"


/*
 *
 * Here, we set the configuration for the phone soap cluster.
 * This will choose the IP and MAC addresses as well as set 
 * the MQTT topics used for broadcasting states.
 *
 */
#if defined(STATION_T)
  #define STATION STATION_T
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEB };
  // IPAddress ip(192, 168, 1, 82);
  IPAddress ip(192, 168, 2, 102);
  #define PS1_TOPIC T1_TOPIC
  #define PS2_TOPIC T2_TOPIC
  #define PS3_TOPIC T3_TOPIC
  #define PS4_TOPIC T4_TOPIC
  #define PS_STATE_TOPIC PS_T_STATE_TOPIC
#elif defined(STATION_W)
  #define STATION STATION_W
  IPAddress ip(192, 168, 2, 103);
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC };
  #define PS1_TOPIC W1_TOPIC
  #define PS2_TOPIC W2_TOPIC
  #define PS3_TOPIC W3_TOPIC
  #define PS4_TOPIC W4_TOPIC
  #define PS_STATE_TOPIC PS_W_STATE_TOPIC
#elif defined(STATION_O)
  #define STATION STATION_O
  IPAddress ip(192, 168, 2, 104);
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF };
  #define PS1_TOPIC O1_TOPIC
  #define PS2_TOPIC O2_TOPIC
  #define PS3_TOPIC O3_TOPIC
  #define PS4_TOPIC O4_TOPIC
  #define PS_STATE_TOPIC PS_O_STATE_TOPIC
#endif


/********************** STATION PINS *********************/
const int PS1_sensorPin = 23; // The phone soap input pin for PS1 (INPUT)
const int PS1_activePin = 22; // The ACTIVE state pin to trigger the PS1 teensy (OUTPUT)
const int PS1_statePin = 40;  // The pin to trigger the PS1 teensy state (ON/OFF) (OUTPUT)

const int PS2_sensorPin = 25; // The phone soap input pin for PS2 (INPUT)
const int PS2_activePin = 24; // The ACTIVE state pin to trigger the PS2 teensy (OUTPUT)
const int PS2_statePin = 41;  // The pin to trigger the PS2 teensy state (ON/OFF) (OUTPUT)

const int PS3_sensorPin = 27; // The phone soap input pin for PS3 (INPUT)
const int PS3_activePin = 26; // The ACTIVE state pin to trigger the PS3 teensy (OUTPUT)
const int PS3_statePin = 42;  // The pin to trigger the PS3 teensy state (ON/OFF) (OUTPUT)

const int PS4_sensorPin = 29; // The phone soap input pin for PS4 (INPUT)
const int PS4_activePin = 28; // The ACTIVE state pin to trigger the PS4 teensy (OUTPUT)
const int PS4_statePin = 43;  // The pin to trigger the PS4 teensy state (ON/OFF) (OUTPUT)


/**************** INITIALIZE CURRENT STATE ***************/
int PS1_currentState;
int PS2_currentState;
int PS3_currentState;
int PS4_currentState;
int PS_state;
int tempState[NUM_STATIONS];


/************** INITIALIZE ETHERNET LIBRARY *************/
EthernetClient net;

/**************** INITIALIZE MQTT LIBRARY ***************/
PubSubClient mqttClient(net);



/********* STATION NAMES (used as MQTT Topics) **********/
const char stations[NUM_STATIONS][20] = {PS1_TOPIC, PS2_TOPIC, PS3_TOPIC, PS4_TOPIC};

/******* STATION STATES (used as MQTT Messages) *********/
const char states[2][10] = {"IDLE", "ACTIVE"};

const char toggleTopics[1][20] = {PS_STATE_TOPIC};

/****** Put the states into an array for indexing *******/
int currentStates[NUM_STATIONS] = {PS1_currentState, PS2_currentState, PS3_currentState, PS4_currentState};

/******* Put the pins into an array for indexing ********/
const int sensorPins[NUM_STATIONS] = {PS1_sensorPin, PS2_sensorPin, PS3_sensorPin, PS4_sensorPin};
const int activePins[NUM_STATIONS] = {PS1_activePin, PS2_activePin, PS3_activePin, PS4_activePin};
const int statePins[NUM_STATIONS] = {PS1_statePin, PS2_statePin, PS3_statePin, PS4_statePin};



/*************** RECONNECT TO MQTT BROKER ***************/
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect with the client ID
    if (mqttClient.connect(STATION)) {
        Serial.println("Connected!");
        // Once connected, publish an announcement...
        mqttClient.publish(STATION, "CONNECTED", true);

        // Subscribe to each station topic
        for (int i = 0; i < NUM_STATIONS; i++) {
          mqttClient.subscribe(stations[i]);
        }
    } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
    }
  }
}


/*********** RECONNECT WITHOUT BLOCKING LOOP ************/
long lastReconnectAttempt = 0;
boolean reconnect_non_blocking() {
  if (mqttClient.connect(STATION)) {
    Serial.println("CONNECTED");
    // Once connected, publish an announcement...
    mqttClient.publish(STATION, "CONNECTED", true);

    mqttClient.subscribe(toggleTopics[0]);
    // Subscribe to each station topic
    for (int i = 0; i < NUM_STATIONS; i++) {
      mqttClient.subscribe(stations[i]);
    }
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }
  return mqttClient.connected();
}

/**** CALLBACK RUN WHEN AN MQTT MESSAGE IS RECEIVED *****/
void messageReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);  
  Serial.println("] ");
  char payloadArr[length+1];
  
  for (unsigned int i=0;i<length;i++)
  {
    payloadArr[i] = (char)payload[i];
  }

  // Terminate array with null value
  payloadArr[length] = 0;
  Serial.println(payloadArr);
  
  if (strcmp(topic, toggleTopics[0]) == 0) {
    if (strcmp(payloadArr, "stop") == 0) PS_state = 0;
    if (strcmp(payloadArr, "start") == 0) PS_state = 1;
  }
  Serial.print("PS_state: ");
  Serial.println(PS_state);
}

void setup() {
  // Initialize serial communication:
  Serial.begin(9600);

  // Initialize the ethernet connection
  Ethernet.begin(mac, ip);
  Serial.print("Ethernet connection initialized with IP Address: ");
  Serial.println(ip);

  // Initialize the MQTT connection
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(messageReceived);
  Serial.print("MQTT connection initialized on MQTT Server: ");
  Serial.print(mqttServer);
  Serial.print(", on port: ");
  Serial.println(mqttPort);


  // Initialize all pins and set currentStates to IDLE
  for (int i = 0; i < NUM_STATIONS; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
    pinMode(activePins[i], OUTPUT);
    pinMode(statePins[i], OUTPUT);
    currentStates[i] = IDLE_STATE;
  }
  lastReconnectAttempt = 0;
  PS_state = 0;
}

void loop() {
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 1000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect_non_blocking()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    mqttClient.loop();
  }
  
  // Turns all phone soap LEDs ON/OFF
  stateToggle(PS_state);

  // Run each statin through the state machine
  for (int i = 0; i < NUM_STATIONS; i++) {
    stateMachine(i);
  }

  // Small delay to keep things stable
  delay(100);
}

void stateToggle (int toggle) {
  Serial.print("TOGGLE: ");
  Serial.println(toggle);
  for (int i = 0; i < NUM_STATIONS; i++) {
    digitalWrite(statePins[i], toggle);
  }
}

/********* STATE DECISION MAKER *********/
void stateMachine (int pos) {
  // Get the current sensor state
  tempState[pos] = getState(pos);
  int currentState = currentStates[pos]; 
#if DEBUG == 1
  if (pos == 0) {
    Serial.print("PS1 state: ");
    Serial.println(tempState[pos]);
    Serial.println(states[tempState[pos]]);
    Serial.println("");
  }
#endif
  /*
   * Only send the state update on the first loop.
   *
   * IF the current (temporary) PS sensor state is not equal to the actual current broadcasted state,
   * THEN we can safely change the actual state and broadcast it.
   *
   */
  if (tempState[pos] != currentState) {
#if DEBUG == 1
    if (pos == 0) {
      Serial.print("Sensor State Changed: ");
      Serial.println(stations[pos]);
      Serial.print("Last State: ");
      Serial.println(currentState);
      Serial.print("New State: ");
      Serial.println(tempState[pos]);
    }
#endif
    currentStates[pos] = tempState[pos];
    // Publish the message for this station. i.e. client.publish("PS1", "ACTIVE", true);
    if (mqttClient.connected()) mqttClient.publish(stations[pos], states[currentStates[pos]], true);
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

/********* SET GIVEN STATION PIN TO ACTIVE *********/
void setActive (int pos) {
#if DEBUG == 1
  if (pos == 0) {
    Serial.println("Set ACTIVE: PS1");
  }
#endif
  digitalWrite(activePins[pos], HIGH);
}

/********** SET GIVEN STATION PIN TO IDLE **********/
void setIdle (int pos) {
#if DEBUG == 1
  if (pos == 0) {
    Serial.println("Set IDLE: PS1");
  }
#endif
  digitalWrite(activePins[pos], LOW);
}

/****** GET STATE OF STATIONS PHONE SOAP PIN *******/
int getState(int pos) {
#if DEBUG == 1
  // if (pos == 0) {
    Serial.print("Input: ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.println(digitalRead(sensorPins[pos]));
  // }
#endif
  return digitalRead(sensorPins[pos]) == LOW;
}