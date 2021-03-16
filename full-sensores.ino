#include <WiFi.h>           // WiFi control for ESP32
#include <ThingsBoard.h>    // ThingsBoard SDK
#include <ArduinoJson.h>
#include <PubSubClient.h>

//Datos de la red wifi a la que se conectara el ESP32
#define WIFI_AP_NAME        ""
#define WIFI_PASSWORD       ""

//datos del servidor de thingsboard y el token del sistema en thingsboard
#define TOKEN               ""
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

#define SERIAL_DEBUG_BAUD    115200

// Initialize ThingsBoard client
WiFiClient espClient;

PubSubClient client(espClient);

// Initialize ThingsBoard instance
ThingsBoard tb(espClient);

// the Wifi radio's status
int status = WL_IDLE_STATUS;


// ------------------------ sensor de humedad y temperatura
#include "DHT.h"

#define DHT_PIN 32     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHT_PIN, DHTTYPE);

#if defined(ARDUINO_ARCH_AVR)
#define SERIAL  Serial

#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
#define SERIAL  SerialUSB
#else
#define SERIAL  Serial
#endif


// ------------------------ sensor de luz
#define LIGHT_PIN 2
#include <math.h>
float Rsensor;                        //Resistance of sensor in K


// ------------------------ sensor de calidad del aire
#include "Air_Quality_Sensor.h"

#define AIR_PIN 33
AirQualitySensor sensor(AIR_PIN);  // pin del sensor de calidad del aire

#include <string>

float temperatura = 0;
float humedad = 0;
int luzV = 0;
String luzS = "";
int calidadV = 0;
String calidadS = "";

// ----------------------------- tira de leds
const int ledPin = 4;

// ----------------------------- OTA
#include <ArduinoOTA.h>


void setup() {
    SERIAL.begin(SERIAL_DEBUG_BAUD);

    pinMode(ledPin, OUTPUT);             
    pinMode(AIR_PIN, INPUT);             
    pinMode(LIGHT_PIN, INPUT);         
    pinMode(DHT_PIN, INPUT);            
    
    connectWiFi();
    client.setServer( THINGSBOARD_SERVER, 1883 );

    Serial.println("Waiting sensor to init...");
    delay(20000);

    if (sensor.init()) {
        Serial.println("Sensor ready.");
    } else {
        Serial.println("Sensor ERROR!");
    }

    Wire.begin();

    dht.begin();

    setupOTA();
}

void loop() {
    // Reconectar al wifi de ser necesario
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    ArduinoOTA.handle();
    actualizarDatos();
    enviarDatos();
    led();
    delay(2000);
}

void setupOTA () {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void enviarDatos()
{
   
    // Reconectar a Thingsboard de ser necesario
    if (!tb.connected()) {
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
        Serial.println("Fallo al conectar");
        return;
      }else{
        //Serial.println("Conexion realizada con exito);
      }
    }
   Serial.println("\nSending data...");
   tb.sendTelemetryFloat("Temperatura", temperatura);
   tb.sendTelemetryFloat("Humedad", humedad);
   tb.sendTelemetryInt("Luz", luzV);
   tb.sendTelemetryInt("Calidad aire", calidadV);
   tb.loop();
}

void led()
{
  if(!luzV) 
  {
    digitalWrite(ledPin, HIGH);
  }
  else 
  {
    digitalWrite(ledPin, LOW);
  }
}

void actualizarDatos() {
// ------------------------ sensor de humedad y temperatura
    float temp_hum_val[2] = {0};
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    
    
    if(!dht.readTempAndHumidity(temp_hum_val)){
        humedad = temp_hum_val[0];
        temperatura = temp_hum_val[1];
        SERIAL.print("\nhumedad: ");
        SERIAL.print(humedad);
        SERIAL.print("%, temperatura: ");
        SERIAL.print(temperatura);
        SERIAL.print("ºC");
    }
    else{
        SERIAL.print("\nFailed to get temprature and humidity value.");
    }
    
// ------------------------ sensor de luz
    luzV = digitalRead(LIGHT_PIN);
    
    // We'll have a few threshholds, qualitatively determined
    if (luzV == 0) {
        luzS = (" - Sin luz");
    } else {
        luzS = (" - Con luz");
    }
    SERIAL.print("\nSensor luminico: ");
    SERIAL.print(luzV);
    SERIAL.print(luzS);

// ------------------------ sensor de calidad del aire
    int quality = sensor.slope();

    calidadV = sensor.getValue();

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
        calidadS = " High pollution! Force signal active.";
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
        calidadS = " High pollution!";
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
        calidadS = " Low pollution!";
    } else if (quality == AirQualitySensor::FRESH_AIR) {
        calidadS = " Fresh air.";
    }
    SERIAL.print("\nSensor calidad del aire: ");
    SERIAL.print(calidadV);
    SERIAL.print(calidadS);

}

void connectWiFi()
{
  Serial.print("Connecting to AP ...");
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  delay(2000);
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    Serial.print(".");
    delay(2000);
  }
  Serial.println(" Connected to AP");
}
