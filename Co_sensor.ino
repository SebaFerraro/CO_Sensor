#include <Arduino.h>
#include <SPI.h>
//#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>      //Nucleo de la librería gráfica.
#include <Adafruit_SSD1306.h>  //Librería para pantallas OLED monocromas de 128x64 y 128x32
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "MQ7.h"

#define OLED_RESET 0
#define OLED_PWR 13
#define DHT_PWR 4
#define DHT_DATA 15
#define DHTTYPE DHT11 
#define ANALOG_PIN_0 34
#define TOKEN_TB "Kwg5jqcXMfgEBzH6GaAh"
#define TEMP_TOPIC    "smarthome/room1/temp"
#define LED_TOPIC     "smarthome/room1/led" /* 1=on, 0=off */

Adafruit_SSD1306 display(OLED_RESET);
DHT dht(DHT_DATA, DHTTYPE);
MQ7 mq7(ANALOG_PIN_0,3.9);
static int taskCore = 0;
int Temp=0;
int Nivel=1;
char bufferT[28]="";
int analog_value = 0;
const char* ssid     = "wifi";
const char* password = "secret1703secret1703";
static int Wconectado = 0;
boolean gpioState[36] = {false};
const char* mqtt_server = "em.isef11.com.ar";
WiFiClient espClient;
PubSubClient client(espClient);

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.  Attempting to reconnect...");
      //WiFi.reconnect();
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 station start");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 station connected to AP");
      break;
    default:      
      Serial.println("Unhandled WiFi Event raised.");
      break;
    }
}

void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
}

void Muestra_display(int t,int h, int hic, int co){
  display.fillScreen(0);        
 
  display.setFont();             
  display.setTextSize(2);
  display.setTextColor(1,0);        
  display.setCursor(0,0);
  sprintf(bufferT,"T:%dc H:%d",t,h); 
  display.println(bufferT);
  display.setCursor(0,24);
  sprintf(bufferT,"ST:%dc",hic); 
  display.println(bufferT);

  display.setCursor(0,48);
  sprintf(bufferT,"CO:%d ppm",co); 
  display.println(bufferT);
  
  display.display();            
}

void Inicia_display(){
  display.fillScreen(0);         
 
  display.setFont();             
  display.setTextSize(3);
  display.setTextColor(1,1);        
  display.setCursor(10,20);
  display.println("SENSOR");
  display.display();
}

String get_gpio_status() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();
  data[String(2)] = gpioState[2] ? true : false;
  char payload[256];
  data.printTo(payload, sizeof(payload));
  String strPayload = String(payload);
  Serial.print("Gpio Estado: ");
  Serial.println(strPayload);
  return strPayload;
}

void set_gpio_status(int pin, boolean enabled) {
   digitalWrite(pin, enabled ? HIGH : LOW);
   // Update GPIOs state
   gpioState[pin] = enabled;
}

void on_message(char* topic, byte* payload, unsigned int length) {

  Serial.println("Mensaje Recibido");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Mensaje: ");
  Serial.println(json);

  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject((char*)json);

  if (!data.success())
  {
    Serial.println("parseObject() Error");
    return;
  }

  String methodName = String((const char*)data["method"]);

  if (methodName.equals("getGpioStatus") or methodName.equals("getValue")) {
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  } else if (methodName.equals("setGpioStatus")) {
    // Update GPIO status and reply
    set_gpio_status(data["params"]["pin"], data["params"]["enabled"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  } else if (methodName.equals("setValue")) {
    set_gpio_status(2, data["params"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), data["params"]);
    client.publish("v1/devices/me/attributes",data["params"]);
  }
}

void send_mqtt(float t, float h, float hic, float co){
  if (Wconectado == 1){
    if(!client.connected()) {
      Serial.print("Conectando ThingsBoard node ...");
      if ( client.connect("Esp32COSensor", TOKEN_TB, NULL) ) {
        Serial.println( "[DONE]" );
        int rsus=client.subscribe("v1/devices/me/rpc/request/+");
        Serial.print( "[SUBSCRIBE]" );
        Serial.println(rsus);
        client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
        Serial.print("Enviando GPIO status ...");
        Serial.println(rsus);
      } else {
        Serial.print( "[FAILED] [ rc = " );
        Serial.print( client.state() );
      }
    }  
    if (client.connected()) {
        
      String temperatura = String(t);
      String humedad = String(h);
      String stermica = String(hic);
      String sco = String(co);
  
      String payload = "{";
      payload += "\"temperature\":";
      payload += temperatura;
      payload += ",";
      payload += "\"humedad\":";
      payload += humedad;
      payload += ",";
      payload += "\"stermica\":";
      payload += stermica;
      payload += ",";
      payload += "\"co\":";
      payload += sco;
      payload += "}";

      // Send payload
      char attributes[100];
      payload.toCharArray( attributes, 100 );
      int rsus=client.publish( "v1/devices/me/telemetry", attributes );
      Serial.print( "Publish : ");
      Serial.println(rsus);
      //client.publish(TEMP_TOPIC, msg);
      Serial.println( attributes );
    }
  }
}


void coreTask( void * pvParameters ){
 
    String taskMessage = "Corriendo en core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
    while(true){
      if(Wconectado == 1){
          if(!client.connected()) {
              Serial.print("Conectando a ThingsBoard node ...");
              if ( client.connect("Esp32COSensor", TOKEN_TB, NULL) ) {
                 Serial.println( "[DONE]" );
                 int rsus=client.subscribe("v1/devices/me/rpc/request/+");
                 Serial.print( "[SUBSCRIBE]" );
                 Serial.println(rsus);
                 rsus=client.publish("v1/devices/me/attributes", get_gpio_status().c_str()); 
                 Serial.print("Enviando GPIO status ...");
                 Serial.println(rsus);
              } else {
                 Serial.print( "[FAILED] [ rc = " );
                 Serial.print( client.state() );
              }
          }
          //Serial.println("Client LOOP");
          //Serial.print("LOOP ");
          if(! client.loop()){
            Serial.println("Error en Loop.");  
          }
          //Serial.println(taskMessage);
      }else{
      Serial.print("No conectado wifi:");
      Serial.println(Wconectado);
    }
   delay(1000);
   }
}

void setup() { 
  Serial.begin(115200);               
  pinMode(OLED_PWR, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(OLED_PWR, HIGH);
  pinMode(DHT_PWR, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  digitalWrite(DHT_PWR, HIGH);
  analogReadResolution(12); //12 bits
  analogSetAttenuation(ADC_11db);  //For all pins
  analogSetPinAttenuation(ANALOG_PIN_0, ADC_11db); //0db attenu
  delay(400);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // //Inicializa pantalla con en la dirección 0x3D para la conexión I2C.
  dht.begin();
    
  client.setServer(mqtt_server, 1883);
  client.setCallback(on_message);
  Inicia_display();
  xTaskCreatePinnedToCore(coreTask, "coreTask", 10000, NULL, 0, NULL, taskCore);
  Serial.println("Hilo Creado...");  
}


void loop() {
 if (Wconectado == 0){
   Serial.println("Error No conectado wifi Wifi_init.");
   Wifi_init();
 }
    
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    delay(600);
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);
  delay(60000);
  analog_value = mq7.getPPM();
  Serial.print("Valor CO:");
  Serial.println(analog_value);
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);
  String taskMessage = "Corriendo en core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  send_mqtt(t,h,hic,analog_value);
  Muestra_display((int)t,(int)h,(int)hic,(int)analog_value);
 }
 

