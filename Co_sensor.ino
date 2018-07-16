#include <Arduino.h>
#include <SPI.h>
//#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>      //Nucleo de la librería gráfica.
#include <Adafruit_SSD1306.h>  //Librería para pantallas OLED monocromas de 128x64 y 128x32
#include <WiFi.h>
#include <PubSubClient.h>
#include "MQ7.h"

#define OLED_RESET 0
#define OLED_PWR 13
#define DHT_PWR 2
#define DHT_DATA 15
#define DHTTYPE DHT11 
#define ANALOG_PIN_0 34
#define TOKEN_TB "Kwg5jqcXMfgEBzH6GaAh"
#define TEMP_TOPIC    "smarthome/room1/temp"
#define LED_TOPIC     "smarthome/room1/led" /* 1=on, 0=off */

Adafruit_SSD1306 display(OLED_RESET);
DHT dht(DHT_DATA, DHTTYPE);
MQ7 mq7(ANALOG_PIN_0,3.9);

int Temp=0;
int Nivel=1;
char bufferT[28]="";
int analog_value = 0;
const char* ssid     = "wifi";
const char* password = "secret1703secret1703";
int Wconectado=0;
const char* mqtt_server = "em.isef11.com.ar";
WiFiClient espClient;
PubSubClient client(espClient);

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  /* we got '1' -> on */
  //if ((char)payload[0] == '1') {
  //  digitalWrite(led, HIGH); 
  //} else {
    /* we got '0' -> on */
  //  digitalWrite(led, LOW);
 // }

}

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
  display.fillScreen(0);         //Limpiamos la pantalla
 
  display.setFont();             //Fuente por defecto -si no la hemos cambiado no es necesario seleccionarla
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
  
  display.display();             //Refrescamos la pantalla para visualizarlo
}

void Inicia_display(){
  display.fillScreen(0);         //Limpiamos la pantalla
 
  display.setFont();             //Fuente por defecto -si no la hemos cambiado no es necesario seleccionarla
  display.setTextSize(3);
  display.setTextColor(1,1);        
  display.setCursor(10,20);
  display.println("SENSOR");
  display.display();
}

void send_mqtt(float t, float h, float hic, float co){
  if (Wconectado ==1){
    if(!client.connected()) {
      Serial.print("Connecting to ThingsBoard node ...");
      if ( client.connect("Esp32COSensor", TOKEN_TB, NULL) ) {
        Serial.println( "[DONE]" );
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
      client.publish( "v1/devices/me/telemetry", attributes );
      //client.publish(TEMP_TOPIC, msg);
      Serial.println( attributes );
    }
  }
}

void setup() { 
  Serial.begin(115200);               
  pinMode(OLED_PWR, OUTPUT);
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
  
  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, 1883);
  /* this receivedCallback function will be invoked 
  when client received subscribed topic */
  client.setCallback(receivedCallback);
  Inicia_display();
}

void loop() {
 if (Wconectado == 0){
   Serial.println("Error No conectado wifi Wifi_init.");
   Wifi_init();
 }
  
  client.loop(); 
  
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    delay(600);
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);
  delay(60000);
  //analog_value = analogRead(ANALOG_PIN_0);
  analog_value = mq7.getPPM();
  Serial.print("Valor CO:");
  Serial.println(analog_value);
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);
  send_mqtt(t,h,hic,analog_value);
  Muestra_display((int)t,(int)h,(int)hic,(int)analog_value);
  
}
 

