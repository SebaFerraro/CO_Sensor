#include <SPI.h>
//#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>      //Nucleo de la librería gráfica.
#include <Adafruit_SSD1306.h>  //Librería para pantallas OLED monocromas de 128x64 y 128x32
#define OLED_RESET 0
#define OLED_PWR 13
#define DHT_PWR 2
#define DHT_DATA 15
#define DHTTYPE DHT11 
#define ANALOG_PIN_0 34

Adafruit_SSD1306 display(OLED_RESET);
DHT dht(DHT_DATA, DHTTYPE);
int Temp=0;
int Nivel=1;
char bufferT[28]="";
int analog_value = 0;

void setup() { 
  Serial.begin(115200);               
  pinMode(OLED_PWR, OUTPUT);
  digitalWrite(OLED_PWR, HIGH);
  pinMode(DHT_PWR, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  digitalWrite(DHT_PWR, HIGH);
  delay(400);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // //Inicializa pantalla con en la dirección 0x3D para la conexión I2C.
  dht.begin();
}

void loop() {
 
//PUNTOS EN LAS CUATRO ESQUINAS DE LA PANTALLA
  analog_value = analogRead(ANALOG_PIN_0);
  Serial.print("Valor CO:");
  Serial.println(analog_value);
  
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);
  
  display.fillScreen(0);         //Limpiamos la pantalla
 
  display.setFont();             //Fuente por defecto -si no la hemos cambiado no es necesario seleccionarla
  display.setTextSize(2);
  display.setTextColor(1,0);        
  display.setCursor(0,0);
  sprintf(bufferT,"T:%dc H:%d",(int)t,(int)h); 
  display.println(bufferT);
  display.setCursor(0,24);
  sprintf(bufferT,"ST:%dc",(int)hic); 
  display.println(bufferT);

  display.setCursor(0,48);
  sprintf(bufferT,"CO:%d",(int)analog_value); 
  display.println(bufferT);
  
  display.display();             //Refrescamos la pantalla para visualizarlo
  delay(60000);
  
 }
