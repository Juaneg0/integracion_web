#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "DHT20.h"
#include "UbidotsEsp32Mqtt.h"

#include "data.h"
#include "Settings.h"

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s

/*** DEFINICIONES DE LAS FUNCIONES *******************************************************/
//*** VARIABLES DE LEDS Y COMUNICACION ***/
int tam = 0;
char pos = '0';
char val = '0';
int indx = 0;
char LED[2] = {'0', '0'};

const uint8_t LED1 = 27; // Pin used to write data based on 1's and 0's coming from Ubidots
const uint8_t LED2 = 26; // Pin used to write data based on 1's and 0's coming from Ubidots

void TurnLEDS(char *topic, byte *payload)
{
  tam = (strlen(topic)) - 4;
  pos = (char)(topic[tam]);
  indx = (int)(pos - '0') - 1;
  val = (char)payload[0];

  LED[indx] = val;

  if (LED[0] == '1')
  {
    digitalWrite(LED1, HIGH);
  }
  else
  {
    digitalWrite(LED1, LOW);
  }

  if (LED[1] == '1')
  {
    digitalWrite(LED2, HIGH);
  }
  else
  {
    digitalWrite(LED2, LOW);
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  TurnLEDS(topic, payload);
}
/*****************************************************************************************/

/*** DEFINICIÓN DE VARIABLES *************************************************************/
DHT20 DHT(&Wire);

const char *UBIDOTS_TOKEN = "BBFF-s28LKWunSHYGoqw0NYf1JaxfZRe0vq"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "Sensor";                               // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL1 = "SW1";                               // Replace with your variable label to subscribe to
const char *VARIABLE_LABEL2 = "SW2";                               // Replace with your variable label to subscribe to

Ubidots ubidots(UBIDOTS_TOKEN);

TFT_eSPI tft = TFT_eSPI();
float Humedad = 0;
float Temperatura = 0;
char WifiCon = 0;

//*** VARIABLES DE TIEMPO ***/
unsigned long TS_ant, TS_act;  // Tiempos para el muestreo del sensor
const long TS_fin = 1000 / 40; // Tiempo en milisegundos para un tiempo de muestreo de 40 Hz
unsigned long TP_ant, TP_act;  // Tiempos para la muestra del dato en el OLED
const long TP_fin = 500;       // Tiempo en milisegundos para una actualizacion de 2 Hz
unsigned long Tx_ant, Tx_act;  // Tiempos para la muestra del dato en el OLED
const long Tx_fin = 5000;      // Tiempo en milisegundos para una actualizacion de 0.2 Hz
unsigned long Rx_ant, Rx_act;  // Tiempos para la muestra del dato en el OLED
const long Rx_fin = 100;       // Tiempo en milisegundos para una actualizacion de 40 Hz

/*****************************************************************************************/

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("TTGO", "12345678");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 20) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
    DHT.begin(); //  ESP32 default pins 21 22
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Realizado por:", 10, 5, 2);
    tft.drawString("Juan E. Gomez", 10, 23, 4);
    tft.drawString("Humedad:", 10, 70, 2);
    tft.drawString("Temperatura", 140, 70, 2);
    tft.drawFastHLine(10, 50, 170, TFT_GREEN);
    tft.fillRect(110, 65, 3, 80, TFT_GREEN);
    Serial.println("--------------Pantalla Encendida-------");

    ubidots.setCallback(callback);
    ubidots.setup();
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL1); // Insert the device and variable's Labels, respectively
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL2); // Insert the device and variable's Labels, respectively

    delay(1000);

    Serial.println("Humidity, Temperature");

    TS_ant = millis();
    TP_ant = millis();
    Tx_ant = millis();
    Rx_ant = millis();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    TS_act = millis();
    TP_act = millis();
    Tx_act = millis();
    Rx_act = millis();

    // Lectura del sensor
    if (TS_act - TS_ant >= TS_fin)
    {
      TS_ant = TS_act;
      // Se toma el dato
      DHT.read();
      Humedad = DHT.getHumidity();
      Temperatura = DHT.getTemperature();
    }

    // Pantalla
    if (TP_act - TP_ant >= TP_fin)
    {
      TP_ant = TP_act;
      // Se muestra el dato en la pantalla
      if (WifiCon = 1)
      {
        tft.drawString("WiFi", 200, 5, 2);
      }
      tft.drawString((String(Humedad, 2)) + '%', 10, 100, 4);
      tft.drawString((String(Temperatura, 2)) + 'C', 140, 100, 4);
      Serial.print(Humedad);
      Serial.print("% \t");
      Serial.print(Temperatura);
      Serial.println("C");
    }

    // Transmision
    if (Tx_act - Tx_ant >= Tx_fin)
    {
      Tx_ant = Tx_act;
      // Se envia el dato a UbiDots
      WifiCon = 1;
      if (WiFi.status() != WL_CONNECTED)
      {
        WifiCon = 0;
      }
      if (!ubidots.connected())
      {
        ubidots.reconnect();
      }
      ubidots.add("Humedad", Humedad);
      ubidots.add("Temperatura", Temperatura);
      ubidots.publish(DEVICE_LABEL);
      ubidots.loop();
    }

    // Recepcion
    if (Rx_act - Rx_ant >= Rx_fin)
    {
      Rx_ant = Rx_act;
      // Se verifica datos de Ubidots
      if (!ubidots.connected())
      {
        ubidots.reconnect();
        ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL1); // Insert the device and variable's Labels, respectively
        ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL2); // Insert the device and variable's Labels, respectively
      }
      ubidots.loop();
    }
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}