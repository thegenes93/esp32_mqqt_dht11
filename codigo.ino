#include "WiFi.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>

/* Definições do LED */
#define PIN_LED     25

/* Definicoes do sensor DHT */
#define DHTPIN 27     //GPIO que está ligado o pino de dados do sensor

#define DHTTYPE DHT11  //sensor em utilização: DHT11
//#define DHTTYPE DHT22   
//#define DHTTYPE DHT21

/* Defines do MQTT */
#define TOPICO_SUBSCRIBE_LED         "topico_liga_desliga"
#define TOPICO_PUBLISH_TEMPERATURA   "topico_sensor_temperatura"
#define TOPICO_PUBLISH_DISTANCIA     "topico_sensor_distancia"
#define TOPICO_PUBLISH_UMIDADE       "topico_sensor_umidade"

#define ID_MQTT  "mqttdash"     //id mqtt (para identificação de sessão)
//IMPORTANTE: este deve ser único no broker 

/* Variaveis, constantes e objetos globais */
DHT dht(DHTPIN, DHTTYPE);

// set the LCD number of columns and rows

LiquidCrystal_I2C lcd(0x27, 16, 2);

const char* SSID = "*****"; // SSID / nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "*******"; // Senha da rede WI-FI que deseja se conectar

const char* BROKER_MQTT = "mqtt.eclipseprojects.io"; //URL do broker MQTT que se deseja utilizar
int BROKER_PORT = 1883; // Porta do Broker MQTT
boolean alarme = false;
boolean ativar = false;
const int pinoSensor = 8; //PINO DIGITAL UTILIZADO PELO SENSOR
const int pinoBuzzer = A0; //PINO DIGITAL PELO SENSOR,
const int pinoRele = A1; //PINO DIGITAL PELO SENSOR,
//Variáveis e objetos globais
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient

// set LCD address, number of columns and rows

// if you don't know your display address, run an I2C scanner sketch


/* Prototypes */
float faz_leitura_temperatura(void);
float faz_leitura_umidade(void);
void initWiFi(void);
void initMQTT(void);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT(void);
void reconnectWiFi(void);
void VerificaConexoesWiFIEMQTT(void);

/*
   Implementações
*/

/* Função: faz a leitura de temperatura (sensor DHT22)
   Parametros: nenhum
   Retorno: temperatura (graus Celsius)
*/
float faz_leitura_temperatura(void)
{
  float t = dht.readTemperature();
  float result;

  if (! (isnan(t)) )
    result = t;
  else
    result = -99.99;

  return result;
}

/* Função: faz a leitura de umidade relativa do ar (sensor DHT22)
   Parametros: nenhum
   Retorno: umidade (0 - 100%)
*/

float faz_leitura_umidade(void)
{
  float h = dht.readHumidity();
  float result;

  if (! (isnan(h)) )
    result = h;
  else
    result = -99.99;

  return result;
}

/* Função: inicializa e conecta-se na rede WI-FI desejada
   Parâmetros: nenhum
   Retorno: nenhum
*/
void initWiFi(void)
{
  delay(10);
  Serial.println("------Conexao WI-FI------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(SSID);
  Serial.println("Aguarde");

  reconnectWiFi();
}

/* Função: inicializa parâmetros de conexão MQTT(endereço do
           broker, porta e seta função de callback)
   Parâmetros: nenhum
   Retorno: nenhum
*/

void initMQTT(void)
{
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
  MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  String msg;

  /* obtem a string do payload recebido */
  for (int i = 0; i < length; i++)
  {
    char c = (char)payload[i];
    msg += c;
  }

  Serial.print("Chegou a seguinte string via MQTT: ");
  Serial.println(msg);

  /* toma ação dependendo da string recebida */
  if (msg.equals("1"))
  {
    digitalWrite(PIN_LED, HIGH);
    Serial.print("LED aceso comando MQTT");
  }

  if (msg.equals("0"))
  {
    digitalWrite(PIN_LED, LOW);
    Serial.print("LED apagado comando MQTT");
  }
}

/* Função: reconecta-se ao broker MQTT
   Parâmetros: nenhum
   Retorno: nenhum
*/
void reconnectMQTT(void)
{
  while (!MQTT.connected())
  {
    Serial.print("* Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT))
    {
      Serial.println("Conectado com sucesso ao broker MQTT!");
      MQTT.subscribe(TOPICO_SUBSCRIBE_LED);
    }
    else
    {
      Serial.println("Falha ao reconectar no broker.");
      Serial.println("Havera nova tentatica de conexao em 2s");
      delay(2000);
    }
  }
}

/* Função: verifica o estado das conexões WiFI e ao broker MQTT.*/
void VerificaConexoesWiFIEMQTT(void)
{
  if (!MQTT.connected())
    reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita

  reconnectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}

/* Função: reconecta-se ao WiFi
   Parâmetros: nenhum
   Retorno: nenhum
*/
void reconnectWiFi(void)
{
  //se já está conectado a rede WI-FI, nada é feito.
  //Caso contrário, são efetuadas tentativas de conexão
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso na rede ");
  Serial.print(SSID);
  Serial.println("IP obtido: ");
  Serial.println(WiFi.localIP());
}

/* Função de setup */
void setup()
{
  Serial.begin(115200);

  /* Configuração do pino ligado ao LED como output
     e inicialização do mesmo em LOW */
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Ativar alarme ?");

  /* Inicializacao do sensor de temperatura */
  dht.begin();

  /* Inicializa a conexao wi-fi */
  initWiFi();

  /* Inicializa a conexao ao broker MQTT */
  initMQTT();
}

void estadoInicial(boolean ativar)
{
  lcd.clear();
  if (ativar == true) {
    lcd.print("Alarme Ativado");
  } else {
    lcd.print("Alarme Desativado");
  }
  analogWrite(pinoRele, 255);
  noTone(pinoBuzzer);
}

void estadoAlerta(boolean alarme) {
  if (alarme == true) {
    Serial.println("objeto dectado");
    lcd.clear();
    lcd.print("Alarme Acionado");
    delay(5000);
    analogWrite(pinoRele, 0);
    tone(pinoBuzzer, 250);
  }
}

/* Loop principal */
void loop()
{
  char temperatura_str[10] = {0};
  char umidade_str[10]     = {0};

  /* garante funcionamento das conexões WiFi e ao broker MQTT */
  VerificaConexoesWiFIEMQTT();

  /* Compoe as strings a serem enviadas pro dashboard (campos texto) */
  sprintf(temperatura_str, "%.2fC", faz_leitura_temperatura());
  sprintf(umidade_str, "%.2f", faz_leitura_umidade());

  /*  Envia as strings ao dashboard MQTT */
  MQTT.publish(TOPICO_PUBLISH_TEMPERATURA, temperatura_str);
  MQTT.publish(TOPICO_PUBLISH_UMIDADE, umidade_str);

  /* keep-alive da comunicação com broker MQTT */
  MQTT.loop();

  /* Refaz o ciclo após 2 segundos */
  delay(2000);
}
