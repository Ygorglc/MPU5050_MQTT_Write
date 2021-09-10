// Basic demo for accelerometer readings from Adafruit MPU6050
//#include <ESP8266WiFi.h>// Essa biblioteca é usada quando se usar o esp8266
#include <WiFi.h>// Essa biblioteca é usada quando se usar o esp32
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
char start[7];
const char* ssid = "Link Start"; 
const char* password =  "1234#563";
const char* mqttServer = "broker.mqttdashboard.com";
const int mqttPort = 1883; 
const char* mqttUser = "ygor_12345678"; 
const char* mqttPassword = "12345678"; 

WiFiClient espClient;
PubSubClient client(espClient);

//Função responsavel por ler os dados do Brocker e converter estes em valores usaveis; 
void callback(char* topic, byte* payload, unsigned int length) {
  char accerchar[6];
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    start[i] = (char)payload[i];
  }
}

void setup(void) {
  Serial.begin(115200); //configura comunicação serial com 115200 bps
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  //// Fazendo a conexão com o wi-fi
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) 
  {   
     delay(100);
    Serial.println("Conectando a WiFi..");
  }
  Serial.println("Conectado!"); 
  /////Fazendo a conexão com o brocker MQTT
  client.setServer(mqttServer, mqttPort); 
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Conectando ao servidor MQTT...");
    
    if (client.connect("Projeto", mqttUser, mqttPassword ))
    {
 
      Serial.println("Conectado ao servidor MQTT!");  
 
    } else {
 
      Serial.print("Falha ao conectar ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
  client.subscribe("StartAccer"); // Se inscreve no Tópico StartAccer para receber a váriavel;
  
  delay(100);
}

void print_accelaration(){
  unsigned long Accer_time = millis(); //Coleta o tempo inicial para coletar a aceleração em função do tempo
  int i = 0;
  
  while(i!=1000){ 
    //Funções reponsaveis por se comunicar com o MPU6050 e instanciar suas variáveis;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    Serial.print(a.acceleration.x,10); // Coleta o valor da aceleração no eixo x;
    Serial.print("   ");
    Serial.print(millis()-Accer_time); // Função responsavel por coletar a variação do tempo
    Serial.println();
    i++;

    String msg;
    msg = String(a.acceleration.x);
    char message[58];
    msg.toCharArray(message,58);
    client.publish("Acceleration", message);

    String msg2;
    msg2 = String(millis()-Accer_time);
    char message2[58];
    msg2.toCharArray(message2,58);
    client.publish("Time_Accer", message2);
      
    delay(5); // Esse delay serve como ajuste de tempo entre as leituras do MPU6050
  }
  start[0] = 'B';
  Serial.println("//////////////////");
}

void loop() {
  // Esse switch faz a verificação se a variavel start é A, a mesma vem do brocker, para inicializar a coleta do sensor;
  switch(start[0]){
    case 'A':

      print_accelaration(); //Chama a função de coleta e envio dos dados do MPU6050;
        
  }  
  
  client.loop(); // Função importante, sem ela as configurações de cliente do broker não vão ser executadas;
  delay(500);
} 
