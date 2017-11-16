/*
  Name:		SimpleClass_Applcation.ino
  Created:	5/26/2017 4:18:28 PM
  Author:	Felipe Fonseca
  Description: Versão de Código para o Trocador de Calor de forma a simplificar
  a operçãoo para tornar possível a utilização em aulas
*/

//includes

//Libs dos sensores
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ultrasonic.h>

//display lcd
#include <LiquidCrystal.h>

//timed action
//#include <Utility.h>
//#include <TimedAction.h>

//lib i2c
#include <Wire.h>

//gerar sinais
#include <waveforms.h>

//Pinos de entrada e sa�da

#define but1 A1  //no programa de testes 42; no original, trocar por A1  //
#define but2 A2  //no programa de testes 44; no original trocar por A2
#define pot A0 //no programa de testes A7; no original trocar por A0

#define mode_switch 42  //no programa de testes 43; no original trocar por 42
#define emergency_button 43 //no programa de testes 51; no original  trocar por 43

#define ultrassonico_echo	50      //no original trocar para 50
#define ultrassonico_trigger 48     //no original trocar para 48
#define ONE_WIRE_BUS 52             //no original trocar para 52  - terminal do conjunto de sensores
#define hf_sensor 9                //no original trocar para 9 -> pino de interrup��o para calculo da vaz�o agua fria
#define TEMPERATURE_PRECISION 12


#define led1 44                  // utilizado para sinalização geral
#define led2 45                  // utilizado para sinalização geral
#define led_heater 47            // no programa de testes 37; no original trocar para 47

#define inversor_rele 49        //no programa de teste 47;no original trocar para 49
#define heater_rele 10          //no programa de testes 49;no original trocar para 10

#define MAX_MENU_ITENS 2         //numero de telas no menu

//testes em prototipo
#define LDR_PIN A3
#define LM35_PIN A4
#define RXLED 3
#define oneHzSample 1000000/maxSamplesNum //gerador de sinais
#define SIMULATORSAMPLETIME 500

float LM35_value;
float LDR_value;

//vari�veis internas

//armazenar valores de entrada
bool switch_state;
int pot_value;
bool emergency_status;

volatile int pump_onoff;
volatile int heater_onoff;
volatile float remote_pumpspeed;

float temp[4];
double vazao_quente;
float vazao_fria;
volatile int flow_frequency;
float vazao1_sf; 

//estrutura de dados para envio i2c
typedef struct processData {
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float hotflow;
  float coldflow;
  float pump_speed;
  byte bstatus;
  byte chksum;
};

typedef union I2C_Send { //compartilha a mesma área de memória
  processData data;
  byte I2C_packet[sizeof(processData)];
};


I2C_Send send_info;
int command; //processar o indice de comando enviado pelo Rpi

//array de bytes auxilar para receber a velocidade da bomba
byte data[4];

//estrutura para receber um float para alterar velocidade
typedef union PumpDataSpeed {
  float fspeed;
  byte bspeed[4];
};

//variáveis utilizadas no calculo de vazao de agua fria/quente
unsigned long currentTime;
unsigned long cloopTime;
long microsec;
float cmMsec;
float nivel;
//float vazao1_sf;   //se for necessário para a função de vazao quente, descomentar

//variáveis para o temporizador de vazão de água quente
unsigned long hotflow_time;
int hotflow_interval;
unsigned long last_hotflow_time;

//variáveis auxiliares para operação local
char flag_button1 = 0x00;
char flag_button2 = 0x00;
char flag_emergency = 0x00;
String mode = "";

//variáveis auxiliares para comando
float pot_value_mapped;

//variáveis para o simulador
unsigned long current_sim_time;
unsigned long simulator_time_elapsed;
int indexwave;

//inicialização de objetos
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress deviceID[] =
{
  { 0x28, 0xFF, 0x46, 0x02, 0x54, 0x16, 0x04, 0xF8 },
  { 0x28, 0xFF, 0xD3, 0xE6, 0x53, 0x16, 0x04, 0xA0 },
  { 0x28, 0xFF, 0xE8, 0xF0, 0x53, 0x16, 0x04, 0x65 },
  { 0x28, 0x1D, 0x9D, 0x27, 0x00, 0x00, 0x80, 0x2E }
};
//delay necessário para leitura dos sensores. Depende da resolução
int delayTempRead = 750 / (1 << (12-TEMPERATURE_PRECISION));
int lastTempRequest = millis();

Ultrasonic ultrasonic(ultrassonico_trigger, ultrassonico_echo);

LiquidCrystal lcd(41, 11, 12, 40, 13, 38);  //no original deve-se utilizar  LiquidCrystal lcd(41, 11, 12, 40, 13, 38);

//funções
void ReadPotentiometer();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

//funções de navegação  - Display LCD
void Menu(int op);
void EmergencyStatus();

//funções de leitura de grandezes
void flow();                //função de interrupção para o calculo de vazao de água fria
void PumpSpeed(float ref);  //função para alterar a velocidade da bomba
void VazaoAguaFria();       //função para calcular a vazão de água fria
void VazaoAguaQuente();     //função para calcular a vazão de água quente
void Temperaturas();        //função para calcular as temperaturas dos sensores
void new_Temperaturas();

//funções do estado do arduino
void LocalState();
void RemoteState();
void emergencia();

//funções para fazerem a leitura periodica dos valores analógicos
void runReads();
void refresh_I2C_Packet();

//função para receber o valor em bytes via i2c e retornar a velocidade em float
void parseSpeed(byte data[]);

//testes em prototipo
void Temperaturas2();
void Simulator();

// the setup function runs once when you press reset or power the board
void setup() {
  //painel
  pinMode(but1, INPUT_PULLUP);
  pinMode(but2, INPUT_PULLUP);
  pinMode(pot, INPUT);
  pinMode(mode_switch, INPUT);
  pinMode(emergency_button, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led_heater, OUTPUT);

  //sensores
  pinMode(ultrassonico_echo, INPUT);
  pinMode(ultrassonico_trigger, OUTPUT);
  pinMode(hf_sensor, INPUT_PULLUP);
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  //reles
  pinMode(heater_rele, OUTPUT);
  pinMode(inversor_rele, OUTPUT);

  //começar inversor desligado
  digitalWrite(inversor_rele, HIGH);

  //habilitar a interrupção para medição de vazão fria
  attachInterrupt(hf_sensor, flow, RISING);

  //iniciar sensores de temperatura
  sensors.begin();
  for (byte i = 0; i <= 4; i++) {
    sensors.setResolution(deviceID[i], TEMPERATURE_PRECISION);
  }
  //modo de leitura assincrona
  sensors.setWaitForConversion(false);
  delayTempRead = 750 / (1 << (12 - TEMPERATURE_PRECISION));

  //iniciar lcd
  lcd.begin(20, 4); //no original utilizar 20,4

  //resolução de escrita do DAC
  analogWriteResolution(10);

  //variáveis que guarda o status da bomba e do aquecedor
  pump_onoff = 0;
  heater_onoff = 0;

  //inicialização da serial
  Serial.begin(115200);

  //inicialização de variáveis
  vazao_fria = 0;
  vazao_quente = 0;
  hotflow_interval = 300; //tempo de execução do intervalo de medição de vazão quente

  //inicialização da estrutura i2c
  send_info.data.chksum = 27;
  Wire.begin(12); //arduino iniciado no endereço 12
  Wire.onReceive(receiveEvent); //callback para recebimento de comandos
  Wire.onRequest(requestEvent); //callback para responder à requisições

  //semente para o simulador
  randomSeed(analogRead(LM35_PIN));
  indexwave = 0;
}


// the loop function runs over and over again until power down or reset
void loop() {

  // le o estado da chave
  switch_state = digitalRead(mode_switch);

  /* lê se o botão de emergencia está acionado ou não
      cobre os casos de pressionar o botão e despressionar o botão
  */
  if (digitalRead(emergency_button) == LOW) { //presisonado botão
    emergencia();
  }
  else if (!digitalRead(emergency_button) == LOW && flag_emergency) { //despressionado o botão
    flag_emergency = 0x00;
    emergency_status = 0;
    lcd.clear();
    if (switch_state) {
      RemoteState();
    }
    else {
      LocalState();
    }
  }
  else { //tudo normal
    if (switch_state) {
      RemoteState();
    }
    else {
      LocalState();
    }
  }
}

//função de interrupção para calcular a vazão
void flow() {
  flow_frequency++;
}

//funções que gerenciam o estado do arduino

void LocalState() {

  //efetua a leitura das grandezas
  runReads();

  //atualiza o lcd com os novos valores
  Menu(0);

  if (!digitalRead(but1)) flag_button1 = 0x01;
  if (!digitalRead(but2)) flag_button2 = 0x01;

  if (digitalRead(but1) && flag_button1) {
    flag_button1 = 0x00;
    //troca o estado da bomba
    digitalWrite(inversor_rele, pump_onoff);  //o estado da bomba � invertido
    //digitalWrite(led_pump, !pump_onoff);  -> N�o � para utilizar o led
    pump_onoff = !pump_onoff;
  }

  if (digitalRead(but2) && flag_button2) {
    flag_button2 = 0x00;
    //troca o estado do aquecedor
    digitalWrite(heater_rele, !heater_onoff);
    digitalWrite(led_heater, !heater_onoff);
    heater_onoff = !heater_onoff;
  }

  //controlar a velocidade da bomba
  ReadPotentiometer();
  PumpSpeed(pot_value_mapped);

  //enquanto modo local, a variável remote_pumpspeed deve ser atualizada para quando
  //ocorrer a mudança para o modo remoto, por exemplo, as informações sejam corretas
  remote_pumpspeed = pot_value_mapped;

  //atualiza a estrutura de envio de dados via i2c
  refresh_I2C_Packet();
}

void RemoteState() {
  //faz a leitura das variáveis
  runReads();

  //atualiza o lcd com os novos valores
  Menu(1);

  //atualiza a estrutura de envio de dados via i2c
  refresh_I2C_Packet();
}

void emergencia() {
  //digitalWrite(led_flow_mode, LOW);//Apaga LED 1
  digitalWrite(led_heater, LOW);//Apaga LED 2
  digitalWrite(inversor_rele, HIGH);//Desliga a bomba
  digitalWrite(heater_rele, LOW);//Desliga o aquecedor

  //atualiza variável de emergência
  emergency_status = 1;

  //vari�veis auxiliares
  pump_onoff = 0x00;
  heater_onoff = 0x00;
  remote_pumpspeed = 0.0;

  //para evitar blinkar o lcd
  if (!flag_emergency) {
    lcd.clear();
    EmergencyStatus();
  }
  flag_emergency = 0x01;
}

//funções para gerenciar a exibição do lcd

void EmergencyStatus() {
  lcd.setCursor(0, 0);
  lcd.print("Emergencia");
  lcd.setCursor(0, 1);
  lcd.print("Comandos");
  lcd.setCursor(0, 2);
  lcd.print("Bloqueados");
}

void Menu(int op) {

  mode = (op == 1) ? "Remote Mode" : "Local  Mode";

  lcd.setCursor(0, 0);
  lcd.print(mode);

  lcd.setCursor(0, 1);
  lcd.print("TA: ");
  lcd.print(temp[0]);
  lcd.print("  ");
  lcd.print("TB: ");
  lcd.print(temp[1]);

  lcd.setCursor(0, 2);
  lcd.print("TC: ");
  lcd.print(temp[2]);
  lcd.print("  ");
  lcd.print("TD: ");
  lcd.print(temp[3]);

  lcd.setCursor(0, 3);
  lcd.print("V1: ");
  lcd.print(vazao_quente);
  lcd.print("  ");
  lcd.print("V2: ");
  lcd.print(vazao_fria);

}

//funções de leitura (e escrita) das grandezas

void Temperaturas() {

  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus

  sensors.requestTemperatures();

  // print the device information
  for (byte i = 0; i <= 4; i++)
  {
    temp[i] = sensors.getTempC(deviceID[i]);
  }
}

void new_Temperaturas(){
  if(millis() - lastTempRequest > 1500){ //delayTempRead
    for (byte i = 0; i <= 4; i++)
    {
      temp[i] = sensors.getTempC(deviceID[i]);
    }
    lastTempRequest = millis();
    sensors.requestTemperatures();
  }
  
}

void VazaoAguaFria() {
  currentTime = millis();
  // Every second, calculate litres/hour
  if (currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min
    noInterrupts();
    vazao_fria = (flow_frequency / 7.5); // (Pulse frequency) / 7.5Q = flowrate in L/min
    flow_frequency = 0; // Reset Counter
    interrupts();
  }
}

void VazaoAguaQuente() {

  hotflow_time = millis();
  if(hotflow_time - last_hotflow_time > hotflow_interval){
    microsec = ultrasonic.timing();
    cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
    nivel = 11.46 - cmMsec;
    vazao1_sf = (0.0537) * pow((nivel * 10), 1.4727);
    if (vazao1_sf > 1) {
      vazao_quente = 0.75 * vazao_quente + 0.25 * vazao1_sf;
    }
    last_hotflow_time = hotflow_time;
  }
}

void Temperaturas2() {
  LM35_value = analogRead(LM35_PIN) * 0.48875855;
  LDR_value = analogRead(LDR_PIN) / 10.0;
  temp[0] = LM35_value;
  temp[1] = LDR_value;
}

void Simulator() {
  current_sim_time = millis();
  if (current_sim_time >= simulator_time_elapsed + SIMULATORSAMPLETIME) {

    simulator_time_elapsed = current_sim_time;

    for (int i = 0; i < 3; ++i) {
      temp[i] = mapfloat(random(1024), 0, 1023, 10, 50);
    }

    //ultima temperatura com valor senoidal
    temp[3] = mapfloat(waveformsTable[0][indexwave], 0, 4095, 0, 100);
    indexwave++;
    if (indexwave == maxSamplesNum) {
      indexwave = 0;// Reset the counter to repeat the wave
    }

    vazao_quente = mapfloat(random(1024), 0, 1023, 0, 30);
    vazao_fria = mapfloat(random(1024), 0, 1023, 0, 30);
  }
}


void PumpSpeed(float ref) {
  if (ref > 100)
    ref = 100;
  else if (ref < 0)
    ref = 0;
  analogWrite(DAC0, ref * 9.43 + 80);
}

void ReadPotentiometer() {
  pot_value = analogRead(pot);
  pot_value_mapped = mapfloat(pot_value, 0, 1023, 0, 100);
}

void runReads() {
  
    //Temperaturas();
    VazaoAguaFria();
    VazaoAguaQuente();
    new_Temperaturas();
  
  //Temperaturas2();
  //Simulator();
}


//funções callback do i2c
void receiveEvent(int Nbytes) {
  command = Wire.read();

  switch (command) {
    case 49: //comando de teste
      //comando liga bomba
      digitalWrite(inversor_rele, LOW); //o estado da bomba é invertido
      pump_onoff = 1;
      break;

    case 50:
      //comando desliga bomba
      digitalWrite(inversor_rele, HIGH);
      pump_onoff = 0;
      break;

    case 51:
      //comando liga aquecedor
      digitalWrite(heater_rele, HIGH);
      digitalWrite(led_heater, HIGH);
      heater_onoff = 1;
      break;

    case 52:
      //comando desliga aquecedor
      digitalWrite(heater_rele, LOW);
      digitalWrite(led_heater, LOW);
      heater_onoff = 0;
      break;

    case 53:
      //comando alterar velocidade da bomba
      int i = 0;
      while (Wire.available()) {
        data[i] = Wire.read();
        i = i + 1;
      }
      parseSpeed(data);
      break;
  }
}

void requestEvent() {
  if (command == 54) {
    Wire.write(send_info.I2C_packet, sizeof(processData));
  }
}

void parseSpeed(byte data[]) {
  //o primeiro byte é o número de bytes do envio; deve-se ignorar
  PumpDataSpeed speed;
  speed.bspeed[0] = data[1];
  speed.bspeed[1] = data[2];
  speed.bspeed[2] = data[3];
  speed.bspeed[3] = data[4];
  //Serial.println(speed.fspeed);
  remote_pumpspeed = speed.fspeed;
  PumpSpeed(remote_pumpspeed); //envia o comando de velocidade para a bomba fisicamente
}

void refresh_I2C_Packet() {
  send_info.data.temp1 = temp[0];
  send_info.data.temp2 = temp[1];
  send_info.data.temp3 = temp[2];
  send_info.data.temp4 = temp[3];

  //se a bomba estiver desligada, ignorar o valor do potenciometro
  if (pump_onoff) {
    if (switch_state) {
      send_info.data.pump_speed = remote_pumpspeed;
    }
    else {
      send_info.data.pump_speed = pot_value_mapped;
    }
  }
  else {
    send_info.data.pump_speed = 0.0;
  }


  send_info.data.hotflow = vazao_quente;
  send_info.data.coldflow = vazao_fria;
  bitWrite(send_info.data.bstatus, 0, pump_onoff);
  bitWrite(send_info.data.bstatus, 1, heater_onoff);
  bitWrite(send_info.data.bstatus, 2, switch_state);
  bitWrite(send_info.data.bstatus, 3, emergency_status);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
