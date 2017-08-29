#include <Wire.h>

/*
Name:    Parse Json Data
Created:  10/07/2017 22:30
Author: Felipe Fonseca
Description: Primeira vers�o do c�digo para testar o funcionamento dos equipamentos
*/

//includes

//Libs dos sensores
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ultrasonic.h>

//display lcd
#include <LiquidCrystal.h>

//lib i2c
#include <Wire.h>

//timed action
//#include <Utility.h>
//#include <TimedAction.h>

//Pinos de entrada e saída

#define but1 A1  //no programa de testes 42; no original, trocar por A1
#define but2 A2  //no programa de testes 44; no original trocar por A2
#define pot A0 //no programa de testes A7; no original trocar por A0

#define mode_switch 42  //no programa de testes 43; no original trocar por 42
#define emergency_button 43 //no programa de testes 51; no original  trocar por 43

#define ultrassonico_echo 50      //no original trocar para 50
#define ultrassonico_trigger 48     //no original trocar para 48
#define ONE_WIRE_BUS 52             //no original trocar para 52  - terminal do conjunto de sensores
#define hf_sensor 9                //no original trocar para 9 -> pino de interrup��o para calculo da vaz�o agua fria


#define led_flow_mode 45         // no original trocar para 45
#define led_temp_mode 44         // no original trocar para 44
#define led_heater 47          // no original trocar para 47

#define inversor_rele 49        //no programa de teste 47;no original trocar para 49
#define heater_rele 10      //no programa de testes 49;no original trocar para 10

#define MAX_MENU_ITENS 5        //numero de telas no menu

//testes em protótipo
#define LDR_PIN A3
#define LM35_PIN A4
#define RXLED 3

//vari�veis internas
#define TEMPERATURE_PRECISION 0

//armazenar valores de entrada
int switch_state;
int pot_value;
int pump_onoff;
int heater_onoff;

float temp[4];
double vazao_quente;
float vazao_fria;
volatile int flow_frequency;
int emergency_status;

//estrutura de dados para envio i2c
typedef struct processData{
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float hotflow;
  float coldflow;
  byte pump_speed;
  byte bstatus;
  byte chksum;
};

typedef union I2C_Send{ //compartilha a mesma área de memória
  processData data;
  byte I2C_packet[sizeof(processData)];
};

I2C_Send send_info;
int command; //processar o indice de comando enviado pelo Rpi
byte data[12];  //processar as informações de comando

//estrutura para receber um float para alterar velocidade
typedef union PumpDataSpeed {
  float fspeed;
  byte bspeed[4];
};

//vari�veis utilizadas no calculo de vazao de agua fria/quente
unsigned long currentTime;
unsigned long cloopTime;
long microsec;
float cmMsec;
float nivel;
//float vazao1_sf;   //se for necessário para a função de vazao quente, descomentar

//variáveis auxiliares para navegação
char menu = 0x01;
char flag_button1 = 0x00;
char flag_button2 = 0x00;
char flag_emergency = 0x00;

//variáveis auxiliares para comando
char pumpstatus;
float pot_value_mapped;
char heaterstatus;

//variáveis para o protótipo de teste em casa
float LM35_value;
float LDR_value;
int LM35_read;
int LDR_read;

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

Ultrasonic ultrasonic(ultrassonico_trigger, ultrassonico_echo);

LiquidCrystal lcd(41, 11, 12, 40, 13, 38);  //no original deve-se utilizar  LiquidCrystal lcd(41, 11, 12, 40, 13, 38);

//funções

//funções de navegação  - Display LCD
void changeMenu();
void dispMenu();
void Introduction();
void ShowTemp();
void ShowFlow();
void PumpCommand();
void HeaterCommand();
void ChangePID();
void EmergencyStatus();

//funções de leitura de grandezes
void flow();                //função de interrupção para o calculo de vazao de �gua fria
void PumpSpeed(float ref);  //função para alterar a velocidade da bomba
void VazaoAguaFria();       //função para calcular a vaz�o de �gua fria
void VazaoAguaQuente();     //função para calcular a vaz�o de �gua quente
void Temperaturas();        //função para calcular as temperaturas dos sensores

//função de emergencia
void emergencia();

//funções auxiliares
void set_rnd_values();     //função para gerar valores para as grandezas para testar o menu

//funções para fazerem a leitura periodica dos valores
void runReads();

//funções para teste em protótipo
void CalcParams();
void Temperaturas2();

//função para receber o valor em bytes via i2c e retornar a velocidade em float
void parseSpeed(byte data[]);

// inicializações e declarações de variáveis
void setup() {

  //painel
  pinMode(but1, INPUT_PULLUP);
  pinMode(but2, INPUT_PULLUP);
  pinMode(pot, INPUT);
  pinMode(mode_switch, INPUT);
  pinMode(emergency_button, INPUT_PULLUP);
  pinMode(led_flow_mode, OUTPUT);
  pinMode(led_temp_mode, OUTPUT);
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

  //habilitar a interrupção
  attachInterrupt(hf_sensor, flow, RISING);

  //iniciar lcd
  lcd.begin(20,4);  //no original utilizar 20,4

  //iniciar sensores de temperatura
  sensors.begin();
  for (byte i = 0; i <= 4; i++){
    sensors.setResolution(deviceID[i], TEMPERATURE_PRECISION);
  }

  //testes de comunicação serial
  pinMode(RXLED,OUTPUT);

  pumpstatus = 0x00;
  heaterstatus = 0x00;

  //resolução de escrita
  analogWriteResolution(10);

  //teste
  set_rnd_values();
  pump_onoff = 0;
  heater_onoff = 0;

  //inicializa��o da serial
  Serial.begin(115200);

  //inicialização da estrutura i2c
  Wire.begin(12); //arduino iniciado no endereço 4
  Wire.onReceive(receiveEvent); //callback para recebimento de comandos
  Wire.onRequest(requestEvent); //callback para responder à requisições

  //inicializaçãop das estruturas de protótipo
  pinMode(LDR_PIN,INPUT);
  pinMode(LM35_PIN,INPUT);
  //analogReference(INTERNAL);
  LDR_value = 0.0;
  LDR_read = 0;
  LM35_value = 0.0;
  LM35_read = 0;
  
}

// the loop function runs over and over again until power down or reset
void loop() {

  if (digitalRead(emergency_button) == LOW){
    emergencia();
  }
  else if (!digitalRead(emergency_button) == LOW && flag_emergency){
    flag_emergency = 0x00;
    lcd.clear();
    menu = 0x01;
  }
  else{
    /*aqui terá que ser feita uma lógica para testar qual o estado da chave para selecionar qual é o modo de operação
     * do arduino. por enquanto, deixando nos dois modos.
     */

    //funções de modo local (display lcd). 
    changeMenu();
    dispMenu();

    //funções de modo remoto
    runReads();
  }
  
}

void emergencia(){
  digitalWrite(led_flow_mode, LOW);//Apaga LED 1
  digitalWrite(led_heater, LOW);//Apaga LED 2
  digitalWrite(led_temp_mode, LOW);//Apaga LED 4
  digitalWrite(inversor_rele, HIGH);//Desliga a bomba
  digitalWrite(heater_rele, LOW);//Desliga o aquecedor

  //vari�veis auxiliares
  pumpstatus = 0x00;
  heaterstatus = 0x00;
  pump_onoff = 0x00;
  heater_onoff = 0x00;

  //para evitar blinkar o lcd
  if (!flag_emergency){
    lcd.clear();
    EmergencyStatus();
  }
  flag_emergency = 0x01;
}

void flow() // Interrupt function
{
  flow_frequency++;
}

void set_rnd_values(){
  vazao_fria = 10;
  vazao_quente = 20;
  temp[0] = 5;
  temp[1] = 10;
  temp[2] = 15;
  temp[3] = 20;

  //inicialização aleatória da estrutura i2c para envio
  send_info.data.temp1 = temp[0];
  send_info.data.temp2 = temp[1];
  send_info.data.temp3 = temp[2];
  send_info.data.temp4 = temp[3];
  send_info.data.hotflow = vazao_fria;
  send_info.data.coldflow = vazao_quente;
  send_info.data.pump_speed = 62;
  bitWrite(send_info.data.bstatus,0,1);
  bitWrite(send_info.data.bstatus,1,1);
  send_info.data.chksum = 27;

}

//função que controla a mudan�a de menu quando o bot�o � solto
void changeMenu(){
  if (!digitalRead(but1)) flag_button1 = 0x01;
  if (digitalRead(but1) && flag_button1){  //apenas quando o bot�o � solto ocorre a execu��o
    flag_button1 = 0x00;
    lcd.clear();
    menu++;

    if (menu > MAX_MENU_ITENS) menu = 0x01;  //volta para o come�o
  }
}

//função para chamar os menus
void dispMenu(){
  switch (menu){
  case 0x01:
    Introduction();
    break;
  case 0x02:
    ShowTemp();
    break;
  case 0x03:
    ShowFlow();
    break;
  case 0x04:
    PumpCommand();
    break;
  case 0x05:
    HeaterCommand();
    break;
  }

}

void Introduction(){
  lcd.setCursor(0, 0);
  lcd.print("Trocador de Calor");
  lcd.setCursor(0, 1);
  lcd.print("C1:");
  lcd.print(digitalRead(emergency_button));
  lcd.print(" C2:");
  lcd.print(digitalRead(mode_switch));
}

void EmergencyStatus(){
  lcd.setCursor(0, 0);
  lcd.print("Emergencia!");
  lcd.setCursor(0, 1);
  lcd.print("Cmds Bloqueados!!");
}

void ShowTemp(){
  Temperaturas2();
  lcd.setCursor(0, 0);
  lcd.print("TA:");
  lcd.print(temp[0]);
  lcd.print(",TB:");
  lcd.print(temp[1]);
  lcd.setCursor(0, 1);
  lcd.print("TC:");
  lcd.print(temp[2]);
  lcd.print(",TD:");
  lcd.print(temp[3]);
}

void ShowFlow(){
  //VazaoAguaFria();
  //VazaoAguaQuente();
  lcd.setCursor(0, 0);
  lcd.print("VA:");
  lcd.print(vazao_quente);
  lcd.print("VB:");
  lcd.print(vazao_fria);
}

void PumpCommand(){

  if (pump_onoff){
    lcd.setCursor(0, 0);
    lcd.print("bomba on ");
  }
  else{
    lcd.setCursor(0, 0);
    lcd.print("bomba off");
  }

  if (!digitalRead(but2)) flag_button2 = 0x01;
  if (digitalRead(but2) && flag_button2){
    flag_button2 = 0x00;
    pumpstatus++;
    if (pumpstatus > 2) pumpstatus = 0x01;

    //ligar ou desligar a bomba
    switch (pumpstatus){
    case 0x01:
      lcd.setCursor(0, 0);
      lcd.print("bomba on");
      digitalWrite(inversor_rele, LOW); //o estado da bomba � invertido
      pump_onoff = 1;
      break;
    case 0x02:
      lcd.setCursor(0, 0);
      lcd.print("bomba off");
      digitalWrite(inversor_rele, HIGH); //o estado da bomba � invertido
      pump_onoff=0;
      break;
    }
  }
  //caso a bomba esteja ligada permitir alterar a velocidade da bomba
  if (pumpstatus == 0x01){
    pot_value = analogRead(pot);
    pot_value_mapped = map(pot_value, 0, 1023, 0, 100);
    PumpSpeed(pot_value_mapped);
    lcd.setCursor(0, 1);
    lcd.print("rot:");
    lcd.print(pot_value_mapped);
  }
}

void PumpSpeed(float ref){
  if (ref>100)
    ref = 100;
  else if (ref<0)
    ref = 0;
  analogWrite(DAC0, ref*9.43 + 80);
}

void HeaterCommand(){

  if (heater_onoff){
    lcd.setCursor(0, 0);
    lcd.print("aquecedor on ");
  }
  else{
    lcd.setCursor(0, 0);
    lcd.print("aqucedor off");
  }

  if (!digitalRead(but2)) flag_button2 = 0x01;
  if (digitalRead(but2) && flag_button2){
    flag_button2 = 0x00;
    heaterstatus++;
    if (heaterstatus > 2) heaterstatus = 0x01;

    //ligar ou desligar a bomba
    switch (heaterstatus){
    case 0x01:
      lcd.setCursor(0, 0);
      lcd.print("aquec on ");
      digitalWrite(led_heater, HIGH);
      digitalWrite(heater_rele, HIGH);
      heater_onoff = 1;
      break;
    case 0x02:
      lcd.setCursor(0, 0);
      lcd.print("aquec off");
      digitalWrite(led_heater, LOW);
      digitalWrite(heater_rele, LOW);
      heater_onoff = 0;
      break;
    }
  }
}

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

void VazaoAguaFria(){
  currentTime = millis();
  // Every second, calculate and print litres/hour
  if (currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
    vazao_fria = (flow_frequency / 7.5); // (Pulse frequency) / 7.5Q = flowrate in L/min
    flow_frequency = 0; // Reset Counter
  }
}

void VazaoAguaQuente(){

  float vazao1_sf; //descobrir o porqu� do nome da variavel

  microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  nivel = 11.46 - cmMsec;
  vazao1_sf = (0.0537)* pow((nivel * 10), 1.4727);
  if (vazao1_sf > 1){
    vazao_quente = 0.75*vazao_quente + 0.25*vazao1_sf;
  }
}

void runReads(){
  //Temperaturas();
  Temperaturas2();
  //VazaoAguaFria();
  //VazaoAguaQuente();
}

void CalcParams(){
  //calcula valores para LDR e LM35
  LDR_read = analogRead(LDR_PIN);
  LM35_read = analogRead(LM35_PIN);
  LM35_value = LM35_read * 0.48875855;
  LDR_value = LDR_read / 10;
}

void Temperaturas2(){
  CalcParams();
  temp[0] = LM35_value;
  temp[1] = LDR_value;

  send_info.data.temp1 = temp[0];
  send_info.data.temp2 = temp[1];
  send_info.data.pump_speed = pot_value_mapped;
  bitWrite(send_info.data.bstatus,0,pump_onoff);
  bitWrite(send_info.data.bstatus,1,heater_onoff);

}

//funções callback do i2c
void receiveEvent(int Nbytes){
  command = Wire.read();
  
  switch(command) {
    case 49: //comando de teste
      //comando liga bomba
      digitalWrite(inversor_rele, LOW); //o estado da bomba é invertido
      pump_onoff = 1;
      break;

    case 50:
      //comando desliga bomba
      digitalWrite(inversor_rele,HIGH);
      pump_onoff = 0;
      break;

    case 51:
      //comando liga aquecedor
      digitalWrite(heater_rele,HIGH);
      heater_onoff = 1;
      break;

    case 52:
      //comando desliga aquecedor
      digitalWrite(heater_rele,LOW);
      heater_onoff = 0;
      break;

    case 53:
      //comando alterar velocidade da bomba
      int i=0;
      while(Wire.available()){
        data[i] = Wire.read();
        i = i + 1;
      }
      parseSpeed(data);
  }
}

void requestEvent(){
  if(command==6){
    Wire.write(send_info.I2C_packet,sizeof(processData));
  }
}

void parseSpeed(byte data[]){
  PumpDataSpeed speed;
  speed.bspeed[0] = data[0];
  speed.bspeed[1] = data[1];
  speed.bspeed[2] = data[2];
  speed.bspeed[3] = data[3];
  Serial.println(speed.fspeed);
  //PumpSpeed(speed.fspeed);
}
