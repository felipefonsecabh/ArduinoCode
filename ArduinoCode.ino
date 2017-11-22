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

//lib i2c
#include <Wire.h>

//gerar sinais de simulação caso necessário
#include <waveforms.h>

//Pinos de entrada e saída

#define but1 A1  
#define but2 A2  
#define pot A0 

#define mode_switch 42  
#define emergency_button 43 

#define ultrassonico_echo	50      
#define ultrassonico_trigger 48     
#define ONE_WIRE_BUS 52             
#define hf_sensor 9                
#define TEMPERATURE_PRECISION 12

#define led1 44                  
#define led2 45                  
#define led_heater 47            

#define inversor_rele 49        
#define heater_rele 10          

#define MAX_MENU_ITENS 2         

//testes em prototipo
#define oneHzSample 1000000/maxSamplesNum //gerador de sinais de simulação
#define SIMULATORSAMPLETIME 500


//variáveis internas

//armazenar valores de entrada
bool switch_state;
int pot_value;
bool emergency_status;

volatile int pump_onoff;
volatile int heater_onoff;
volatile float remote_pumpspeed;

//vetor para armazenar a leitura de temperaturas
float temp[4];

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
  char serial_packet[sizeof(processData)];
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
float raw_coldflow_value;
double vazao_quente;
float vazao_fria;
volatile int flow_frequency;

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

//estatísticas
unsigned long temp_errors;  //número de erros de envio da leitura de temperatura

//uma contagem de tempo inicial
unsigned long start_time;

//buffers para medição da filtragem
float temp_buffer[4];

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

//funções do estado do arduino
void LocalState();
void RemoteState();
void emergencia();

//funções para fazerem a leitura periodica dos valores analógicos
void runReads();
void refresh_I2C_Packet();

//função para receber o valor em bytes via i2c e retornar a velocidade em float
void parseSpeed(byte data[]);

//função de filtragem
void Remove_TempSpike(float *temp_buffer, float temps[]);

//função para simulação de valores
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
  start_time = millis();

  //inicialização da estrutura i2c
  send_info.data.chksum = 27;
  Wire.begin(12); //arduino iniciado no endereço 12
  Wire.onReceive(receiveEvent); //callback para recebimento de comandos
  Wire.onRequest(requestEvent); //callback para responder à requisições

  //semente para o simulador
  randomSeed(2);
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

  //variáveis auxiliares
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
  lcd.print(temp_buffer[0]);
  lcd.print("  ");
  lcd.print("TB: ");
  lcd.print(temp_buffer[1]);

  lcd.setCursor(0, 2);
  lcd.print("TC: ");
  lcd.print(temp_buffer[2]);
  lcd.print("  ");
  lcd.print("TD: ");
  lcd.print(temp_buffer[3]);

  lcd.setCursor(0, 3);
  lcd.print("V1: ");
  lcd.print(vazao_quente);
  lcd.print("  ");
  lcd.print("V2: ");
  lcd.print(vazao_fria);

}

//funções de leitura (e escrita) das grandezas
void Temperaturas(){
  if(millis() - lastTempRequest > delayTempRead){ //delayTempRead
    lastTempRequest = millis();
    for (byte i = 0; i <= 4; i++)
    {
      temp[i] = sensors.getTempC(deviceID[i]); //captura as temperaturas do sensor

    }
  
    sensors.requestTemperatures();  //faz nova requisição por temperatura 

    if(millis() - start_time < 5000){  //insere um tempo de acomodação para começar a remover os spikes
      *temp_buffer = *temp;
    }
    else{
      Remove_TempSpike(temp_buffer,temp); //lógica de remoção de spike
    }
  } 
}

void VazaoAguaFria() {
  currentTime = millis();
  // Every second, calculate litres/hour
  if (currentTime >= (cloopTime + 1000)) //função só calcula a vazão a cada segundo
  {
    cloopTime = currentTime; // atualiza tempo para próxima execução
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min
    noInterrupts();
    vazao_fria = (flow_frequency / 7.5); // (Pulse frequency) / 7.5Q = flowrate in L/min
    flow_frequency = 0; // Reset Counter
    interrupts();
  }
}

void VazaoAguaQuente() {

  hotflow_time = millis(); 
  if(hotflow_time - last_hotflow_time > hotflow_interval){  //função só calcula a vazão em um intervalo de tempo e não a todo tempo
    last_hotflow_time = hotflow_time; // atualiza tempo para próxima execução
    microsec = ultrasonic.timing();
    cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
    nivel = 11.46 - cmMsec;
    raw_coldflow_value = (0.0537) * pow((nivel * 10), 1.4727);
    if (raw_coldflow_value > 1) {
      vazao_quente = 0.95 * vazao_quente + 0.05 * raw_coldflow_value;
    }
  }
}

void Simulator() {
  current_sim_time = millis();
  if (current_sim_time >= simulator_time_elapsed + SIMULATORSAMPLETIME) {

    simulator_time_elapsed = current_sim_time;

    for (int i = 0; i < 3; ++i) {
      temp[i] = mapfloat(random(1024), 0, 1023, 10, 50);
    }

    //ultima temperatura com valor senoidal
    temp[3] = mapfloat(waveformsTable[0][indexwave], 0, 4095, 0, 50);
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
  
    VazaoAguaFria();
    VazaoAguaQuente();
    Temperaturas();
  
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
  remote_pumpspeed = speed.fspeed;
  PumpSpeed(remote_pumpspeed); //envia o comando de velocidade para a bomba fisicamente
}

void refresh_I2C_Packet() {
  send_info.data.temp1 = temp_buffer[0];  
  send_info.data.temp2 = temp_buffer[1];  
  send_info.data.temp3 = temp_buffer[2];  
  send_info.data.temp4 = temp_buffer[3]; 

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

void Remove_TempSpike(float *temp_buffer, float temps[]){
 
  for(int i=0;i<4;i++){
    if(temps[i]==- 127){ //valor ruim, deve ser mantido o valor anterior
      temp_errors++;
    }
    else{ 
      temp_buffer[i] = temps[i]; //pega o valor atual
    }
  } 
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
