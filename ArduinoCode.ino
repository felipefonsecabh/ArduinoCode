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

//lib Modbus
#include <Modbusino.h>

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
#define TEMPERATURE_PRECISION 10


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

//estrutura de dados para troca de informações em modbus
typedef struct processData{
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float hotflow;
  float coldflow;
  float pump_speed;
  uint16_t bstatus;
  uint16_t command;
  float speedcommand;
  uint16_t chksum;
};

int command; //processar o indice de comando enviado pelo Rpi

//inicialização do slave modbus
ModbusinoSlave modbusino_slave(1);

//Alocação de registros para trocar dados; 19 registros são necessários incluindo os comandos

typedef union modbus_read{
  processData data;
  uint16_t tab_reg[19];
};

modbus_read modbusinfo;

//status do último comando
int lastcommand;

//ultima velocidade
float lastspeed;

//variáveis utilizadas no calculo de vazao de agua fria/quente
unsigned long currentTime;
unsigned long cloopTime;
long microsec;
float cmMsec;
float nivel;
//float vazao1_sf;   //se for necessário para a função de vazao quente, descomentar

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

//funções do estado do arduino
void LocalState();
void RemoteState();
void emergencia();

//funções para fazerem a leitura periodica dos valores analógicos
void runReads();
void refresh_modbus_packet();

//função para interpretação de comandos
void detectChanges();
void runCommands(int command);

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
  for (byte i = 0; i <= 4; i++){
    sensors.setResolution(deviceID[i], TEMPERATURE_PRECISION);
  }
  
  //iniciar lcd
  lcd.begin(20,4);  //no original utilizar 20,4

	//resolução de escrita do DAC
	analogWriteResolution(10);

  //variáveis que guarda o status da bomba e do aquecedor
	pump_onoff = 0;
  heater_onoff = 0;
    
  //inicialização da serial
  Serial.begin(115200);

  //inicialização da estrutura modbus
  lastspeed = 0;
  lastcommand = 0;
  modbusino_slave.setup(115200);
  modbusinfo.data.chksum = 27;

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
	if (digitalRead(emergency_button) == LOW){ //presisonado botão
		emergencia();
	}
	else if (!digitalRead(emergency_button) == LOW && flag_emergency){ //despressionado o botão
		flag_emergency = 0x00;
    emergency_status = 0;
		lcd.clear();
		if(switch_state){
            RemoteState();
        }
        else{
            LocalState();
        }
	}
	else{ //tudo normal
		if(switch_state){
            RemoteState();
        }
        else{
            LocalState();
        }
	}
}

//função de interrupção para calcular a vazão
void flow(){
  flow_frequency++;
}

//funções que gerenciam o estado do arduino

void LocalState(){

  //efetua a leitura das grandezas
  runReads();

    //atualiza o lcd com os novos valores
	Menu(0);

	if (!digitalRead(but1)) flag_button1 = 0x01;
	if (!digitalRead(but2)) flag_button2 = 0x01;

	if (digitalRead(but1) && flag_button1){
		flag_button1 = 0x00;
		//troca o estado da bomba
		digitalWrite(inversor_rele, pump_onoff);  //o estado da bomba � invertido
		//digitalWrite(led_pump, !pump_onoff);  -> N�o � para utilizar o led
		pump_onoff = !pump_onoff;
	}

	if (digitalRead(but2) && flag_button2){
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

  //atualiza as informações
  refresh_modbus_packet();
}

void RemoteState(){
  //faz a leitura das variáveis
  runReads();

  //atualiza o lcd com os novos valores
  Menu(1);

  //atualiza a estrutura de envio de dados via i2c
  refresh_modbus_packet();

  //detecta modificações nas variáveis
  detectChanges();
}

void emergencia(){
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
	if (!flag_emergency){
		lcd.clear();
		EmergencyStatus();
	}
	flag_emergency = 0x01;
  refresh_modbus_packet();
}

//funções para gerenciar a exibição do lcd

void EmergencyStatus(){
	lcd.setCursor(0, 0);
	lcd.print("Emergencia");
	lcd.setCursor(0, 1);
  lcd.print("Comandos");
  lcd.setCursor(0,2);
  lcd.print("Bloqueados");
}

void Menu(int op){

  mode = (op==1) ? "Remote Mode" : "Local  Mode";

  lcd.setCursor(0,0);
  lcd.print(mode);
    
  lcd.setCursor(0,1);
  lcd.print("TA: ");
  lcd.print(temp[0]);
  lcd.print("  ");
  lcd.print("TB: ");
  lcd.print(temp[1]);

  lcd.setCursor(0,2);
  lcd.print("TC: ");
  lcd.print(temp[2]);
  lcd.print("  ");
  lcd.print("TD: ");
  lcd.print(temp[3]);

  lcd.setCursor(0,3);
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

void VazaoAguaFria(){
  currentTime = millis();
  // Every second, calculate litres/hour
  if (currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
    vazao_fria = (flow_frequency / 7.5); // (Pulse frequency) / 7.5Q = flowrate in L/min
    flow_frequency = 0; // Reset Counter
  }
}

void VazaoAguaQuente(){

  float vazao1_sf; //descobrir o porquê do nome da variavel

  microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  nivel = 11.46 - cmMsec;
  vazao1_sf = (0.0537)* pow((nivel * 10), 1.4727);
  if (vazao1_sf > 1){
    vazao_quente = 0.75*vazao_quente + 0.25*vazao1_sf;
  }
}

void Temperaturas2(){
  LM35_value = analogRead(LM35_PIN) *0.48875855;
  LDR_value = analogRead(LDR_PIN) / 10.0;
  temp[0] = LM35_value;
  temp[1]= LDR_value;
}

void Simulator(){
  current_sim_time = millis();
  if(current_sim_time >= simulator_time_elapsed + SIMULATORSAMPLETIME){
    
    simulator_time_elapsed = current_sim_time;
    
    for(int i=0; i<3; ++i){
      temp[i] = mapfloat(random(1024),0,1023,10,50);
    }
    
    //ultima temperatura com valor senoidal
    temp[3] = mapfloat(waveformsTable[0][indexwave],0,4095,0,100);
    indexwave++;
    if(indexwave == maxSamplesNum){
      indexwave = 0;// Reset the counter to repeat the wave
    }
      
    vazao_quente = mapfloat(random(1024),0,1023,0,30);
    vazao_fria = mapfloat(random(1024),0,1023,0,30);
  }
}

void PumpSpeed(float ref){
	if (ref>100)
		ref = 100;
	else if (ref<0)
		ref = 0;
	analogWrite(DAC0, ref*9.43 + 80);
}

void ReadPotentiometer(){
  pot_value = analogRead(pot);
  pot_value_mapped = mapfloat(pot_value, 0, 1023, 0, 100);
}

void runReads(){
  /*
  Temperaturas();
  VazaoAguaFria();
  VazaoAguaQuente();
  */
  //Temperaturas2();
  Simulator();
}

void detectChanges(){
  if(lastcommand != modbusinfo.data.command){
    lastcommand = modbusinfo.data.command;
    runCommands(modbusinfo.data.command);
  }
}

//funções callback do i2c
void runCommands(int command){
    
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
        digitalWrite(led_heater,HIGH);
        heater_onoff = 1;
        break;
  
      case 52:
        //comando desliga aquecedor
        digitalWrite(heater_rele,LOW);
        digitalWrite(led_heater,LOW);
        heater_onoff = 0;
        break;
  
      case 53:
        //comando alterar velocidade da bomba
        remote_pumpspeed = modbusinfo.data.speedcommand;
        PumpSpeed(remote_pumpspeed);
        break;
    }

    //zera comando no pacote
    modbusinfo.data.command = 0;
}


void refresh_modbus_packet(){
  modbusinfo.data.temp1 = temp[0];
  modbusinfo.data.temp2 = temp[1];
  modbusinfo.data.temp3 = temp[2];
  modbusinfo.data.temp4 = temp[3];
  
  //se a bomba estiver desligada, ignorar o valor do potenciometro
  if(pump_onoff){
    if(switch_state){
      modbusinfo.data.pump_speed = remote_pumpspeed;
    }
    else{
      modbusinfo.data.pump_speed = pot_value_mapped;
    }
  }
  else{
    modbusinfo.data.pump_speed = 0.0;
  }
    
    
  modbusinfo.data.hotflow = vazao_quente;
  modbusinfo.data.coldflow = vazao_fria;
  bitWrite(modbusinfo.data.bstatus,0,pump_onoff);
  bitWrite(modbusinfo.data.bstatus,1,heater_onoff);
  bitWrite(modbusinfo.data.bstatus,2,switch_state);
  bitWrite(modbusinfo.data.bstatus,3,emergency_status);

  //teste
  modbusinfo.data.chksum = 27;

  //loop_modbus
  modbusino_slave.loop(modbusinfo.tab_reg, 19);
}
  
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
