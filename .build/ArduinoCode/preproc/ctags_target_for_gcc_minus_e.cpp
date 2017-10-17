# 1 "e:\\ArduinoCode\\ArduinoCode.ino"
# 1 "e:\\ArduinoCode\\ArduinoCode.ino"
/*
 Name:		SimpleClass_Applcation.ino
 Created:	5/26/2017 4:18:28 PM
 Author:	Felipe Fonseca
 Description: Versão de Código para o Trocador de Calor de forma a simplificar
 a operçãoo para tornar possível a utilização em aulas
*/

//includes

//Libs dos sensores
# 13 "e:\\ArduinoCode\\ArduinoCode.ino" 2
# 14 "e:\\ArduinoCode\\ArduinoCode.ino" 2
# 15 "e:\\ArduinoCode\\ArduinoCode.ino" 2

//display lcd
# 18 "e:\\ArduinoCode\\ArduinoCode.ino" 2

//timed action
//#include <Utility.h>
//#include <TimedAction.h>

//lib i2c
# 25 "e:\\ArduinoCode\\ArduinoCode.ino" 2

//Pinos de entrada e sa�da
# 51 "e:\\ArduinoCode\\ArduinoCode.ino"
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


//estrutura de dados para envio i2c
typedef struct processData{
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

typedef union I2C_Send{ //compartilha a mesma área de memória
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

//variáveis auxiliares para navegação
char menu = 0x01;
char flag_button1 = 0x00;
char flag_button2 = 0x00;
char flag_emergency = 0x00;
String mode = "";

//variáveis auxiliares para comando
char pumpstatus;
float pot_value_mapped;
char heaterstatus;

//inicialização de objetos
OneWire oneWire(52 /*no original trocar para 52  - terminal do conjunto de sensores*/);
DallasTemperature sensors(&oneWire);
DeviceAddress deviceID[] =
{
  { 0x28, 0xFF, 0x46, 0x02, 0x54, 0x16, 0x04, 0xF8 },
  { 0x28, 0xFF, 0xD3, 0xE6, 0x53, 0x16, 0x04, 0xA0 },
  { 0x28, 0xFF, 0xE8, 0xF0, 0x53, 0x16, 0x04, 0x65 },
  { 0x28, 0x1D, 0x9D, 0x27, 0x00, 0x00, 0x80, 0x2E }
};

Ultrasonic ultrasonic(48 /*no original trocar para 48*/, 50 /*no original trocar para 50*/);

LiquidCrystal lcd(41, 11, 12, 40, 13, 38); //no original deve-se utilizar  LiquidCrystal lcd(41, 11, 12, 40, 13, 38);

//funções
void ReadPotentiometer();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

//funções de navegação  - Display LCD
void Menu(int op);
void EmergencyStatus();

//funções de leitura de grandezes
void flow(); //função de interrupção para o calculo de vazao de água fria
void PumpSpeed(float ref); //função para alterar a velocidade da bomba
void VazaoAguaFria(); //função para calcular a vazão de água fria
void VazaoAguaQuente(); //função para calcular a vazão de água quente
void Temperaturas(); //função para calcular as temperaturas dos sensores

//funções do estado do arduino
void LocalState();
void RemoteState();
void emergencia();

//funções para fazerem a leitura periodica dos valores analógicos
void runReads();
void refresh_I2C_Packet();

//função para receber o valor em bytes via i2c e retornar a velocidade em float
void parseSpeed(byte data[]);

// the setup function runs once when you press reset or power the board
void setup() {
 //painel
 pinMode(A1 /*no programa de testes 42; no original, trocar por A1  /|*/, 0x2);
 pinMode(A2 /*no programa de testes 44; no original trocar por A2*/, 0x2);
 pinMode(A0 /*no programa de testes A7; no original trocar por A0*/, 0x0);
 pinMode(42 /*no programa de testes 43; no original trocar por 42*/, 0x0);
 pinMode(43 /*no programa de testes 51; no original  trocar por 43*/, 0x0);
 pinMode(45 /* no original trocar para 45*/, 0x1);
 pinMode(44 /* no programa de testes 35; no original trocar para 44*/, 0x1);
 pinMode(47 /* no programa de testes 37; no original trocar para 47*/, 0x1);

 //sensores
 pinMode(50 /*no original trocar para 50*/, 0x0);
 pinMode(48 /*no original trocar para 48*/, 0x1);
 pinMode(9 /*no original trocar para 9 -> pino de interrup��o para calculo da vaz�o agua fria*/, 0x2);
 pinMode(52 /*no original trocar para 52  - terminal do conjunto de sensores*/, 0x2);

 //reles
 pinMode(10 /*no programa de testes 49;no original trocar para 10*/, 0x1);
 pinMode(49 /*no programa de teste 47;no original trocar para 49*/, 0x1);

 //começar inversor desligado
  digitalWrite(49 /*no programa de teste 47;no original trocar para 49*/, 0x1);

  //habilitar a interrupção para medição de vazão fria
  attachInterrupt(9 /*no original trocar para 9 -> pino de interrup��o para calculo da vaz�o agua fria*/, flow, 3);

  //iniciar sensores de temperatura
  sensors.begin();
  for (byte i = 0; i <= 4; i++){
    sensors.setResolution(deviceID[i], 10);
  }

  //iniciar lcd
  lcd.begin(20,4); //no original utilizar 20,4

 //resolução de escrita
 analogWriteResolution(10);

 //teste
  //set_rnd_values();
  pumpstatus = 0x00;
  heaterstatus = 0x00;
 pump_onoff = 0;
  heater_onoff = 0;

  //inicialização da serial
  Serial.begin(115200);

  //inicialização da estrutura i2c
  Wire.begin(12); //arduino iniciado no endereço 12
  Wire.onReceive(receiveEvent); //callback para recebimento de comandos
  Wire.onRequest(requestEvent); //callback para responder à requisições
}


// the loop function runs over and over again until power down or reset
void loop() {

  // le o estado da chave
  switch_state = digitalRead(42 /*no programa de testes 43; no original trocar por 42*/);

  /* lê se o botão de emergencia está acionado ou não
      cobre os casos de pressionar o botão e despressionar o botão
  */
 if (digitalRead(43 /*no programa de testes 51; no original  trocar por 43*/) == 0x0){ //presisonado botão
  emergencia();
 }
 else if (!digitalRead(43 /*no programa de testes 51; no original  trocar por 43*/) == 0x0 && flag_emergency){ //despressionado o botão
  flag_emergency = 0x00;
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

 if (!digitalRead(A1 /*no programa de testes 42; no original, trocar por A1  /|*/)) flag_button1 = 0x01;
 if (!digitalRead(A2 /*no programa de testes 44; no original trocar por A2*/)) flag_button2 = 0x01;

 if (digitalRead(A1 /*no programa de testes 42; no original, trocar por A1  /|*/) && flag_button1){
  flag_button1 = 0x00;
  //troca o estado da bomba
  digitalWrite(49 /*no programa de teste 47;no original trocar para 49*/, pump_onoff); //o estado da bomba � invertido
  //digitalWrite(led_pump, !pump_onoff);  -> N�o � para utilizar o led
  pump_onoff = !pump_onoff;
 }

 if (digitalRead(A2 /*no programa de testes 44; no original trocar por A2*/) && flag_button2){
  flag_button2 = 0x00;
  //troca o estado do aquecedor
  digitalWrite(10 /*no programa de testes 49;no original trocar para 10*/, !heater_onoff);
  digitalWrite(47 /* no programa de testes 37; no original trocar para 47*/, !heater_onoff);
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

void RemoteState(){
  //faz a leitura das variáveis
  runReads();

  //atualiza o lcd com os novos valores
  Menu(1);

  //atualiza a estrutura de envio de dados via i2c
  refresh_I2C_Packet();
}

void emergencia(){
 //digitalWrite(led_flow_mode, LOW);//Apaga LED 1
 digitalWrite(47 /* no programa de testes 37; no original trocar para 47*/, 0x0);//Apaga LED 2
 //digitalWrite(led_pump, LOW);//Apaga LED 4
 digitalWrite(49 /*no programa de teste 47;no original trocar para 49*/, 0x1);//Desliga a bomba
 digitalWrite(10 /*no programa de testes 49;no original trocar para 10*/, 0x0);//Desliga o aquecedor

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

  mode = (op==1) ? "Remote Mode" : "Local Mode";

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

void PumpSpeed(float ref){
 if (ref>100)
  ref = 100;
 else if (ref<0)
  ref = 0;
 analogWrite(DAC0, ref*9.43 + 80);
}

void ReadPotentiometer(){
  pot_value = analogRead(A0 /*no programa de testes A7; no original trocar por A0*/);
  pot_value_mapped = mapfloat(pot_value, 0, 1023, 0, 100);
}

void runReads(){
  Temperaturas();
  VazaoAguaFria();
  VazaoAguaQuente();
}


//funções callback do i2c
void receiveEvent(int Nbytes){
    command = Wire.read();

    switch(command) {
      case 49: //comando de teste
        //comando liga bomba
        digitalWrite(49 /*no programa de teste 47;no original trocar para 49*/, 0x0); //o estado da bomba é invertido
        pump_onoff = 1;
        break;

      case 50:
        //comando desliga bomba
        digitalWrite(49 /*no programa de teste 47;no original trocar para 49*/,0x1);
        pump_onoff = 0;
        break;

      case 51:
        //comando liga aquecedor
        digitalWrite(10 /*no programa de testes 49;no original trocar para 10*/,0x1);
        digitalWrite(47 /* no programa de testes 37; no original trocar para 47*/,0x1);
        heater_onoff = 1;
        break;

      case 52:
        //comando desliga aquecedor
        digitalWrite(10 /*no programa de testes 49;no original trocar para 10*/,0x0);
        digitalWrite(47 /* no programa de testes 37; no original trocar para 47*/,0x0);
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
        break;
    }
}

void requestEvent(){
  if(command==6){
    Wire.write(send_info.I2C_packet,sizeof(processData));
  }
}

void parseSpeed(byte data[]){
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

void refresh_I2C_Packet(){
  send_info.data.temp1 = temp[0];
  send_info.data.temp2 = temp[1];
  send_info.data.temp3 = temp[2];
  send_info.data.temp4 = temp[3];

  //se a bomba estiver desligada, ignorar o valor do potenciometro
  if(pump_onoff){
    if(switch_state){
      send_info.data.pump_speed = remote_pumpspeed;
    }
    else{
      send_info.data.pump_speed = pot_value_mapped;
    }
  }
  else{
    send_info.data.pump_speed = 0.0;
  }


  send_info.data.hotflow = vazao_quente;
  send_info.data.coldflow = vazao_fria;
  (pump_onoff ? ((send_info.data.bstatus) |= (1UL << (0))) : ((send_info.data.bstatus) &= ~(1UL << (0))));
  (heater_onoff ? ((send_info.data.bstatus) |= (1UL << (1))) : ((send_info.data.bstatus) &= ~(1UL << (1))));
  (switch_state ? ((send_info.data.bstatus) |= (1UL << (2))) : ((send_info.data.bstatus) &= ~(1UL << (2))));
  (emergency_status ? ((send_info.data.bstatus) |= (1UL << (3))) : ((send_info.data.bstatus) &= ~(1UL << (3))));
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
