#include <SPI.h>
#include "mcp2515_can.h"
#include <math.h>
#include <stdio.h>

const int SPI_CS_PIN1 = 9;
const int SPI_CS_PIN2 = 10;  //SPI_CS_PIN und SPI_CS_PIN1 definieren die Chip-Select-Pins, die für die Kommunikation mit den beiden CAN-Modulen verwendet werden
const int SPI_CS_PIN3 = 8;
const int CAN_INIT_PIN = 2;  // gibt Pin an, der für die Initialisierung des CAN-Moduls verwedet wird
mcp2515_can CAN1(SPI_CS_PIN1);
mcp2515_can CAN2(SPI_CS_PIN2);
mcp2515_can CAN3(SPI_CS_PIN3);

int maxIterations = 200;

void read_aCar(int *receive, float *Voltage, float *Current, float *Power, float *SOC, float *Speed) {
  unsigned char len = 0;
  unsigned char buf[8];
  uint16_t iVoltage = 0;
  int16_t iCurrent = 0;
  float FactorVoltage = 0.01;  // Offset Voltage
  float FactorCurrent = 0.1;   // Offset Current
  bool received601 = false;
  bool received18B = false;
  bool allMessagesReceived = false;
  *receive = 0;
  while (allMessagesReceived == false) {
    if (CAN_MSGAVAIL == CAN1.checkReceive()) {
      CAN1.readMsgBuf(&len, buf); 
      unsigned long canID = CAN1.getCanId();
      if (canID == 0x601) {
        *receive = 1;
        iVoltage = (buf[7] << 8) | buf[6];    // ivoltage = 16 bit int -> die logik setzt buf[7] auf die vorderen 8 bit und buf[6] auf die hinteren 8 Bit
        *Voltage = iVoltage * FactorVoltage;  //Factor offset voltage
        iCurrent = (buf[5] << 8) | buf[4];
        *Current = FactorCurrent * iCurrent;
        *Power = *Current * *Voltage;
        *SOC = buf[0];

        received601 = true;
      }

      if (canID == 0x18B){
        *Speed = buf[5];

        received18B = true;
      }

      allMessagesReceived = received601 && received18B;
      if (allMessagesReceived == true) {
        *receive = 1;
      }
    } else {
      *receive = 0;
    }
  }
}



///Constants********************************************************************************************************************************************************************************************************************************
float SOC_FP = 0;
float SOC_MB = 0;

double R_I = 0;
double U_OCV = 0;

int receive_aCarONOFF = 0;
int receive_MB_ONOFF = 0;
int Status_aCarONOFF = 0;
int aCar_receive = 0;
float aCar_Voltage = 0;
float aCar_Current = 0;
float aCar_Power = 0;
float aCar_Speed = 0;

float Pdem_MB = 0;
float Pdem_FP = 0;

float U_DCDC = 0;
float I_DCDC = 0;

float P_ist = 0;
float P_soll = 0;
float P_Last = 0;
int t_count = 0;

float e = 0;
float eMem = 0;
float yp = 0.0;
float dyi = 0.0;
float yi = 0.0;
float y = 0.0;
const float Kp = 0.0001;
const float Ki = -0.0001;

float radicand = 0;
float pre_control_output = 0;

const float dt = 100;  // in ms
float t = 0.0;
float tMem = t;

int receive_MB = 0;
float Voltage_MB = 0;
float Current_MB = 0;
float Power_MB = 0;
int n_MB = 0;

float act_Voltage_1 = 0;
float act_Voltage_2 = 0;

float set_Voltage = 0;

///Setup********************************************************************************************************************************************************************************************************************************

void setup() {
  delay(3000);

  Serial.begin(115200);                      //initialisiert die serielle Kommunikation mit einer Baudrate von 115200 -> Geschwindigkeit der Datenübertragung zwischen dem Arduino und dem Computer
  while (CAN_OK != CAN1.begin(CAN_250KBPS))  // Initialisierung des CAN-Controllers, stellt sicher das Initialiserung erfolgreich isst
  {
    Serial.println("CAN 1 shield fail, retry.");
    delay(500);
  }
  Serial.println("CAN 1 shield okay");

  while (CAN_OK != CAN2.begin(CAN_250KBPS)) {
    Serial.println("CAN 2 shield fail, retry.");
    delay(500);
  }
  Serial.println("CAN 2 shield okay");

  while (CAN_OK != CAN3.begin(CAN_500KBPS)) {  // 500 für better Pack
    Serial.println("CAN 3 shield fail, retry.");
    delay(500);
  }
  Serial.println("CAN 3 shield okay");

  // initialisieren von Filter und Maken für die CAN-Bus-Kommunikation, damit nur bestimmte IDs empfangen werden
  CAN1.init_Mask(0, 0, 0x7ff);  //Rx Mask 1 (erste Zahl: 0 oder 1 beschreibt welcher masken-register initialisiert wird, zweit Zahl: 0 steht fpr exteded ID, dreite Zahl: gibt Maske an-> welche Bits des CAN-IDs als Filter verwendet werden) -  CAN-ID AND Maske muss übereinstimmen damit Nachricht empfangen wird
  //CAN1.init_Mask(1, 0, 0x7ff);  //Rx Mask 2 // funktioniert mit 3ff und mit 7ff

  CAN1.init_Filt(0, 0, 0x601);  //Filter for message with the ID 0x601
  CAN1.init_Filt(1, 0, 0x18B);  //For additional filter, add CAN.init_Filt(X, 0, 0xID), where X describes # of filter (up to 6 filters)

  //CAN1.init_Filt(2, 0, 0x2F1);
  //CAN1.init_Filt(3, 0, 0x2F3);
  //CAN1.init_Filt(4, 0, 0x2F4);
}

///Main Loop********************************************************************************************************************************************************************************************************************************

void loop() {
  int a = 0;
  if (a==0){
    read_aCar(&aCar_receive, &aCar_Voltage, &aCar_Current, &aCar_Power, &SOC_FP, &aCar_Speed);
    Serial.print("Voltage: ");
    Serial.println(aCar_Voltage);
    Serial.print("Current: ");
    Serial.println(aCar_Current);
    Serial.print("Power: ");
    Serial.println(aCar_Power);
    Serial.print("SOC: ");
    Serial.println(SOC_FP);
    Serial.print("Speed: ");
    Serial.println(aCar_Speed);
    Serial.println("--------------");
    delay(1000);


    }
    t_count++;
  }


