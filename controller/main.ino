#include <SPI.h>
#include "mcp2515_can.h"
#include <math.h>
#include <stdio.h>

const int SPI_CS_PIN1 = 9;
const int SPI_CS_PIN2 = 10;
const int SPI_CS_PIN3 = 8;
const int CAN_INIT_PIN = 2;
mcp2515_can CAN1(SPI_CS_PIN1);
mcp2515_can CAN2(SPI_CS_PIN2);
mcp2515_can CAN3(SPI_CS_PIN3);

int maxIterations = 40;

void turn_DCDC_on() {  //turn both DCDC on
  unsigned char buf_on[1] = { 1 };
  CAN1.sendMsgBuf(0x2E0, 0, 1, buf_on);
  CAN2.sendMsgBuf(0x2E0, 0, 1, buf_on);
  Serial.println("DCDC-Converters turned ON\n");
}
void turn_DCDC_off() {  // turn both DCDC-converter off
  unsigned char buf_off[1] = { 0 };
  CAN1.sendMsgBuf(0x2E0, 0, 1, buf_off);
  CAN2.sendMsgBuf(0x2E0, 0, 1, buf_off);
  Serial.println("DCDC-Converters turned OFF\n");
}
void set_input_current_DCDC(double Current) { //set input current
  float FactorCurrent = 100;
  unsigned int I_100 = round(Current / 2 * FactorCurrent);
  unsigned char buf[4] = { 0, 0, 0, 0 };
  if (I_100 <= 255) { 
    buf[0] = I_100 & 0xFF;
  }
  if (I_100 > 255 && I_100 <= 3000) {
    buf[0] = I_100 & 0xFF;
    buf[1] = (I_100 >> 8) & 0xFF;
  }
  CAN1.sendMsgBuf(0x2E9, 0, 4, buf);
  CAN2.sendMsgBuf(0x2E9, 0, 4, buf);
}
void read_aCar(int *receive, float *Voltage, float *Current, float *Power, float *SOC, float *Speed) { //read aCar values
  unsigned char len = 0;
  unsigned char buf[8];
  uint16_t iVoltage = 0;
  int16_t iCurrent = 0;
  float FactorVoltage = 0.01;  
  float FactorCurrent = 0.1;   
  bool received601 = false;
  bool received18B = false;
  bool allMessagesReceived = false;
  *receive = 0;
  unsigned long startTime;
  unsigned long elapsedTime;
  startTime = millis();
  while (allMessagesReceived == false) {
    elapsedTime = millis() - startTime;
    if (elapsedTime > 5000) {
      Serial.println("aCar CAN messages taking long time");
    }
    if (CAN_MSGAVAIL == CAN1.checkReceive()) {
      CAN1.readMsgBuf(&len, buf);
      unsigned long canID = CAN1.getCanId();
      if (canID == 0x601) {
        *receive = 1;
        iVoltage = (buf[7] << 8) | buf[6]; 
        *Voltage = iVoltage * FactorVoltage; 
        iCurrent = (buf[5] << 8) | buf[4];
        *Current = FactorCurrent * iCurrent;
        *Power = *Current * *Voltage;
        *SOC = buf[0];

        received601 = true;
      }
      if (canID == 0x18B) {
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
void read_MB(int *receive, float *Voltage, float *Current, float *Power, float *SOC, int *n_MB) {//read MB values
  unsigned char len = 0;
  unsigned char buf[8];
  uint16_t iVoltage_MB = 0;
  int16_t iCurrent_MB = 0;
  float FactorVoltage = 0.01;
  float FactorCurrent = 0.1;   

  bool voltageReceived = false;
  bool currentReceived = false;
  bool socReceived = false;
  bool received355 = false;
  bool received1001 = false;
  bool received1021 = false;
  bool n_MB_Received = false;
  bool allMessagesReceived = false;

  int iterations = 0;
  unsigned long startTime;
  unsigned long elapsedTime;
  startTime = millis();
  while (allMessagesReceived == false) {
    elapsedTime = millis() - startTime;
    if (elapsedTime > 5000) {
      Serial.println("MB CAN messages taking long time");
    }

    if (CAN_MSGAVAIL == CAN3.checkReceive()) {
      CAN3.readMsgBuf(&len, buf);
      unsigned long canID = CAN3.getCanId();

      if (canID == 0x356) {
        iVoltage_MB = (buf[1] << 8) | buf[0];
        *Voltage = iVoltage_MB * FactorVoltage;
        iCurrent_MB = (buf[3] << 8) | buf[2];
        *Current = FactorCurrent * iCurrent_MB;
        *Power = *Current * *Voltage;
        voltageReceived = true;
        currentReceived = true;
      }
      if (canID == 0x355) {
        *SOC = buf[0];
        socReceived = true;
        received355 = true;
      }
      if (canID == 0x1001) {
        received1001 = true;
      }
      if (canID == 0x1021) {
        received1021 = true;
      }
      if (received355 && !received1001 && !received1021) {
        *n_MB = 1;
        n_MB_Received = true;
      }
      if (received355 && received1001 && !received1021) {
        *n_MB = 2;
        n_MB_Received = true;
      }
      if (received355 && received1001 && received1021) {
        *n_MB = 3;
        n_MB_Received = true;
      }
      
      allMessagesReceived = voltageReceived && currentReceived && socReceived && n_MB_Received;
      if (allMessagesReceived == true) {
        *receive = 1;
      }
    } else {
      *receive = 0;
    }
    iterations++;
  }
}
void energy_managment_monotonous(float SOC_MB, float SOC_FP, int n_MB, float *Pdem_MB) {//calculate Power demand for the MB
  int E_MB_nom =  ;
  int E_FP_nom = 16500;
  int v_avg = 30;
  int b_avg = 270;
  float factorSOC = 0.01;

  float E_MB = n_MB * SOC_MB * factorSOC * E_MB_nom;
  float E_tot = SOC_FP * factorSOC * E_FP_nom + E_MB;
  float C_rate = (v_avg * b_avg) / E_tot;
  float Pdem_MB_check = E_MB * C_rate;

  if (SOC_FP >= 98) {
    *Pdem_MB = 0;
  } else {
    if (n_MB == 1 && Pdem_MB_check > 2600) {
      *Pdem_MB = 2600;
    } else if (n_MB == 2 && Pdem_MB_check > 5200) {
      *Pdem_MB = 5200;
    } else if (n_MB == 3 && Pdem_MB_check > 7800) {
      *Pdem_MB = 7800;
    } else {
      *Pdem_MB = Pdem_MB_check;
    }
  }
}

///Constants********************************************************************************************************************************************************************************************************************************
float SOC_FP = 0;
float SOC_MB = 0;

int Status_aCarONOFF = 0;
int aCar_receive = 0;
float aCar_Voltage = 0;
float aCar_Current = 0;
float aCar_Power = 0;
float aCar_Speed = 0;

float Pdem_MB = 0;

float U_DCDC = 0;
float I_DCDC = 0;

double P_actual = 0;
double P_target = 0;

int receive_MB = 0;
float Voltage_MB = 0;
float Current_MB = 0;
float Power_MB = 0;
int n_MB = 0;

double current_DCDC = 0;
int a = 0;
unsigned int t_count;

///Setup********************************************************************************************************************************************************************************************************************************

void setup() {
  delay(1000);
  Serial.begin(115200);
  while (CAN_OK != CAN1.begin(CAN_250KBPS)) {
    Serial.println("CAN 1 shield fail, retry.");
    delay(500);
  }
  Serial.println("CAN 1 shield okay");

  while (CAN_OK != CAN2.begin(CAN_250KBPS)) {
    Serial.println("CAN 2 shield fail, retry.");
    delay(500);
  }
  Serial.println("CAN 2 shield okay");

  while (CAN_OK != CAN3.begin(CAN_500KBPS)) {
    Serial.println("CAN 3 shield fail, retry.");
    delay(500);
  }
  Serial.println("CAN 3 shield okay");


  CAN1.init_Mask(0, 0, 0x7ff); 
  CAN1.init_Mask(1, 0, 0x7ff);  

  CAN1.init_Filt(0, 0, 0x601);  
  CAN1.init_Filt(1, 0, 0x18B);  
  CAN1.init_Filt(2, 0, 0x2F1);
  CAN1.init_Filt(3, 0, 0x2F3);
  CAN1.init_Filt(4, 0, 0x2F4);

  CAN2.init_Mask(0, 0, 0x7ff);
  CAN2.init_Mask(1, 0, 0x7ff);

  CAN2.init_Filt(0, 0, 0x2F1);
  CAN2.init_Filt(1, 0, 0x2F3);
  CAN2.init_Filt(2, 0, 0x2F4);

  CAN3.init_Mask(0, 0, 0xFFE);
  CAN3.init_Filt(0, 0, 0x355);
  CAN3.init_Filt(1, 0, 0x356);

  CAN3.init_Mask(1, 1, 0xFFC0);
  CAN3.init_Filt(2, 1, 0x1001);
}

///Main Loop********************************************************************************************************************************************************************************************************************************

void loop() {
  read_aCar(&aCar_receive, &aCar_Voltage, &aCar_Current, &aCar_Power, &SOC_FP, &aCar_Speed);
  read_MB(&receive_MB, &Voltage_MB, &Current_MB, &Power_MB, &SOC_MB, &n_MB);
  if (aCar_Speed > 0){                                                                               
  //if (a == 0) {
    t_count = 0;
    turn_DCDC_on();
    while(aCar_Speed > 0){
    //while (a == 0) {

      read_aCar(&aCar_receive, &aCar_Voltage, &aCar_Current, &aCar_Power, &SOC_FP, &aCar_Speed);
      read_MB(&receive_MB, &Voltage_MB, &Current_MB, &Power_MB, &SOC_MB, &n_MB);

      energy_managment_monotonous(SOC_MB, SOC_FP, n_MB, &Pdem_MB);

      P_target = Pdem_MB;

      current_DCDC = (Pdem_MB / Voltage_MB);
      if (current_DCDC > 80) {
        current_DCDC = 80;
      }
      set_input_current_DCDC(current_DCDC);



      //Serial.print("P_actual: ");
      //Serial.println(P_actual);
      //Serial.print(",");
      Serial.print("P_target: ");
      Serial.println(P_target);
      //Serial.print(",");
      //Serial.print("aCar_power: ");
      //Serial.println(aCar_Power);
      Serial.print("MB Power Output: ");
      Serial.println(Power_MB);
      Serial.print("MB current: ");
      Serial.println(Current_MB);
      Serial.print("current soll: ");
      Serial.println(current_DCDC);
      Serial.println("-----------------");
      delay(500);
    }

  } else {
    while (t_count < 3) {
      turn_DCDC_off();
      t_count++;
    }
    delay(100);
  }
}
