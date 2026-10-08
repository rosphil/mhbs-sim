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
int maxIterations = 15;

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
void aCar_ON(int *receive) {  //checkt IGNSTATE des aCars, IGNSTATE=0, falls der motor aus ist
  unsigned char len = 0;                   //byte
  unsigned char buf[8];
  int receiveAttempt = 0;
  *receive = 0;

  while (receiveAttempt == 0) {
    if (CAN_MSGAVAIL == CAN1.checkReceive()) {
      CAN1.readMsgBuf(&len, buf);
      unsigned long canID = CAN1.getCanId();
      if (canID == 0x18B) {
        *receive = 1;
        receiveAttempt = 1;  // damit er aus der while schleife geht
        Serial.println("aCar is on");
      }
    } else {              // werden keine Nachrichten empfangen, ist das aCar ausgeschaltet und recieve wird auf 0 gesetzt und die Schleife beendet
        Serial.println("aCar is NOT on");
        delay(100);
    }
  }
}
void MB_ON(int *receive){
  unsigned char len = 0;
  unsigned char buf[8];
  int receiveAttempt = 0;
  while (receiveAttempt == 0 ) {
    if (CAN_MSGAVAIL == CAN3.checkReceive()) {
      CAN3.readMsgBuf(&len, buf);
      unsigned long canID = CAN3.getCanId();
      Serial.println("hello");
      if (canID == 0x335 || canID == 0x1001) {
        *receive = 1;
        receiveAttempt = 1;
        Serial.println("Modular Battery is on");
      }
    }else {
        Serial.println("Modular Battery NOT on");
        delay(1000);
    }
  }
}
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
void read_MB(int *receive, float *Voltage, float *Current, float *Power, float *SOC, int *n_MB) {
  unsigned char len = 0;
  unsigned char buf[8];
  uint16_t iVoltage_MB = 0;
  int16_t iCurrent_MB = 0;
  float FactorVoltage = 0.01;  // Offset Voltage
  float FactorCurrent = 0.1;   // Offset Current

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
    if (elapsedTime > 2000){
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
      }
      if (canID == 0x355) {
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

      // Check if all messages have been received
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
void energy_managment_monotonous(float SOC_MB, float SOC_FP, int n_MB, float *Pdem_MB) {
  int E_MB_nom = 2346;
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

int receive_aCarONOFF = 0;
int receive_MB_ONOFF = 0;
int Status_aCarONOFF = 0;
int aCar_receive = 0;
float aCar_Voltage = 0;
float aCar_Current = 0;
float aCar_Power = 0;

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

float sqr_root = 0;
float pre_control_output = 0;

const float dt = 100;  // in ms
float t = 0.0;
float tMem = t;

int receive_MB = 0;
float Voltage_MB = 0;
float Current_MB = 0;
float Power_MB = 0;
int n_MB = 0;

float speed =0;

///Setup********************************************************************************************************************************************************************************************************************************
void setup() {
  delay(5000);

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
  CAN1.init_Mask(1, 0, 0x7ff);  //Rx Mask 2 // funktioniert mit 3ff und mit 7ff

  CAN1.init_Filt(0, 0, 0x601);  //Filter for message with the ID 0x601
  CAN1.init_Filt(1, 0, 0x18B);  //For additional filter, add CAN.init_Filt(X, 0, 0xID), where X describes # of filter (up to 6 filters)

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
  //aCar_ON(&receive_aCarONOFF);
  //MB_ON(&receive_MB_ONOFF);
int a = 1;
   while (/*receive_aCarONOFF == 1 && receive_MB_ONOFF == 1*/ a == 1) {
        //aCar_ON(&receive_aCarONOFF);
        //MB_ON(&receive_MB_ONOFF);

        read_aCar(&aCar_receive, &aCar_Voltage, &aCar_Current, &aCar_Power, &SOC_FP, &speed);
        read_MB(&receive_MB, &Voltage_MB, &Current_MB, &Power_MB, &SOC_MB, &n_MB);  

        energy_managment_monotonous(SOC_MB, SOC_FP, n_MB, &Pdem_MB);

        Serial.print("MB SOC = ");
        Serial.println(SOC_MB);

        Serial.print("FP SOC = ");
        Serial.println(SOC_FP);

        Serial.print("n_MB = ");
        Serial.println(n_MB);

        Serial.print("Pdem_MB = ");
        Serial.println(Pdem_MB);

        Serial.println("---------------------------------------------");
    }
   
}
