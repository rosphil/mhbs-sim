#include <SPI.h>
#include "mcp2515_can.h"
#include <math.h>
#include <stdio.h>

const int SPI_CS_PIN1 = 9;
const int SPI_CS_PIN2 = 10;  // SPI_CS_PIN und SPI_CS_PIN1 definieren die Chip-Select-Pins, die für die Kommunikation mit den beiden CAN-Modulen verwendet werden
const int SPI_CS_PIN3 = 8;
const int CAN_INIT_PIN = 2;  // gibt Pin an, der für die Initialisierung des CAN-Moduls verwedet wird
mcp2515_can CAN1(SPI_CS_PIN1);
mcp2515_can CAN2(SPI_CS_PIN2);
mcp2515_can CAN3(SPI_CS_PIN3);


void read_MB(int *receive, float *Voltage, float *Current, float *Power, int *SOC, int *n_MB) {
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


  const int maxIterations = 500;  // Set a limit to avoid infinite loops
  int iterations = 0;

  while (allMessagesReceived == false) {
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

/// Constants
int SOC_FP = 0;
int SOC_MB = 0;

int receive_MB_ONOFF = 0;

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
float Voltage_MB = 0.0;
float Current_MB = 0.0;
float Power_MB = 0.0;
int n_MB = 0;

/// Setup
void setup() {
  //delay(5000);

  Serial.begin(115200);                      // initialisiert die serielle Kommunikation mit einer Baudrate von 115200 -> Geschwindigkeit der Datenübertragung zwischen dem Arduino und dem Computer
  while (CAN_OK != CAN1.begin(CAN_250KBPS))  // Initialisierung des CAN-Controllers, stellt sicher das Initialiserung erfolgreich ist
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

  CAN3.init_Mask(0, 0, 0xFFE);
  CAN3.init_Filt(0, 0, 0x355);
  CAN3.init_Filt(1, 0, 0x356);

  CAN3.init_Mask(1, 1, 0xFFC0);
  CAN3.init_Filt(2, 1, 0x1001);
}

/// Main Loop
void loop() {
  read_MB(&receive_MB, &Voltage_MB, &Current_MB, &Power_MB, &SOC_MB, &n_MB);
  while (receive_MB == 1) {
    read_MB(&receive_MB, &Voltage_MB, &Current_MB, &Power_MB, &SOC_MB, &n_MB);

    Serial.print("receive = ");
    Serial.print(receive_MB);
    Serial.print("\n");
    Serial.print("Voltage = ");
    Serial.print(Voltage_MB);
    Serial.print("\n");
    Serial.print("Current = ");
    Serial.print(Current_MB);
    Serial.print("\n");
    Serial.print("SOC_MB = ");
    Serial.print(SOC_MB);
    Serial.print("\n");
    Serial.print("n_MB = ");
    Serial.print(n_MB);
    Serial.print("\n");
    Serial.print("-------------------------------------------------");
    Serial.print("\n");
    delay(500);
  }
  t_count++;
  // }
}
