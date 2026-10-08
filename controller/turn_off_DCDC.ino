#include <SPI.h>
#include "mcp2515_can.h"
#include <math.h>
#include <stdio.h>

const int SPI_CS_PIN1 = 9;  //Set SPI CS Pin to D9
const int SPI_CS_PIN2 = 10;
const int CAN_INT_PIN = 2;
mcp2515_can CAN1(SPI_CS_PIN1);
mcp2515_can CAN2(SPI_CS_PIN2);

int maxIterations = 15;

void turn_DCDC_on() {  // Funktion zum Einschalten beider DCDC-Wandler
  unsigned char buf_on[1] = { 1 };
  CAN1.sendMsgBuf(0x2E0, 0, 1, buf_on);
  CAN2.sendMsgBuf(0x2E0, 0, 1, buf_on);
  Serial.println("DCDC-Converters turned ON\n");
}
void turn_DCDC_off() {  // Funktion zum Ausschalten beider DCDC-Wandler
  unsigned char buf_off[1] = { 0 };
  CAN1.sendMsgBuf(0x2E0, 0, 1, buf_off);
  CAN2.sendMsgBuf(0x2E0, 0, 1, buf_off);
  Serial.println("DCDC-Converters turned OFF\n");
}
void set_voltage_DCDC(double Voltage) {                 //set Voltage
  float FactorVoltage = 100;                            // Offset Voltage
  unsigned int V_100 = round(Voltage * FactorVoltage);  // Voltage wird auf beide DCDC-Wandler aufgeteilt und mit *100 multipliziert um als CAN-Message weitergegben werden zu können, round damit keine Komma zahl entstehen kann
  unsigned char buf[4] = { 0, 0, 0, 0 };
  if (V_100 <= 255) {  // besetzt nur buf[0] -> kein intel-Format
    buf[0] = V_100 & 0xFF;
  }
  if (V_100 > 255 && V_100 <= 3000) {  // besezt buf[0] und buf[1] im intel Format
    buf[0] = V_100 & 0xFF;
    buf[1] = (V_100 >> 8) & 0xFF;
  }
  CAN2.sendMsgBuf(0x2E1, 0, 4, buf);
}
void read_DCDC_act_Voltage(mcp2515_can CAN, float *act_Voltage) {  // DCDC-Wandler: antworten lesen
  CAN.sendMsgBuf(0x2E3, 0, 0, 0);
  unsigned char len = 0;  //byte
  unsigned char buf[8];
  uint16_t xVoltage = 0;
  float FactorVoltage = 100;  // Offset Voltage
  int receiveAttempts = 0;
  while (receiveAttempts <= maxIterations) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      CAN.readMsgBuf(&len, buf);
      unsigned long canID = CAN.getCanId();
      if (canID == 0x2F3) {
        receiveAttempts = maxIterations - 4;
        xVoltage = (buf[1] << 8) | buf[0];
        *act_Voltage = (float)xVoltage / FactorVoltage;
      }
    }
    receiveAttempts++;
  }
}
void read_DCDC_set_Voltage(mcp2515_can CAN, float *set_Voltage) {  // liest eingestellte Spannun des DCDC-Wandlers ab
  float FactorVoltage = 100;                                       // Offset Voltage
  CAN.sendMsgBuf(0x2E1, 0, 0, 0);                                  // Nachricht senden, um eine Antwort des DCDC-Wandlers auszulösen
  unsigned char len = 0;                                           // Größe der Nachricht in Byte
  unsigned char buf[8];                                            // Puffer zum Speichern der Antwortnachricht
  uint16_t xVoltage = 0;                                           // Zwischenspeicher für die Spannung
  int receiveAttempts = 0;                                         // Zähler für die Anzahl der Empfangsversuche
  while (receiveAttempts <= maxIterations) {                       // Schleife, die solange läuft bis die maximale Anzahl an Iterationen erreicht ist
    if (CAN_MSGAVAIL == CAN.checkReceive()) {                     // Überprüfen, ob eine Nachricht empfangen wurde
      CAN.readMsgBuf(&len, buf);                                  // Nachricht lesen
      unsigned long canID = CAN.getCanId();                        // ID der empfangenen Nachricht abrufen
      if (canID == 0x2F1) {                                        // Überprüfen, ob es sich um die richtige ID handelt
        receiveAttempts = maxIterations;                           // Empfangsversuche auf maximale Iterationen
        xVoltage = (buf[1] << 8) | buf[0];                         // Spannungswert aus der Antwortnachricht extrahieren
        *set_Voltage = (float)xVoltage / FactorVoltage;            // Spannungswert durch 100 teilen und in "act_Voltage" speichern
      }
    }
    receiveAttempts++;  // Anzahl der Empfangsversuche erhöhen
  }
}


void setup() {
  Serial.begin(115200);

  while (CAN_OK != CAN2.begin(CAN_250KBPS)) {
    Serial.println("CAN2 init fail, retry!");
    delay(1000);
  }
  Serial.println("CAN2 init ok!");

  CAN2.init_Mask(0, 0, 0x7ff);
  CAN2.init_Mask(1, 0, 0x7ff);

  CAN2.init_Filt(0, 0, 0x2F1);
  CAN2.init_Filt(1, 0, 0x2F3);
  CAN2.init_Filt(2, 0, 0x2F4);
}


// Parameter
float act_Current_1 = 0;
float act_Voltage_1 = 0;
float act_Current_2 = 0;
float act_Voltage_2 = 0;
float act_Voltage_ges = 0;
double Voltage = 10;
float set_Voltage_1 = 0;
float set_Voltage_2 = 0;
float set_Voltage_ges = 0;
unsigned char len = 0;  //byte
unsigned char buf[8];
int i = 0;




void loop() {
  // Anschalten:
  turn_DCDC_off();
  Serial.println("OFF");
  delay(10000);
}


