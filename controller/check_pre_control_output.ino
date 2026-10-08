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

int maxIterations = 50;

double U_OCV_FP(float SOC_FP) {
  double U_OCV = 0;
  double OCV_FP[101] = { 31.943324, 36.172279, 37.531829, 38.437631, 39.053117, 39.402310, 39.662698, 39.875618, 40.054849, 40.211677,
                         40.354007, 40.488733, 40.625955, 40.771108, 40.920759, 41.071572, 41.219237, 41.361562, 41.500198, 41.636080,
                         41.765566, 41.885593, 41.996387, 42.096397, 42.188551, 42.274745, 42.354620, 42.432020, 42.506761, 42.577683,
                         42.648224, 42.717742, 42.786716, 42.854649, 42.921813, 42.990603, 43.059084, 43.128457, 43.198499, 43.268734,
                         43.341608, 43.416160, 43.491180, 43.567812, 43.647214, 43.727882, 43.811129, 43.896500, 43.984070, 44.073411,
                         44.167022, 44.262120, 44.358918, 44.458415, 44.560340, 44.663747, 44.768874, 44.876102, 44.985356, 45.094057,
                         45.204469, 45.315176, 45.425738, 45.536382, 45.646119, 45.754656, 45.862437, 45.967453, 46.073430, 46.177545,
                         46.280738, 46.382898, 46.483535, 46.585096, 46.686236, 46.788842, 46.892362, 47.000119, 47.109582, 47.224189,
                         47.342321, 47.464060, 47.591746, 47.723297, 47.858195, 47.996816, 48.138056, 48.280241, 48.421288, 48.561842,
                         48.698537, 48.830517, 48.955862, 49.075834, 49.192016, 49.306364, 49.424668, 49.550789, 49.691235, 49.862474,
                         49.862474 };
  int SOC_FP_r = round(SOC_FP);
  U_OCV = OCV_FP[SOC_FP_r];
  return U_OCV;
}
double R_I_FP(float SOC_FP) {
  double R_I_FP = 0;
  double R_FP[101] = { 0.024851, 0.033319, 0.027790, 0.024617, 0.022215, 0.019027, 0.016471, 0.014633, 0.013311, 0.012322,
                       0.011584, 0.011013, 0.010545, 0.010108, 0.009728, 0.009481, 0.009312, 0.009192, 0.009120, 0.009124,
                       0.009150, 0.009127, 0.009103, 0.009052, 0.008953, 0.008837, 0.008726, 0.008610, 0.008523, 0.008413,
                       0.008325, 0.008239, 0.008160, 0.008084, 0.008022, 0.007961, 0.007912, 0.007875, 0.007829, 0.007807,
                       0.007771, 0.007762, 0.007747, 0.007735, 0.007735, 0.007743, 0.007741, 0.007759, 0.007769, 0.007777,
                       0.007771, 0.007782, 0.007769, 0.007752, 0.007700, 0.007676, 0.007627, 0.007558, 0.007507, 0.007441,
                       0.007374, 0.007317, 0.007244, 0.007169, 0.007110, 0.007034, 0.006972, 0.006898, 0.006837, 0.006775,
                       0.006702, 0.006639, 0.006580, 0.006509, 0.006443, 0.006359, 0.006287, 0.006198, 0.006107, 0.006011,
                       0.005889, 0.005778, 0.005663, 0.005557, 0.005461, 0.005388, 0.005330, 0.005306, 0.005315, 0.005341,
                       0.005418, 0.005542, 0.005704, 0.005888, 0.006049, 0.006115, 0.006090, 0.005953, 0.005663, 0.005074,
                       0.005074 };
  int SOC_FP_r = round(SOC_FP);
  R_I_FP = R_FP[SOC_FP_r];
  return R_I_FP;
}
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
void set_voltage_DCDC(double Voltage) {                          //set Voltage
  float FactorVoltage = 100;                                // Offset Voltage
  unsigned int V_100 = round(Voltage * FactorVoltage / 2);  // Voltage wird auf beide DCDC-Wandler aufgeteilt und mit *100 multipliziert um als CAN-Message weitergegben werden zu können, round damit keine Komma zahl entstehen kann
  unsigned char buf[4] = { 0, 0, 0, 0 };
  if (V_100 <= 255) {  // besetzt nur buf[0] -> kein intel-Format
    buf[0] = V_100 & 0xFF;
  }
  if (V_100 > 255 && V_100 <= 3000) {  // besezt buf[0] und buf[1] im intel Format
    buf[0] = V_100 & 0xFF;
    buf[1] = (V_100 >> 8) & 0xFF;
  }
  CAN1.sendMsgBuf(0x2E1, 0, 4, buf);
  CAN2.sendMsgBuf(0x2E1, 0, 4, buf);
}
void aCar_ON(int *receive) {  //checkt IGNSTATE des aCars, IGNSTATE=0, falls der motor aus ist
  unsigned char len = 0;                   //byte
  unsigned char buf[8];
  int receiveAttempt = 0;
  *receive = 0;

  while (receiveAttempt == 0) {  // solange das aCar CAN-Messenges empfängt, checkt es diese solange, bis 0x18B gelesen werden konnte
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
  unsigned char len = 0;                   //byte
  unsigned char buf[8];
  int receiveAttempt = 0;
  *receive = 0;
  while (receiveAttempt == 0) {  // solange das aCar CAN-Messenges empfängt, checkt es diese solange, bis 0x18B gelesen werden konnte
    if (CAN_MSGAVAIL == CAN3.checkReceive()) {
      CAN1.readMsgBuf(&len, buf);
      unsigned long canID = CAN3.getCanId();
      if (canID == 0x355) {
        *receive = 1;
        receiveAttempt = 1;
        Serial.println("Modular Battery is on");
      }
    } else {
      Serial.println("Modular Battery NOT on");
      delay(100);
    }
  }
}
void read_aCar(int *receive, float *Voltage, float *Current, float *Power, float *SOC) {
  unsigned char len = 0;
  unsigned char buf[8];
  uint16_t iVoltage = 0;
  int16_t iCurrent = 0;
  float FactorVoltage = 0.01;  // Offset Voltage
  float FactorCurrent = 0.1;   // Offset Current
  int receiveAttempts = 0;
  *receive = 0;
  while (receiveAttempts <= maxIterations) {
    if (CAN_MSGAVAIL == CAN1.checkReceive()) {
      unsigned long canID = CAN1.getCanId();
      CAN1.readMsgBuf(&len, buf);  // Nachricht wird gelesen und auf Array "buf" gespeichert, länge der Nachricht auf len gespeichert
      if (canID == 0x601) {
        receiveAttempts = maxIterations;
        *receive = 1;
        iVoltage = (buf[7] << 8) | buf[6];    // ivoltage = 16 bit int -> die logik setzt buf[7] auf die vorderen 8 bit und buf[6] auf die hinteren 8 Bit
        *Voltage = iVoltage * FactorVoltage;  //Factor offset voltage

        iCurrent = (buf[5] << 8) | buf[4];
        *Current = FactorCurrent * iCurrent;

        *Power = *Current * *Voltage;

        *SOC = buf[0];
      }
    } else {
      *receive = 0;
    }
    receiveAttempts++;
  }
}
void read_MB(int *receive, float *Voltage, float *Current, float *Power, float *SOC, int *n_MB){
  unsigned char len = 0;
  unsigned char buf[8];
  uint16_t iVoltage_MB = 0;
  int16_t iCurrent_MB = 0;
  float FactorVoltage = 0.01;  // Offset Voltage
  float FactorCurrent = 0.1;   // Offset Current
  int receiveAttempts = 0;
  *receive = 0;
  while (receiveAttempts <= maxIterations) {
    if (CAN_MSGAVAIL == CAN3.checkReceive()) {
      unsigned long canID = CAN3.getCanId();
      CAN3.readMsgBuf(&len, buf);
      if (canID == 0x356) { 
        receiveAttempts = maxIterations;                  
        *receive = 1;
        iVoltage_MB = (buf[1] << 8) | buf[0];  
        *Voltage = iVoltage_MB * FactorVoltage;
        iCurrent_MB = (buf[3] << 8) | buf[2];
        *Current = FactorCurrent * iCurrent_MB;
        *Power = *Current * *Voltage;
      }

      if (canID == 0x355){
        *SOC = buf[0];
      }
      if (canID == 0x355 && canID != 0x1001){
        *n_MB = 1;
      }
      if(canID == 0x1001 && canID != 0x1021){
        *n_MB = 2;
      }
      if (canID == 0x1021){
        *n_MB = 3;
      }
    } else {
      *receive = 0;
    }
    receiveAttempts++;
  }

}
void energy_managment_monotonous(float SOC_MB, float SOC_FP, float Pdem, int n_MB, float *Pdem_MB, float *Pdem_FP){
  int E_start_MB = 2346;
  int E_start_FP = 16500;
  int v_avg = 30;
  int b_avg = 270;
  float Pdem_MB_check;

  float E_MB = n_MB * SOC_MB * E_start_MB;
  float E_tot = SOC_FP * E_start_FP + E_MB;
  float C_rate = (v_avg * b_avg) / E_tot;  //check /0
  Pdem_MB_check = E_MB * C_rate;

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
  *Pdem_FP = Pdem - *Pdem_MB;
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

float Pdem_MB = 0;
float Pdem_FP = 0;

float U_DCDC = 0;
float I_DCDC = 0;

float P_ist = 0;
float P_soll = 0;
float P_Last = 0;
int t_count = 0;


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


///Main Loop********************************************************************************************************************************************************************************************************************************

void loop() {
  aCar_ON(&receive_aCarONOFF);
  MB_ON(&receive_MB_ONOFF);
  if (receive_aCarONOFF == 0  && receive_MB_ONOFF == 1 ) {
    turn_DCDC_on();
    while (receive_aCarONOFF == 1 && receive_MB_ONOFF == 1) {
        aCar_ON(&receive_aCarONOFF);
        MB_ON(&receive_MB_ONOFF);
      if (millis() >= t + dt) {
        tMem = t;
        t = millis();

        read_aCar(&aCar_receive, &aCar_Voltage, &aCar_Current, &aCar_Power, &SOC_FP);
        read_MB(&receive_MB, &Voltage_MB, &Current_MB, &Power_MB, &SOC_MB, &n_MB);
        energy_managment_monotonous(SOC_MB, SOC_FP, aCar_Power, n_MB ,&Pdem_MB, &Pdem_FP);
        
        if (aCar_receive == 1) {
          R_I = R_I_FP(SOC_FP);
          U_OCV = U_OCV_FP(SOC_FP);
        }

        P_Last = aCar_Power;

        radicand = (U_OCV * U_OCV) - 4 * (aCar_Power - Pdem_MB) * R_I;
        if (radicand < 0) {
          Serial.println("Error: Negative Number under square root!");
          radicand = 0;
        }
        pre_control_output = (U_OCV + sqrt(radicand)) / 2;

        Serial.print("pre control output voltage: ");
        Serial.print(pre_control_output);
        Serial.print("\n");
        Serial.print("FP Voltage");
        Serial.print(aCar_Voltage);
        Serial.print("\n");
        Serial.print("--------------------------------");
        Serial.print("\n");
      }
      t_count++;
    }
  }
}
