#include "stdio.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "MK64F12.h"
#include "uart.h"
#include "LEDS.h"
#include "Bluetooth.h"


// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk
//    (camera clk is the mod value set in FTM2)
// default = .0075f
#define INTEGRATION_TIME .0075f
#define BAUD_RATE 9600      //default baud rate
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

#define MAX_BUF_SZ 128

/*
 Limit n to the lower and upper bounds only.
 */
#define clip(n, lower, upper){\
if(n<lower){\
n=lower;\
}else if(n>upper){\
n=upper;\
}\
}

extern void normalSet(void);
extern int control[10];

extern float LB;
extern float UB;
extern float LEFT_DESIRED;
extern float RIGHT_DESIRED;

extern int ready;
extern float manualDelta[2];
int RX_lcv = 0; //Current index of UART_TX_STR

extern float KP;
extern float KI;
extern float KD;

extern int manualControl;

int controlIndex = -1;
extern float angle;

extern android_data *data;

char sigStartBuffer[] = "00";
char sigStopBuffer[] = "00";
char dataNameBuffer[] = "00";
uint8_t *dataValueBuffer;

/*
 Initialize UART3 on pins PTB10 and PTB 11. Used to transmit
 and recieve control data over the HC-06 bluetooth slave module.
 Enables recieve interrupts at 9600 baud.
 Initializes android data structure.
 */
void init_BT(){
  uint16_t ubd, brfa;
  SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTB_PCR11 |= PORT_PCR_MUX(3); //BT TX
  PORTB_PCR10 |= PORT_PCR_MUX(3); //BT RX
  UART3_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
  UART3_C1 = 0;
  UART3_BDH &= ~UART_BDH_SBR_MASK;
  ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));
  UART3_BDH = (((ubd & 0x1F00) >> 8));
  UART3_BDL = (uint8_t)(ubd & UART_BDL_SBR_MASK);
  UART3_C4 &= ~(UART_C4_BRFA_MASK);
  UART3_C4 |= UART_C4_BRFA(brfa);
  UART3_C2 |= UART_C2_RIE_MASK; //Enable recieve interrupts
  UART3_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
  NVIC_EnableIRQ(UART3_RX_TX_IRQn);
  
  // Memory allocations
  data = malloc(sizeof(android_data));
  dataValueBuffer = malloc(sizeof(uint8_t) * SZ_DOUBLE);
}

/*
 * Simple cleanup for bt.
 */
void delData() {
  free(data);
  free(dataValueBuffer);
}

/*
 Gets information input from Android device via bluetooth.
 Protocol:
 [Input Char][Input value]
 Known Inputs:
 SXXX == Speed = XXX, 0 <= XXX <= 100
 TX.XX == Turn = X.XX, -30 <= X.XX <= 30
 AxX.XXAyY.YYAzZ.ZZ = (Accelerometer) x = X.XX, y = Y.YY, z = Z.ZZ
 GxX.XXXXGyY.YYYY = (GPS Location) x = X.XXXX, y = Y.YYYY
 
 */
void UART3_RX_IRQHandler(void) {
  /*
   uint8_t val;
   char name;
   if(UART3_D >= '0' && UART3_D <= '9') {
   val = UART3_D;
   } else {
   name = UART3_D;
   }
   if (controlIndex == -1) {
   switch (name) {
   case 'S':
   controlIndex = 1;
   break;
   case 'T':
   controlIndex = 2;
   break;
   case 'Ax':
   controlIndex = 3;
   break;
   case 'Ay':
   controlIndex = 4;
   break;
   case 'Az':
   controlIndex = 5;
   break;
   case 'Gx':
   controlIndex = 6;
   break;
   case 'Gy':
   controlIndex = 7;
   break;
   default:
   controlIndex = -1;
   break;
   }
   } else {
   control[controlIndex] = val;
   controlIndex = -1;
   }
   */
}

void UART3_TX_IRQHandler(void) {
  return;
}

void sendFloatTx(void) {
  
}


void pollGPSRx(void) {
  
}

/*
 Gets input from bluetooth.
 Styles:
 0 == purposly left blank
 1 == stops car
 2 == reset to normal speeds
 3 == manual control
 4 == ready
 5 == speed
 6 == turn angle
 
 
 */
//TODO
//have it send stuff back? (Say what it did)
void UART3_RX_TX_IRQHandler(void){
  uint8_t ctrl;
  char get_str[254]={0};
  char  c[255];
  float f;
  
  UART3_S1; //clears interrupt
  ctrl = UART3_D;
  
  //LEDon(YELLOW);
  if(ctrl > 0){
    sprintf(c,"Command: %i",ctrl);
    put(c);
    put("\r\n");
  }
  //LEDon(YELLOW);
  //sprintf(c,"Command: %i",ctrl);
  //put(c);
  //put("\r\n");
  if(ctrl >= 0 && ctrl <= 14){
    //Disable interrupts, start polling
    UART3_C2 &= ~UART_C2_RIE_MASK;
    if(ready == 0 && ctrl != 4){
      //Re-enable interrupts
      UART3_C2 |= UART_C2_RIE_MASK;
      return;
    }
    if(ctrl == 1){//stop car
      LEFT_DESIRED = 0.0f;
      RIGHT_DESIRED = 0.0f;
      manualDelta[0] = 0.0f;
      manualDelta[1] = 0.0f;
      manualControl=0;
      ready=0;
    }else if(ctrl == 2){ //normal speed
      ready=0;
    }else if(ctrl == 1){ //normal speed
      normalSet();
    }else if(ctrl == 3){//manual control
      if(manualControl==0){
        manualControl=1;
        LEDon(WHITE);
      }
      else{
        manualControl=0;
        LEDon(GREEN);
      }
      manualDelta[0]=0.0f;
      manualDelta[1]=0.0f;
    }else if(ctrl == 4) { //ready
      
      manualDelta[0]=0.0f;
      manualDelta[1]=0.0f;
    }else if(ctrl == 3) { //ready
      ready = 1;
    }else if(ctrl==5){ //speed
      bt_getAscii(get_str);
      //put(get_str);
      //put("\r\n");
      f = (float)atof(get_str);
      manualDelta[0] = f;
      manualDelta[1] = f;
    }else if(ctrl == 6){//angle
      bt_getAscii(get_str);
      //put(get_str);
      //put("\r\n");
      angle = (int)atoi(get_str);
    }
  }
  //Re-enable interrupts
  UART3_C2 |= UART_C2_RIE_MASK;
  return;
}

uint8_t bt_getbyte(void){
  while(!(UART3_S1 & UART_S1_RDRF_MASK));
  return UART3_D;
}

void bt_getAscii(char *ptr_str){
  int lcv;
  uint8_t cu;
  lcv=0;
  while(lcv < 254){
    cu = bt_getbyte();
    if(cu == 0){ //if entered character is character return
      return;
    }
    ptr_str[lcv] = cu;
    lcv++;
  }
  return;
}

/*
 * Primary function to get data from android app.
 */
void bt_getData() {
  uint8_t b;                        // Storage for most recent bt byte
  int attempt = 0, lcv = 0, i = 0;  // Various loop control variables
  enum android_index index = Err;   // Android name control index
  
  while (attempt < 50) {                    // Arbitrary limit on data retrieval attempts
    if (strcmp(sigStartBuffer, SIGNAL)) {   // If not equal, the app hasn't sent data yet
      b = bt_getbyte();                     // Get current bt byte
      checkSigBuffer(sigStartBuffer, b);    // Cmp buffer to SIGNAL and modify as appropriate
      attempt += 1;                         // Increment control variable
      continue;                             // No point not continuing while start signal not received
    } else {
      while (lcv < 254) {                     // Arbitrary limit
        b = bt_getbyte();
        checkSigBuffer(sigStopBuffer, b);     // If stop buffer receives SIGNAL, function exits
        if (!strcmp(sigStopBuffer, SIGNAL)) { // Exit function
          return;
        }
        if (index != Err) {             // index == Err on first loop, collect data name first
          switch (index) {              // Switch so we know how many bytes are coming
            case Speed:
            case Turn:
              dataValueBuffer[i] = b;
              if (++i == SZ_SHORT) {                  // Let the loop go until we have all the bytes
                stitchBytes(dataValueBuffer, index);  // Casts data to the right type and sets android data
                index = Err;                          // Return to first loop status, index == Err
              }
              break;
            case AccelX:
            case AccelY:
            case AccelZ:
              dataValueBuffer[i] = b;
              if (++i == SZ_FLOAT) {
                stitchBytes(dataValueBuffer, index);
                index = Err;
              }
              break;
            case GpsX:
            case GpsY:
              dataValueBuffer[i] = b;
              if (++i == SZ_DOUBLE) {
                stitchBytes(dataValueBuffer, index);
                index = Err;
              }
              break;
            case Atn:
            case Sensor:
              dataValueBuffer[i] = b;
              if (++i == SZ_BOOL) {
                stitchBytes(dataValueBuffer, index);
                index = Err;
              }
              break;
              
            default:
              break;
          }
        } else {
          i = 0;                                    // Reset dataValueBuffer index after stitching
          if (dataNameBuffer[0] == '0') {           // Various name -> easier to assume what's next
            dataNameBuffer[0] = b;
          } else if (dataNameBuffer[1] == '0') {
            dataNameBuffer[1] = b;
          } else {
            index = checkDataName(dataNameBuffer);  // Will return Err if assumption failed
            dataNameBuffer[0] = dataNameBuffer[1];  // Avoid potential alignment issues
            dataNameBuffer[1] = '0';
          }
        }
      }
      break;
    }
  }
}

void stitchBytes(uint8_t *buffer, enum android_index index) {
  uint8_t val_b;
  uint32_t val_i;
  float val_f;
  double val_d;
  
  switch (index) {
    case Speed:
    case Turn:
      val_i = buffer[0] && (buffer[1] << 8) && (buffer[2] << 16) && (buffer[3] << 24);
      setData(index, &val_i);
      break;
    case AccelX:
    case AccelY:
    case AccelZ:
      val_f = buffer[0] && (buffer[1] << 8) && (buffer[2] << 16) && (buffer[3] << 24);
      setData(index, &val_f);
      break;
    case GpsX:
    case GpsY: // TODO IDE says shift count 32 >= size of type. True? Double should be 8 bytes
      val_d = buffer[0] && (buffer[1] << 8) && (buffer[2] << 16) && (buffer[3] << 24)
      && (buffer[4] << 32) && (buffer[5] << 40) && (buffer[6] << 48) && (buffer[7] << 56);
      setData(index, &val_d);
      break;
    case Atn:
    case Sensor:
      val_b = buffer[0] > 0 ? 1 : 0;
      setData(index, &val_b);
      break;
      
    default:
      break;
  }
}

/*
 * Helper function to improve readability.
 * Fills sig buffers as appropriate.
 */
void checkSigBuffer(char buffer[], uint8_t byte) {
  if (byte == SIGNAL[0]) {
    buffer[0] = SIGNAL[0];
  } else if (buffer[0] == SIGNAL[0] && byte == SIGNAL[1]) {
    buffer[1] = SIGNAL[1];
  } else {
    buffer[0] = buffer[1];
    buffer[1] = '0';
  }
}

/*
 * Match buffer with correct android setting.
 */
enum android_index checkDataName(char buffer[]) {
  if(!strcmp(buffer, SPEED)) {
    return Speed;
  } else if(!strcmp(buffer, TURN)) {
    return Turn;
  } else if(!strcmp(buffer, ACCELX)) {
    return AccelX;
  } else if(!strcmp(buffer, ACCELY)) {
    return AccelY;
  } else if(!strcmp(buffer, ACCELZ)) {
    return AccelZ;
  } else if(!strcmp(buffer, GPSX)) {
    return GpsX;
  } else if(!strcmp(buffer, GPSY)) {
    return GpsY;
  } else if(!strcmp(buffer, ATN)) {
    return Atn;
  } else if(!strcmp(buffer, SENS)) {
    return Sensor;
  } else {
    return Err;
  }
}

void setData(enum android_index index, void* value) {
  switch (index) {
    case Speed:
      data->speed = *(int32_t *) value;
      break;
    case Turn:
      data->turn = *(int32_t *) value;
      break;
    case AccelX:
      data->accelx = *(float *) value;
      break;
    case AccelY:
      data->accely = *(float *) value;
      break;
    case AccelZ:
      data->accelz = *(float *) value;
      break;
    case GpsX:
      data->gpsx = *(double *) value;
      break;
    case GpsY:
      data->gpsy = *(double *) value;
      break;
    case Atn:
      data->atn = *(uint8_t *) value;
      break;
    case Sensor:
      data->sensor = *(uint8_t *) value;
      break;
      
    default:
      break;
  }
}

uint8_t getData_b(enum android_index index) {
  switch (index) {
    case Atn:
      return data->atn;
    case Sensor:
      return data->sensor;
      
    default:
      return ANDROID_DATA_ERR;
  }
}

int32_t getData_i(enum android_index index) {
  switch (index) {
    case Speed:
      return data->speed;
    case Turn:
      return data->turn;
      
    default:
      return ANDROID_DATA_ERR;
  }
}

float getData_f(enum android_index index) {
  switch (index) {
    case AccelX:
      return data->accelx;
    case AccelY:
      return data->accely;
    case AccelZ:
      return data->accelz;
      
    default:
      return ANDROID_DATA_ERR;
  }
}

double getData_d(enum android_index index) {
  switch (index) {
    case GpsX:
      return data->gpsx;
    case GpsY:
      return data->gpsy;
      
    default:
      return ANDROID_DATA_ERR;
  }
}













