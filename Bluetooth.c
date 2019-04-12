#include "stdio.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "MK64F12.h"
#include "uart.h"
#include "LEDS.h"
#include <string.h>
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
char sigStopBuffer[]  = "00";
char dataNameBuffer[] = "00";

char SIGNAL[]  = "AC";
char SPEED[]   = "Sp";
char TURN[]    = "Tu";
char ACCELX[]  = "Ax";
char ACCELY[]  = "Ay";
char ACCELZ[]  = "Az";
char GPSX[]    = "Gx";
char GPSY[]    = "Gy";
char ACCLOC[]  = "AcL";
char AVGGPSX[] = "Agx";
char AVGGPSY[] = "Agy";
char ACCAVG[]  = "AcA";

const int SZ_BOOL = 1, SZ_SHORT = 3, SZ_FLOAT = 4, SZ_DOUBLE = 8;
int android_size[] = {0, SZ_SHORT, SZ_SHORT, SZ_FLOAT, SZ_FLOAT, SZ_FLOAT, SZ_DOUBLE, SZ_DOUBLE, SZ_BOOL, SZ_BOOL};
const uint8_t ANDROID_DATA_ERR = (uint8_t) -9999;

int gotGPSdata = 0;
int receiveGPSdata = 0;

extern void delay(int);
float bt_lat;
float bt_long;

uint8_t *dataValueBuffer;

/*
 Initialize UART3 on pins PTB10 and PTB 11. Used to transmit
 and recieve control data over the HC-06 bluetooth slave module.
 Enables recieve interrupts at 9600 baud.
 Initializes android data structure.
 */
void init_BT(){
  uint16_t ubd, brfa = 0;
    
  // Memory allocations
  data = malloc(sizeof(android_data));
  dataValueBuffer = malloc(sizeof(uint8_t) * SZ_DOUBLE);
  
  // Initialize struct
  initAndroidData();
    
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
  
  
}

void get_BT_GPS_dev(double *latitude, double *longitude){
    receiveGPSdata = 1;
    while(gotGPSdata == 0);
    receiveGPSdata = 0;
    *latitude = bt_lat;
    *longitude = bt_long;
    gotGPSdata = 0;
    put("past gps bt");
}

/*
 * Simple initialize for android data
 */
void initAndroidData() {
  uint32_t val_i = 0;
  float val_f    = 0;
  double val_d   = 0;
  
  setData(Speed,   &val_i);
  setData(Turn,    &val_i);
  setData(AccelX,  &val_f);
  setData(AccelY,  &val_f);
  setData(AccelZ,  &val_f);
  setData(GpsX,    &val_d);
  setData(GpsY,    &val_d);
  setData(AccLoc,  &val_d);
  setData(AvgGpsX, &val_d);
  setData(AvgGpsY, &val_d);
  setData(AccAvg,  &val_d);
}


/*
 * Simple cleanup for bt.
 */
void delData() {
  free(data);
  free(dataValueBuffer);
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
  char get_str[254];
  char stream[1024];
  char *splitStream;
  char  c[255];
  float f;
  double d;

  UART3_C2 &= ~UART_C2_RIE_MASK;
  UART3_S1; //clears interrupt
  ctrl = UART3_D;
  

    if (ctrl == 14){
        bt_getStream(stream);
        //put("\r\n");
        splitStream = strtok(stream," ");
        while(splitStream != NULL){
            //put(splitStream);
            //put("\r\n");
            if (!strcmp(splitStream,"ALo")){
                splitStream = strtok(NULL, " ");
                /*put("BT Lat: ");
                put(splitStream);
                put("\r\n");*/
                sscanf(splitStream, "%lf", &d);
                data->avggpsx=d;
                splitStream = strtok(NULL, " ");
                splitStream = strtok(NULL, " ");
                /*put("BT Long: ");
                put(splitStream);
                put("\r\n");*/
                sscanf(splitStream, "%lf", &d);
                data->avggpsy=d;
                break;
            }
            splitStream = strtok(NULL, " ");
        }
      
      
      
        /*  
        bt_getAscii(c);
        bt_getAscii(c);
        bt_getAscii(c);
        data->speed = atoi(c);

        bt_getAscii(c);
        bt_getAscii(c);
        data->turn = atoi(c);

        bt_getAscii(c);
        bt_getAscii(c);
        sscanf(c, "%lf", &d);
        data->accelx = d;

        bt_getAscii(c);
        bt_getAscii(c);
        sscanf(c, "%lf", &d);
        data->accely = d;

        bt_getAscii(c);
        bt_getAscii(c);
        sscanf(c, "%lf", &d);
        data->accelz = d;

        bt_getAscii(c);
        bt_getAscii(c);
        sscanf(c, "%lf", &d);
        data->gpsx = d;

        bt_getAscii(c);
        bt_getAscii(c);
        sscanf(c, "%lf", &d);
        data->gpsy = d;



        bt_getAscii(c);
        bt_getAscii(c);
        sscanf(c, "%lf", &d);
        data->accloc = d;

        bt_getAscii(c);
        bt_getAscii(c);
        //put(c);
        //put("\r\n");
        sscanf(c, "%lf", &d);
        data->avggpsx = d;


        bt_getAscii(c);
        bt_getAscii(c);
        //put(c);
        //put("\r\n");
        sscanf(c, "%lf", &d);
        data->avggpsy = d;

        bt_getAscii(c);
        bt_getAscii(c);
        sscanf(c, "%lf", &d);
        data->accavg = d;

        //bt_getData();
        */
        UART3_C2 |= UART_C2_RIE_MASK;


        return;
    }
  if(ctrl >= 0 && ctrl <= 14){
    //Disable interrupts, start polling
    UART3_C2 &= ~UART_C2_RIE_MASK;
    if(ready == 0 && ctrl != 4 && ctrl != 14){
      //Re-enable interrupts
      UART3_C2 |= UART_C2_RIE_MASK;
      return;
    }
    if(ctrl == 1){//stop car
      LEFT_DESIRED   = 0.0f;
      RIGHT_DESIRED  = 0.0f;
      manualDelta[0] = 0.0f;
      manualDelta[1] = 0.0f;
      manualControl  = 0;
      ready = 0;
    } else if(ctrl == 2){ //normal speed
      normalSet();
    } else if(ctrl == 3){//manual control
      if(manualControl == 0){
        manualControl = 1;
        LEDon(WHITE);
      } else{
        manualControl = 0;
        LEDon(GREEN);
      }
      manualDelta[0] = 0.0f;
      manualDelta[1] = 0.0f;
    } else if(ctrl == 4) { //ready
      ready = 1;
      manualDelta[0] = 0.0f;
      manualDelta[1] = 0.0f;
    } else if(ctrl == 5){ //speed
      bt_getAscii(get_str);
      //put(get_str);
      //put("\r\n");
      f = (float) atof(get_str);
      manualDelta[0] = f;
      manualDelta[1] = f;
    } else if(ctrl == 6){//angle
      bt_getAscii(get_str);
      //put(get_str);
      //put("\r\n");
      angle = (int) atoi(get_str);
    } else if (ctrl == 7){
        //Will fail and hard break if tested in lab
        //Reason is due to lack of location services
        bt_getAscii(get_str);
        bt_lat=(float) atof(get_str);
        bt_getAscii(get_str);
        bt_long= (float) atof(get_str);
        gotGPSdata=1;
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


void sendFloatTx(void) {
  
}


void pollGPSRx(void) {
  
}








void bt_getStream(char *stream_ptr){
    const int data_array_sections = 9; //items in bt data array
    int lcv=0;
    int kcv=0;
    int qcv=0;
    uint8_t cu;
    uint8_t au;
    while(lcv < 1024){
        cu = bt_getbyte();
        if(cu==0){
            stream_ptr[lcv] = ' ';
            lcv++;
            stream_ptr[lcv] = ' ';
            lcv++;
            continue;
        }
        //uart_putchar(cu);
        stream_ptr[lcv] = cu;
        lcv++;
        if (stream_ptr[lcv-2]=='A' & stream_ptr[lcv-1]=='C'){
            kcv++;
            if(kcv==2){
                return;
            }
        }
    }
    
    
}


void bt_getAscii(char *ptr_str){
  int lcv;
  uint8_t cu;
  lcv = 0;
  while(lcv < 255){
    cu = bt_getbyte();
    if(cu == 0){ //if entered character is character return
      return;
    }
    uart_putchar(cu);
    ptr_str[lcv] = cu;
    lcv++;
  }
  return;
}

/*
 * Primary function to get data from android app.
 */
void bt_getData(void) { //TODO: Make this actually work.
  uint8_t b;                        // Storage for most recent bt byte
  int attempt = 0, lcv = 0, i = 0;  // Various loop control variables
  enum android_index index = Err;   // Android name control index
  
  while (attempt < DEFAULT_SYSTEM_CLOCK*10) {                    // Arbitrary limit on data retrieval attempts
    if (!strcmp(sigStartBuffer, SIGNAL)) {  // If not equal, the app hasn't sent data yet
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
            case AccLoc:
            case AvgGpsX:
            case AvgGpsY:
            case AccAvg:
              dataValueBuffer[i] = b;
              if (++i == SZ_DOUBLE) {
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
  uint32_t val_i;
  float val_f;
  double val_d;
  
  switch (index) {
    case Speed:
    case Turn:
      val_i = 0;
      memcpy(&val_i, buffer, sizeof(val_i));
      setData(index, &val_i);
      break;
    case AccelX:
    case AccelY:
    case AccelZ:
      val_f = 0;
      memcpy(&val_f, buffer, sizeof(val_f));
      setData(index, &val_f);
      break;
    case GpsX:
    case GpsY:
    case AccLoc:
    case AvgGpsX:
    case AvgGpsY:
    case AccAvg:
      val_d = 0;
      memcpy(&val_d, buffer, sizeof(val_d));
      setData(index, &val_d);
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
  } else if(!strcmp(buffer, ACCLOC)) {
    return AccLoc;
  } else if(!strcmp(buffer, AVGGPSX)) {
    return AvgGpsX;
  } else if(!strcmp(buffer, AVGGPSY)) {
    return AvgGpsY;
  } else if(!strcmp(buffer, ACCAVG)) {
    return AccAvg;
  } else {
    return Err;
  }
}

void setData(enum android_index index, void* value) {
  switch (index) {
    case Speed:
      memcpy(&data->speed,   value, sizeof(data->speed));
      break;
    case Turn:
      memcpy(&data->turn,    value, sizeof(data->turn));
      break;
    case AccelX:
      memcpy(&data->accelx,  value, sizeof(data->accelx));
      break;
    case AccelY:
      memcpy(&data->accely,  value, sizeof(data->accely));
      break;
    case AccelZ:
      memcpy(&data->accelz,  value, sizeof(data->accelz));
      break;
    case GpsX:
      memcpy(&data->gpsx,    value, sizeof(data->gpsx));
      break;
    case GpsY:
      memcpy(&data->gpsy,    value, sizeof(data->gpsy));
      break;
    case AccLoc:
      memcpy(&data->accloc, value, sizeof(data->accloc));
      break;
    case AvgGpsX:
      memcpy(&data->avggpsx, value, sizeof(data->avggpsx));
      break;
    case AvgGpsY:
      memcpy(&data->avggpsy, value, sizeof(data->avggpsy));
      break;
    case AccAvg:
      memcpy(&data->accavg, value, sizeof(data->accavg));
      break;
      
    default:
      break;
  }
}

uint8_t getData_b(enum android_index index) {
  switch (index) {
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
    case AccLoc:
      return data->accloc;
    case AvgGpsY:
      return data->avggpsy;
    case AvgGpsX:
      return data->avggpsx;
    case AccAvg:
      return data->accavg;
      
    default:
      return ANDROID_DATA_ERR;
  }
}













