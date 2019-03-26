#ifndef Bluetooth_H_
#define Bluetooth_H_

const uint8_t ANDROID_DATA_ERR = (uint8_t) -9999;
const int SZ_BOOL = 1, SZ_SHORT = 3, SZ_FLOAT = 4, SZ_DOUBLE = 8;
enum android_index{Err=-1, Speed=0, Turn, AccelX, AccelY, AccelZ, GpsX, GpsY, Atn, Sensor};
int android_size[] =
{0, SZ_SHORT, SZ_SHORT, SZ_FLOAT, SZ_FLOAT, SZ_FLOAT, SZ_DOUBLE, SZ_DOUBLE, SZ_BOOL, SZ_BOOL};

char
SIGNAL[] = "AC",
SPEED[]  = "Sp",
TURN[]   = "Tu",
ACCELX[] = "Ax",
ACCELY[] = "Ay",
ACCELZ[] = "Az",
GPSX[]   = "Gx",
GPSY[]   = "Gy",
ATN[]    = "Au",
SENS[]   = "Se";

typedef struct {
  int32_t speed;
  int32_t turn;
  float accelx;
  float accely;
  float accelz;
  double gpsx;
  double gpsy;
  uint8_t atn;
  uint8_t sensor;
} android_data;


void init_BT(void);

void delData(void);
void setData(enum android_index index, void* value);
uint8_t getData_b(enum android_index index);
int32_t getData_i(enum android_index index);
float getData_f(enum android_index index);
double getData_d(enum android_index index);
void bt_getData(void);

void stitchBytes(uint8_t *buffer, enum android_index index);
void checkSigBuffer(char buffer[], uint8_t byte);
enum android_index checkDataName(char buffer[]);

void UART3_RX_IRQHandler(void);
void UART3_TX_IRQHandler(void);
void UART3_RX_TX_IRQHandler(void);
void sendFloatTx(void);
void pollGPSRx(void);
uint8_t bt_getbyte(void);
void bt_getAscii(char *ptr_str);

#endif /* Bluetooth_H_ */
