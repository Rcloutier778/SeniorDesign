#ifndef Bluetooth_H_
#define Bluetooth_H_

enum android_index{Err=-1, Speed=0, Turn, AccelX, AccelY, AccelZ, GpsX, GpsY, Atn, Sensor};

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
void get_BT_GPS_dev(double *latitude, double *longitude);

void initAndroidData(void);
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
