#ifndef GPS_H_
#define GPS_H_

void getGPS(void);
void gpsDemo(void);
void uart2_init(void);
uint8_t uart2_getchar(void);
void uart2_putchar(char ch);
void uart2_get(char *ptr_str);
void uart2_put(char *ptr_str);
void uart2_get_DistAngle(char *ptr_str_dist, char *ptr_str_angle);
#endif /* GPS_H_ */
