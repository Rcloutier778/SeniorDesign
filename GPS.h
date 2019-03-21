#ifndef GPS_H_
#define GPS_H_
void initGPS(void);
void getGPS(void);
void uart2_init(void);
uint8_t uart2_getchar(void);
void uart2_putchar(char ch);
void uart2_get(char *ptr_str);
void uart2_put(char *ptr_str);
#endif /* GPS_H_ */
