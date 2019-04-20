#ifndef CAMERA_H_
#define CAMERA_H_

void getCamera(void);
void uart4_init(void);
uint8_t uart4_getchar(void);
void uart4_putchar(char ch);
void uart4_get(char *ptr_str);
void uart4_put(char *ptr_str);
void uart4_get_DistAngle(char *ptr_str_dist, char *ptr_str_angle);
#endif /* CAMERA_H_ */
