/**
 * @file : retarget.h
 * @brief : Contain function to add printf and scanf function use the UART2
 */

#ifndef INC_RETARGET_H_
#define INC_RETARGET_H_

#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32f4xx_hal.h"
#include <sys/stat.h>

/**
 * Reconfigure stdin, stdout and stderr to use UART
 * @param huart Structure wich represent an UART
 */
void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);
int _getpid(void);
int _kill(int pid, int sig);

#endif //#ifndef _RETARGET_H__

#endif /* INC_RETARGET_H_ */
