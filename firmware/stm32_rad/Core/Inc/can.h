/*
 * can.h
 *
 *  Created on: Oct 18, 2023
 *      Author: ishan
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include "main.h"


#define CAN_MESSAGE_DATA_LENGTH 8

typedef union
{

  uint32_t value:29;
  struct
  {
    uint8_t id:4;		//byte 0, bits 4-7
    uint32_t unused :25;			//byte 0, bits 1 - 3, bytes 1 - 3
  };
} CAN_RAD_IDENTIFIER;

typedef struct
{
	CAN_RAD_IDENTIFIER id;
	uint8_t data[CAN_MESSAGE_DATA_LENGTH];
} CAN_RAD_MESSAGE;

typedef enum
{
	CAN_OK = 0,
	CAN_ERROR_INIT,
	CAN_ERROR_NO_MESSAGE,
	CAN_ERROR_MAX_MESSAGES
} CAN_ERROR;

CAN_ERROR CAN_init();
CAN_ERROR CAN_addMessage(uint32_t * id, uint8_t * data);
CAN_ERROR CAN_getMessage(CAN_RAD_MESSAGE * msg);
CAN_ERROR CAN_sendMessage(CAN_RAD_MESSAGE * msg);

#endif /* INC_CAN_H_ */
