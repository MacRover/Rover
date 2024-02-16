#ifndef __CAN_DEFINE_H_
#define __CAN_DEFINE_H_

#include <stdint.h>

#define GLOBAL_PRIORITY_IDENTIFIER_URGENT	0x00	//00
#define GLOBAL_PRIORITY_IDENTIFIER_HIGH		0x01	//01
#define GLOBAL_PRIORITY_IDENTIFIER_MEDIUM	0x02	//10
#define GLOBAL_PRIORITY_IDENTIFIER_LOW		0x03	//11

//define your cluster here
#define GLOBAL_CLUSTER_IDENTIFIER_RAD		0x06	//110

#define RAD_JOINT_IDENTIFIER_CLAW 			0x00	//000
#define RAD_JOINT_IDENTIFIER_ELBOW 			0x01	//001
#define RAD_JOINT_IDENTIFIER_WRIST_LEFT 	0x02	//010
#define RAD_JOINT_IDENTIFIER_WRIST_RIGHT 	0x03	//011
													//100
													//101
#define RAD_JOINT_IDENTIFIER_BASE_LEFT		0x06	//110
#define RAD_JOINT_IDENTIFIER_BASE_RIGHT 	0x07	//111

#define RAD_FUNCTION_IDENTIFIER_STEP_STOP	0x00	//00
#define RAD_FUNCTION_IDENTIFIER_STEP_ENCODER	0x01	//01
#define RAD_FUNCTION_IDENTIFIER_STEP_CW		0x02	//10
#define RAD_FUNCTION_IDENTIFIER_STEP_CCW	0x03	//11

#define GLOBAL_PRIORITY_IDEINTIFIER_MASK	0x0300	//011 00000000
#define GLOBAL_CLUSTER_IDENTIFIER_MASK 		0xE0 	//000 11100000
#define RAD_JOINT_IDENTIFIER_MASK			0x1C 	//000 00011100
#define RAD_FUNCTION_IDENTIFIER_MASK		0x03	//000 00000011

uint8_t currnentRadJoint;

#define CURRENT_RAD_JOINT currentRadJoint;

//if EEPROM Doesn't work out, manually configure device being flashed here
//#define CURRENT_RAD_JOINT RAD_JOINT_IDENTIFIER_CLAW


typedef union
{

  uint32_t value:29;
  struct
  {
    uint8_t function:2;		//byte 0, bits 5-7
    uint8_t joint:3;		//byte 0, bits 2-4
    uint8_t cluster:3;		//byte 0, bits 0-1
    uint8_t priority:2;		//byte 1, bits 0-1		//byte 1, bits 2-7
    uint32_t unused :19;			//byte 3, bits 0 - 54
  };
} CAN_RAD_IDENTIFIER;

typedef struct
{
	CAN_RAD_IDENTIFIER id;
	uint8_t data[8];
} CAN_RAD_MESSAGE;































#endif
