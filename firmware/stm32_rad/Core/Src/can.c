/*
 * can.c
 *
 *  Created on: Oct 18, 2023
 *      Author: ishan
 */


#include "can.h"


#define QUEUE_SIZE 5

typedef struct
{
	uint8_t head;
	uint8_t tail;
	CAN_RAD_MESSAGE messages[QUEUE_SIZE];
} CAN_QUEUE;

CAN_QUEUE RxQueue = {0};
CAN_QUEUE TxQueue = {0}; //most likely don't need this


bool _isQueueEmpty(CAN_QUEUE * queue);
bool _isQueueFull(CAN_QUEUE * queue);
void _getFirstMessageFromQueue(CAN_RAD_MESSAGE * msg, CAN_QUEUE * queue);
void _addMessageToQueue(CAN_RAD_MESSAGE * msg, CAN_QUEUE * queue);
void _addHeaderAndDataToQueue(uint32_t * id, uint8_t * data, CAN_QUEUE * queue);
void _removeFirstMessageFromQueue(CAN_QUEUE * queue);


//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//
//}


bool _isQueueEmpty(CAN_QUEUE * queue)
{
    return queue->head == queue->tail;
}

bool _isQueueFull(CAN_QUEUE * queue)
{
	return queue->head == (queue->tail + 1) % QUEUE_SIZE;
}

void _getFirstMessageFromQueue(CAN_RAD_MESSAGE * msg, CAN_QUEUE * queue)
{
    memcpy(msg, &(queue->messages[queue->head]), sizeof(CAN_RAD_MESSAGE));
}

void _addMessageToQueue(CAN_RAD_MESSAGE * msg, CAN_QUEUE * queue)
{

	memcpy(&(queue->messages[queue->tail]), msg, sizeof(CAN_RAD_MESSAGE));
	queue->tail = (queue->tail + 1) % QUEUE_SIZE;
}

void _addHeaderAndDataToQueue(uint32_t * id, uint8_t * data, CAN_QUEUE * queue)
{

	memcpy(&(queue->messages[queue->tail].id), id, sizeof(CAN_RAD_IDENTIFIER));
	memcpy(&(queue->messages[queue->tail].data), data, CAN_MESSAGE_DATA_LENGTH * sizeof(uint8_t));

	queue->tail = (queue->tail + 1) % QUEUE_SIZE;

}

void _removeFirstMessageFromQueue(CAN_QUEUE * queue)
{

    memset(&(queue->messages[queue->head]), 0, sizeof(CAN_RAD_MESSAGE));
    queue->head = (queue->head + 1) % QUEUE_SIZE;
}


CAN_ERROR CAN_init()
{

	//apply filter masks based off RAD_BOARD_ID

	return CAN_OK;
}

CAN_ERROR CAN_addMessage(uint32_t * id, uint8_t * data)
{
	if (_isQueueFull(&RxQueue))
	{
		return CAN_ERROR_MAX_MESSAGES;
	}

	_addHeaderAndDataToQueue(id, data, &RxQueue);

	return CAN_OK;
}

CAN_ERROR CAN_getMessage(CAN_RAD_MESSAGE * msg)
{

	if (_isQueueEmpty(&RxQueue))
	{
		return CAN_ERROR_NO_MESSAGE;
	}

    _getFirstMessageFromQueue(msg, &RxQueue);

    _removeFirstMessageFromQueue(&RxQueue);

    return CAN_OK;
}

CAN_ERROR CAN_sendMessage(CAN_RAD_MESSAGE * msg)
{

	if (_isQueueFull(&TxQueue))
	{
		return CAN_ERROR_MAX_MESSAGES;
	}

	_addMessageToQueue(msg, &TxQueue);

	//send mechanism needed

	return CAN_OK;
}

