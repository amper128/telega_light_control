/**
 * @file can.c
 * @author Алексей Хохлов <root@amper.me>
 * @copyright WTFPL License
 * @date 2022
 * @brief Функции работы с CAN
 */

#include <can.h>
#include <main.h>
#include <stdbool.h>

#include <stm32f1xx_hal_can.h>

#define CAN_IDE_32 (0x4U) /* 0b00000100 */

#define MSGS_QUEUE_SIZE (32U)

/**
 * @brief Дескриптор CAN
 */
static CAN_HandleTypeDef can;

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan);

void USB_HP_CAN_TX_IRQHandler(void);
void USB_LP_CAN_RX0_IRQHandler(void);
void CAN_RX1_IRQHandler(void);
void CAN_SCE_IRQHandler(void);

static CAN_TxHeaderTypeDef TxHeader;
static CAN_RxHeaderTypeDef RxHeader;
static uint32_t TxMailbox;
static uint8_t my_address;

static can_packet_t rx_messages[MSGS_QUEUE_SIZE];
static can_packet_t tx_messages[MSGS_QUEUE_SIZE];

static uint32_t rx_head = 0u;
static uint32_t rx_tail = 0u;

static uint32_t tx_head = 0u;
static uint32_t tx_tail = 0u;

void
can_start(void)
{
	HAL_CAN_Start(&can);
}

static inline bool
can_ready_tx(void)
{
	uint32_t tsr = READ_REG(can.Instance->TSR);
	bool result = false;

	if (((tsr & CAN_TSR_TME0) != 0U) || ((tsr & CAN_TSR_TME1) != 0U) ||
	    ((tsr & CAN_TSR_TME2) != 0U)) {
		result = true;
	}

	return result;
}

static inline void
add_msg(can_hdr_t *hdr, uint8_t len, uint8_t data[])
{
	can_packet_t *add = &tx_messages[tx_head % MSGS_QUEUE_SIZE];

	add->hdr.cmd = hdr->cmd;
	add->hdr.id = hdr->id;
	add->len = len;

	union {
		uint8_t *u8;
		uint8_t *u32;
	} dest, src;

	/* memcpy */
	dest.u8 = add->data;
	src.u8 = data;
	dest.u32[0] = src.u32[0];
	dest.u32[1] = src.u32[1];

	tx_head++;
}

void
can_work(void)
{
	/* исходящая очередь */
	if (tx_head != tx_tail) {
		can_packet_t *msg = &tx_messages[tx_tail % MSGS_QUEUE_SIZE];

		if (can_ready_tx()) {
			union {
				can_hdr_t *hdr;
				uint32_t *u32;
			} id;
			id.hdr = &msg->hdr;

			TxHeader.ExtId = *id.u32;
			TxHeader.DLC = msg->len;

			if (HAL_CAN_AddTxMessage(&can, &TxHeader, msg->data,
						 &TxMailbox) != HAL_OK) {
				/* Transmission request Error */
				/* FIXME: error statistics */
				Error_Handler();
			}

			tx_tail++;
		}
	}

	/* входящая очередь */
	if (rx_head != rx_tail) {
		can_packet_t *p;
		p = &rx_messages[rx_tail % MSGS_QUEUE_SIZE];

		/*printf("can msg: addr 0x%02X, cmd 0x%02X, len %u\r\n",
		       p->hdr.id, p->hdr.cmd, (uint8_t)RxHeader.DLC);*/

		parse_can_msg(p);
		rx_tail++;
	}
}

void
can_send(uint8_t cmd, uint8_t len, uint8_t data[])
{
	can_hdr_t hdr;

	hdr.id = my_address;
	hdr.cmd = cmd;

	add_msg(&hdr, len, data);
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
void
can_init(uint8_t address)
{
	can.Instance = CAN1;
	can.Init.Prescaler = 4;
	can.Init.Mode = CAN_MODE_NORMAL;
	can.Init.SyncJumpWidth = CAN_SJW_1TQ;
	can.Init.TimeSeg1 = CAN_BS1_9TQ;
	can.Init.TimeSeg2 = CAN_BS2_8TQ;

	can.Init.TimeTriggeredMode = DISABLE;
	can.Init.AutoBusOff = DISABLE;
	can.Init.AutoWakeUp = DISABLE;
	can.Init.AutoRetransmission = ENABLE;
	can.Init.ReceiveFifoLocked = DISABLE;
	can.Init.TransmitFifoPriority = DISABLE;

	if (HAL_CAN_Init(&can) == HAL_OK) {
		CAN_FilterTypeDef canFilterConfig;

		union {
			can_hdr_t hdr;
			uint32_t u32;
		} id_mask;

		id_mask.hdr.id = address;
		id_mask.hdr.cmd = 0U;
		id_mask.hdr.__reserved[0] = 0U;
		id_mask.hdr.__reserved[1] = 0U;

		/* сохраняем адресный id для дальнейшего использования */
		my_address = address;

		/* основной фильтр сообщений для своего адреса */
		canFilterConfig.FilterBank = 0;
		canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		canFilterConfig.FilterIdHigh = (id_mask.u32 >> 13U);
		canFilterConfig.FilterIdLow = (id_mask.u32 << 3U) | CAN_IDE_32;

		/* маска адреса - должны совпадать все биты */
		id_mask.u32 = 0U;
		id_mask.hdr.id = 0xFFU;

		canFilterConfig.FilterMaskIdHigh =
		    (uint16_t)(id_mask.u32 >> 13U);
		canFilterConfig.FilterMaskIdLow =
		    (uint16_t)(id_mask.u32 << 3U) | CAN_IDE_32;

		canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		canFilterConfig.FilterActivation = ENABLE;
		canFilterConfig.SlaveStartFilterBank = 1;

		HAL_CAN_ConfigFilter(&can, &canFilterConfig);

		// printf("CAN init ok\r\n");
	} else {
		Error_Handler();
	}

	/* Activate CAN RX notification */
	if (HAL_CAN_ActivateNotification(&can, CAN_IT_RX_FIFO0_MSG_PENDING) !=
	    HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}

	TxHeader.StdId = 0x100;
	TxHeader.ExtId = 0x01;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 2;
	TxHeader.TransmitGlobalTime = DISABLE;
}

/**
 * @brief CAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void
HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (hcan->Instance == CAN1) {
		/* Peripheral clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN GPIO Configuration
		   PB8     ------> CAN_RX
		   PB9     ------> CAN_TX
		   */
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		__HAL_AFIO_REMAP_CAN1_2();

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
		HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
	}
}

/**
 * @brief CAN MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void
HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{

	if (hcan->Instance == CAN1) {
		/* Peripheral clock disable */
		__HAL_RCC_CAN1_CLK_DISABLE();

		/**CAN GPIO Configuration
		PB8     ------> CAN_RX
		PB9     ------> CAN_TX
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

		/* CAN interrupt DeInit */
		HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
		HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
	}
}

/**
 * @brief This function handles USB high priority or CAN_TX interrupts.
 */
void
USB_HP_CAN1_TX_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&can);
}

/**
 * @brief This function handles USB low priority or CAN_RX0 interrupts.
 */
void
USB_LP_CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&can);
}

/**
 * @brief This function handles CAN RX1 interrupt.
 */
void
CAN1_RX1_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&can);
	// printf("%s\r\n", __FUNCTION__);
}

/**
 * @brief This function handles CAN SCE interrupt.
 */
void
CAN1_SCE_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&can);
	// printf("%s\r\n", __FUNCTION__);
}

void
HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can_packet_t *rx_packet = &rx_messages[rx_head % MSGS_QUEUE_SIZE];
	uint8_t *p = rx_packet->data;

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, p) != HAL_OK) {
		/* Reception Error */
		Error_Handler();
	}

	union {
		can_hdr_t *msg;
		uint32_t *u32;
	} can_hdr;
	can_hdr.msg = &rx_packet->hdr;
	can_hdr.u32[0] = RxHeader.ExtId;
	rx_packet->len = (uint8_t)RxHeader.DLC;

	rx_head++;
}
