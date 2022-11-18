/**
 * @file can.h
 * @author Алексей Хохлов <root@amper.me>
 * @copyright WTFPL License
 * @date 2022
 * @brief Работа с CAN
 */

#pragma once

#include <stdint.h>

#include <proto/vesc_proto.h>

typedef struct {
	can_hdr_t hdr;
	uint8_t len;
	uint8_t data[8];
} can_packet_t;

/**
 * @brief Функция настройки CAN
 */
void can_init(uint8_t address);

/**
 * @brief Запуск работы CAN
 */
void can_start(void);

/**
 * @brief Основная функция обработки CAN
 */
void can_work(void);

/**
 * @brief Функция для отправки сообщения
 * @param cmd [in] - ID команды
 * @param len [in] - длина данных
 * @param data [in] - данные запроса
 */
void can_send(uint8_t cmd, uint8_t len, uint8_t data[]);

/**
 * @brief Функция для разбора принятого сообщения
 * @param msg_id [in] - заголовок сообщения
 * @param len [in] - длина данных
 * @param errcode [in] - данные сообщения
 */
void parse_can_msg(can_packet_t *packet);
