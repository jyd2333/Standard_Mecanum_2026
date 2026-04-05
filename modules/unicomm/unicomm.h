#ifndef UNICOMM_H
#define UNICOMM_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_types.h"

#define UNICOMM_FRAME_HEAD           0xFFu
#define UNICOMM_FRAME_TYPE_CTRL_CMD  0x0Du
#define UNICOMM_FRAME_TYPE_UPLOAD    0xFDu

#define UNICOMM_CTRL_FRAME_LEN       (sizeof(Chassis_Ctrl_Cmd_s_uart) + 3u)
#define UNICOMM_UPLOAD_FRAME_LEN     (sizeof(Chassis_Upload_Data_s_uart) + 3u)

uint8_t UniCommChecksum(const uint8_t *data, uint16_t len);

void UniCommFloatToBytes(float value, uint8_t *bytes);
float UniCommBytesToFloat(const uint8_t *bytes);

uint16_t UniCommPackChassisCtrl(const Chassis_Ctrl_Cmd_s_uart *src, uint8_t *dst, uint16_t dst_len);
bool UniCommUnpackChassisCtrl(const uint8_t *src, uint16_t src_len, Chassis_Ctrl_Cmd_s_uart *dst);

uint16_t UniCommPackChassisUpload(const Chassis_Upload_Data_s_uart *src, uint8_t *dst, uint16_t dst_len);
bool UniCommUnpackChassisUpload(const uint8_t *src, uint16_t src_len, Chassis_Upload_Data_s_uart *dst);

#endif // UNICOMM_H
