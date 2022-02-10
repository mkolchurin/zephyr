
#ifndef ZEPHYR_INCLUDE_SMP_MODBUS
#define ZEPHYR_INCLUDE_SMP_MODBUS

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int modbus_virtual_com_wr_callback(uint8_t reg);
int modbus_virtual_com_rd_callback(uint8_t *reg);

#ifdef __cplusplus
}
#endif

#endif //ZEPHYR_INCLUDE_SMP_MODBUS