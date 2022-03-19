#ifndef ZEPHYR_INCLUDE_MGMT_SMP_PIPE_H_
#define ZEPHYR_INCLUDE_MGMT_SMP_PIPE_H_

#ifdef __cplusplus
extern "C" {
#endif

int smp_pipe_rx(unsigned char *buf, unsigned int len);
int smp_pipe_set_tx(int (*t)(unsigned char * m, unsigned int s));

#ifdef __cplusplus
}
#endif

#endif // ZEPHYR_INCLUDE_MGMT_SMP_PIPE_H_
