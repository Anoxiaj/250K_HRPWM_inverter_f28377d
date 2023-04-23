#ifndef CONTROL_INCLUDE_MAIN_CONTROL_H_
#define CONTROL_INCLUDE_MAIN_CONTROL_H_



#ifdef __cplusplus
extern "C" {
#endif

void mainDataInit(void);
__interrupt void MainControlISR(void);



#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
