#ifndef __CONTROLLER_MPC_H__
#define __CONTROLLER_MPC_H__

#include "stabilizer_types.h"

void controllerMpcInit(void);
bool controllerMpcTest(void);
void controllerMpcReset(void);
void controllerMpc(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif //__CONTROLLER_MPC_H__
