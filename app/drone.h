/**
 *******************************************************************************
 * @file    drone.h
 * @brief   Déclarations pour la machine à états principale du drone
 *******************************************************************************
 */

#ifndef DRONE_H_
#define DRONE_H_

#include "stm32g4xx_hal.h"

/* Énumération des états du drone (selon votre cahier des charges) */
typedef enum
{
    INIT,
    STATE_ARMING,
    STATE_TAKEOFF,
    STATE_FLIGHT,
    STATE_LANDING,
    STATE_EMERGENCY
} drone_state_e;

/**
 * @brief  Initialise les structures du drone
 */
void Drone_Init(void);

/**
 * @brief  Fonction principale de contrôle du drone (à appeler dans le while(1) du main)
 */
void Drone_Process(void);

#endif /* DRONE_H_ */
