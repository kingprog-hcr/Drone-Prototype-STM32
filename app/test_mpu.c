/**
 *******************************************************************************
 * @file    test_mpu.c
 * @brief   Implémentation du test unitaire du MPU6050
 *******************************************************************************
 */

#include "test_mpu.h"
#include "config.h"
#include "stm32g4_mpu6050.h"
#include "stm32g4_uart.h"
#include "stm32g4_utils.h"
#include <stdio.h>

void test_MPU6050_run(void)
{
    MPU6050_t MPU_Drone;
    MPU6050_Result_t status;

    printf("     LANCEMENT DU TEST UNITAIRE MPU6050     \n");


    // Initialisation du MPU (NULL, 0 car alimentation directe sur le 3V3 du PCB)
    status = MPU6050_Init(&MPU_Drone, NULL, 0, MPU6050_Device_0, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_2000s);

    if (status == MPU6050_Result_Ok) {
        printf("[SUCCESS] MPU6050 detecte et initialise avec succes.\n\n");
    } else {
        printf("[ERROR] Echec de l'initialisation du MPU6050. Code : %d\n", status);
        printf("Verifiez le cablage (SDA:PB5, SCL:PA8, Vcc:3V3, GND:GND)\n");
        while (1); // On bloque ici si le capteur n'est pas trouvé
    }

    /* Boucle locale de test */
    while (1)
    {
        if (MPU6050_ReadAll(&MPU_Drone) == MPU6050_Result_Ok)
        {
            printf("ACCEL [X:%5d Y:%5d Z:%5d] | GYRO [X:%5d Y:%5d Z:%5d] | T:%.1f C\r",
                    MPU_Drone.Accelerometer_X,
                    MPU_Drone.Accelerometer_Y,
                    MPU_Drone.Accelerometer_Z,
                    MPU_Drone.Gyroscope_X,
                    MPU_Drone.Gyroscope_Y,
                    MPU_Drone.Gyroscope_Z,
                    MPU_Drone.Temperature);
        }
        else
        {
            printf("[ERROR] Erreur de lecture I2C...\n");
        }

        HAL_Delay(200); // Rafraîchissement toutes les 200ms
    }
}
