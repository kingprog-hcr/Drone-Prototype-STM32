/**
 *******************************************************************************
 * @file    test_ultrason.c
 * @brief   Implémentation du test unitaire de l'ultrason HC-SR04
 *******************************************************************************
 */

#include "test_ultrason.h"
#include "config.h"
#include "stm32g4_hcsr04.h"
#include "stm32g4_uart.h"
#include "stm32g4_utils.h"
#include <stdio.h>

void test_Ultrason_run(void)
{
    uint8_t id_ultrason;
    uint16_t distance_mm = 0;
    uint32_t t_last_measure = 0;


    printf("     LANCEMENT DU TEST UNITAIRE ULTRASON    \n");


    /* * Ajout du capteur ultrason avec les broches :
         * - Trigger : PB3 (GPIOB, GPIO_PIN_3)
         * - Echo    : PA7 (GPIOA, GPIO_PIN_7)
         */
        if (BSP_HCSR04_add(&id_ultrason, GPIOB, GPIO_PIN_3, GPIOA, GPIO_PIN_7) == HAL_OK) {
            printf("[SUCCESS] Capteur ultrason ajoute (Trig:PB3, Echo:PA7) - ID : %d\n", id_ultrason);
        } else {
            printf("[ERROR] Impossible d'ajouter le capteur ultrason.\n");
            while(1);
        }


    t_last_measure = HAL_GetTick();

    /* Boucle de test */
    while (1)
    {

        BSP_HCSR04_process_main();

        // Déclencher une mesure toutes les 60 ms (période standard pour un HC-SR04)
        if (HAL_GetTick() - t_last_measure >= 60)
        {
            BSP_HCSR04_run_measure(id_ultrason);
            t_last_measure = HAL_GetTick();
        }

        // 1. Récupérer et afficher la valeur dès qu'elle est prête
        switch (BSP_HCSR04_get_value(id_ultrason, &distance_mm))
        {
            case HAL_OK:
                // Affichage en cm (\r pour réécrire sur la même ligne)
                printf("Distance mesurée : %3d cm\r", distance_mm / 10);
                break;

            case HAL_TIMEOUT:
                printf("[WARN] Timeout : Aucun obstacle détecté à portée.\r");
                break;

            case HAL_ERROR:
                printf("[ERROR] Erreur interne du capteur.\r");
                break;

            case HAL_BUSY:
                // La mesure est en cours (l'onde sonore voyage), on attend le prochain tour
                break;
        }
    }
}
