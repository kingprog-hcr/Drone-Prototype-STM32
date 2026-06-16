/**
 *******************************************************************************
 * @file    drone.c
 * @brief   Implementation de la machine a etats du drone
 *******************************************************************************
 */

#include "drone.h"
#include "config.h"
#include "stm32g4_mpu6050.h"
#include "stm32g4_hcsr04.h"
#include "stm32g4_uart.h"
#include "stm32g4_gpio.h"
#include "motors.h"
#include <stdio.h>
#include <stdlib.h> // Pour abs()

/* Variables globales privees */
static drone_state_e current_state = INIT;
static MPU6050_t h_mpu;
static MPU6050_Result_t status;
static uint8_t id_ultrason;

// Variables pour le clignotement de la LED Jaune
static uint32_t last_led_tick = 0;
static uint8_t led_state = 0;

// Constantes et variables pour la rampe (Echelle 0 à 1000)
static int16_t takeoff_speed = 0;
#define MOTOR_ARM_SPEED     300
#define HOVER_SPEED_APPROX  750
#define TAKEOFF_RAMP_STEP   4

#define FILTER_SIZE  10
static int16_t buffer_X[FILTER_SIZE] = {0};
static int16_t buffer_Y[FILTER_SIZE] = {0};
static uint8_t filter_index = 0;

// Variables pour l'animation des LEDs en TAKEOFF
static uint32_t last_takeoff_led_tick = 0;
static uint8_t takeoff_led_step = 0;
#define TAKEOFF_LED_DELAY 150 // Vitesse de la chenille en millisecondes


static uint16_t target_altitude_mm = 400; // Altitude de consigne par défaut
static void Lightning_LED(void);
static void Filter_IMU(int16_t raw_x, int16_t raw_y, int16_t *mean_x, int16_t *mean_y);
// Fonction de filtrage par moyenne glissante
static void Filter_IMU(int16_t raw_x, int16_t raw_y, int16_t *mean_x, int16_t *mean_y)
{
    buffer_X[filter_index] = raw_x;
    buffer_Y[filter_index] = raw_y;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    int32_t sum_x = 0;
    int32_t sum_y = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum_x += buffer_X[i];
        sum_y += buffer_Y[i];
    }
    *mean_x = (int16_t)(sum_x / FILTER_SIZE);
    *mean_y = (int16_t)(sum_y / FILTER_SIZE);
}

void Drone_Init(void)
{
    // 1. Initialisation de l'IMU (MPU6050)
    status = MPU6050_Init(&h_mpu, NULL, 0, MPU6050_Device_0, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_2000s);
    if (status == MPU6050_Result_Ok) {
        printf("[DRONE] IMU Initialisée.\n");
    }

    // 2. Initialisation de l'Ultrason (Trigger: PB3, Echo: PA7)
    if (BSP_HCSR04_add(&id_ultrason, GPIOB, GPIO_PIN_3, GPIOA, GPIO_PIN_7) == HAL_OK) {
        printf("[DRONE] Ultrason Initialisé.\n");
    }

    // 3. Initialisation des moteurs
    Motors_Init();

    // 4. Configuration des broches GPIO
    BSP_GPIO_pin_config(LED_V_PORT, LED_V_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    BSP_GPIO_pin_config(LED_J_PORT, LED_J_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    BSP_GPIO_pin_config(LED_B_PORT, LED_B_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    BSP_GPIO_pin_config(BUZZER_PORT, BUZZER_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    current_state = INIT;
}

void Drone_Process(void)
{
    uint16_t distance_h_mm = 0;
    uint8_t command = 0;

    // Variables pour stocker les valeurs filtrées
    int16_t filtered_X = 0;
    int16_t filtered_Y = 0;

    // Taches de fond obligatoires pour les capteurs
    BSP_HCSR04_process_main();
    MPU6050_ReadAll(&h_mpu);

    BSP_HCSR04_get_value(id_ultrason, &distance_h_mm);


    Filter_IMU(h_mpu.Accelerometer_X, h_mpu.Accelerometer_Y, &filtered_X, &filtered_Y);

    // Lecture Bluetooth non bloquante
    if (BSP_UART_data_ready(UART1_ID))
    {
        command = BSP_UART_getc(UART1_ID);

        if (command == 'E') {
            current_state = STATE_EMERGENCY;
            printf("[CRITICAL] EMERGENCY ! Arrêt d'urgence activé.\n");
        }
    }

    switch (current_state)
    {
        case INIT: // ① INIT
        	target_altitude_mm = 400;
            Motors_SetSpeed(0, 0);
            Motors_SetSpeed(1, 0);
            Motors_SetSpeed(2, 0);
            Motors_SetSpeed(3, 0);

            HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

            if (command == 'A') {
                current_state = STATE_ARMING;
                last_led_tick = HAL_GetTick();
                led_state = 1;
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
                printf("[STATE] INIT -> ARMING\n");
            }
            break;

        case STATE_ARMING: // ② ARMING
            HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);

            // 1. Gestion du clignotement asymétrique de la LED Jaune
            uint32_t current_tick = HAL_GetTick();
            if (led_state == 1 && (current_tick - last_led_tick >= 150)) {
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
                led_state = 0;
                last_led_tick = current_tick;
            }
            else if (led_state == 0 && (current_tick - last_led_tick >= 500)) {
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
                led_state = 1;
                last_led_tick = current_tick;
            }

            // 2. Vérification IMU en utilisant les VALEURS FILTRÉES à la place des brutes
            if (abs((int)filtered_X) > 750 || abs((int)filtered_Y) > 750) {
                printf("[ARMING ERROR] Drone incliné ! (Filtré X:%d, Y:%d) Retour à INIT.\n", filtered_X, filtered_Y);
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
                HAL_Delay(300);
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
                current_state = INIT;
                break;
            }

            // 3. Test des moteurs à bas régime
            Motors_SetSpeed(0, MOTOR_ARM_SPEED);
            Motors_SetSpeed(1, -MOTOR_ARM_SPEED);
            Motors_SetSpeed(2, MOTOR_ARM_SPEED);
            Motors_SetSpeed(3, -MOTOR_ARM_SPEED);

            if (command == 'T') {
                current_state = STATE_TAKEOFF;
                takeoff_speed = MOTOR_ARM_SPEED;
                printf("[STATE] ARMING -> TAKEOFF\n");
            }
            else if (command == 'I') {
                current_state = INIT;
                printf("[STATE] ARMING -> Retour à INIT\n");
            }
            break;

        case STATE_TAKEOFF: // ③ TAKEOFF
                    // 1. Gestion de la rampe moteur
                    if (takeoff_speed < HOVER_SPEED_APPROX) {
                        takeoff_speed += TAKEOFF_RAMP_STEP;
                    }

                    Motors_SetSpeed(0, takeoff_speed);
                    Motors_SetSpeed(1, -takeoff_speed);
                    Motors_SetSpeed(2, takeoff_speed);
                    Motors_SetSpeed(3, -takeoff_speed);

                    // 2. ANIMATION CHENILLE DES LEDS (Non bloquante)

                    Lightning_LED();

                    // 3. Condition de transition vers le vol stationnaire
                    if (distance_h_mm > 0 && distance_h_mm >= target_altitude_mm) {
                        current_state = STATE_FLIGHT;
                        HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
                        printf("[STATE] TAKEOFF -> FLY ! Altitude stable à %d mm\n", distance_h_mm);
                    }
                    break;

        case STATE_FLIGHT: // ④ FLY
                    // 1. Gestion des commandes Bluetooth spécifiques au vol
                    if (command == '+') {
                        target_altitude_mm += 50; // Augmente la consigne de 5 cm
                        if (target_altitude_mm > 1000) target_altitude_mm = 1000; // Max 1 mètre
                        printf("[FLIGHT] Nouvelle consigne : %d mm\n", target_altitude_mm);
                    }
                    else if (command == '-') {
                        target_altitude_mm -= 50; // Diminue la consigne de 5 cm
                        if (target_altitude_mm < 200) target_altitude_mm = 200; // Min 20 cm
                        printf("[FLIGHT] Nouvelle consigne : %d mm\n", target_altitude_mm);
                    }
                    else if (command == 'L') {
                        current_state = STATE_LANDING;
                        // On initialise la vitesse de descente avec la dernière vitesse calculée
                        printf("[STATE] FLY -> LANDING\n");
                        break;
                    }

                    // 2. Sécurité IMU (Fail-safe d'inclinaison critique en vol)
                    if (abs((int)filtered_X) > 1500 || abs((int)filtered_Y) > 1500) {
                        current_state = STATE_EMERGENCY;
                        printf("[CRITICAL] Drone instable en vol (X:%d, Y:%d) -> EMERGENCY\n", filtered_X, filtered_Y);
                        break;
                    }

                    // 3. Asservissement proportionnel simplifié pour l'altitude
                    int16_t flight_speed = HOVER_SPEED_APPROX;
                    if (distance_h_mm > 0) { // On vérifie que la mesure ultrason est valide
                        int16_t error = (int16_t)target_altitude_mm - (int16_t)distance_h_mm;

                        // Gain proportionnel (ajustable selon la réactivité de vos moteurs)
                        // Par exemple, 1 unité de vitesse moteur en plus par millimètre d'erreur
                        int16_t correction = error * 1;

                        // Limitation de la correction pour éviter les coups de gaz violents
                        if (correction > 100)  correction = 100;
                        if (correction < -100) correction = -100;

                        flight_speed = HOVER_SPEED_APPROX + correction;
                    }

                    // Application de la vitesse calculée aux moteurs
                    Motors_SetSpeed(0, flight_speed);
                    Motors_SetSpeed(1, -flight_speed);
                    Motors_SetSpeed(2, flight_speed);
                    Motors_SetSpeed(3, -flight_speed);

                    // Mémorisation pour l'état LANDING pour une transition fluide
                    takeoff_speed = flight_speed;

                    // 4. Gestion des alertes de proximité (Sol trop proche)
                    if (distance_h_mm > 0 && distance_h_mm < 150) {
                        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
                    } else {
                        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
                    }

                    // LEDs de statut de vol : Fixes pour indiquer la stabilité
                    HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
                    break;

        case STATE_LANDING: // ⑤ LANDING
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

            if (takeoff_speed > 0) {
                takeoff_speed -= TAKEOFF_RAMP_STEP;
            }

            Motors_SetSpeed(0, takeoff_speed);
            Motors_SetSpeed(1, -takeoff_speed);
            Motors_SetSpeed(2, takeoff_speed);
            Motors_SetSpeed(3, -takeoff_speed);

            if (distance_h_mm > 0 && distance_h_mm <= 40) {
                current_state = INIT;
                printf("[STATE] LANDING -> INIT (Drone Posé)\n");
            }
            Lightning_LED();
            break;

        case STATE_EMERGENCY: // ⑥ EMERGENCY
            Motors_SetSpeed(0, 0);
            Motors_SetSpeed(1, 0);
            Motors_SetSpeed(2, 0);
            Motors_SetSpeed(3, 0);

            HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);

            if (command == 'R') {
                current_state = INIT;
                printf("[EMERGENCY] Reset validé. Retour à INIT.\n");
            }
            break;
    }

    static uint32_t last_ultrasonic_tick = 0;
        if (HAL_GetTick() - last_ultrasonic_tick >= 60) { // Le HC-SR04 a besoin d'environ 60ms entre deux mesures pour éviter les échos fantômes
            BSP_HCSR04_run_measure(id_ultrason);
            last_ultrasonic_tick = HAL_GetTick();
        }
}



static void Lightning_LED(void){
	uint32_t tk_tick = HAL_GetTick();
	                    if (tk_tick - last_takeoff_led_tick >= TAKEOFF_LED_DELAY) {
	                        last_takeoff_led_tick = tk_tick;
	                        takeoff_led_step = (takeoff_led_step + 1) % 6; // Boucle sur 6 étapes

	                        switch (takeoff_led_step) {
	                            case 0: // Étape 1 : On commence par allumer la Verte
	                                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
	                                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
	                                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
	                                break;
	                            case 1: // Étape 2 : On ajoute la Jaune
	                                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
	                                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
	                                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
	                                break;
	                            case 2: // Étape 3 : Tout est allumé (on ajoute la Bleue)
	                                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
	                                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
	                                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
	                                break;
	                            case 3: // Étape 4 : On commence à éteindre la Verte
	                                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);
	                                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
	                                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
	                                break;
	                            case 4: // Étape 5 : On éteint la Jaune
	                                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);
	                                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
	                                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
	                                break;
	                            case 5: // Étape 6 : Tout est éteint (on éteint la Bleue)
	                                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);
	                                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
	                                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
	                                break;
	                        }
	                    }
}
