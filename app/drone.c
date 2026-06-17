/**
 *******************************************************************************
 * @file    drone.c
 * @brief   Implémentation ultra-commentée de la machine à états du drone.
 * Gère exclusivement la stabilisation active et le maintien d'altitude.
 * CONFIGURATION GÉOMÉTRIQUE :
 * M0 = Arrière-Droit
 * M1 = Arrière-Gauche
 * M2 = Avant-Gauche
 * M3 = Avant-Droit
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
#include <stdlib.h>

//VARIABLES GLOBALES PRIVÉES (CONTEXTE DE LA MACHINE)
static drone_state_e current_state = INIT; // Variable d'état de la machine
static MPU6050_t h_mpu;                    // Structure de configuration et de stockage des données de l'IMU MPU6050
static MPU6050_Result_t status;            // Variable de retour pour stocker le code d'erreur de l'initialisation IMU
static uint8_t id_ultrason;                // Identifiant unique attribué au capteur de distance ultrason par le BSP

// Variables temporelles et d'état pour le clignotement asynchrone de la LED Jaune en ARMING
static uint32_t last_led_tick = 0;         // Stocke le dernier temps système (HAL_GetTick) du changement d'état LED
static uint8_t led_state = 0;              // Variable d'état booléenne pour la LED (0 = éteinte, 1 = allumée)

// Variables de consigne de vitesse brute calculées pour les moteurs
static int16_t takeoff_speed = 0;          // Vitesse courante injectée pendant la rampe de décollage
static int16_t flight_speed = 0;           // Vitesse globale ajustée en vol par le correcteur d'altitude

// Tableaux de buffers circulaires pour stocker les N dernières mesures brutes de l'IMU (Moyenne Glissante)
static int16_t buffer_X[FILTER_SIZE] = {0}; // Buffer pour l'axe de Roulis (X)
static int16_t buffer_Y[FILTER_SIZE] = {0}; // Buffer pour l'axe de Tangage (Y)
static uint8_t filter_index = 0;           // index pointant sur la case courante à écraser dans les buffers

// Variables pour cadencer l'animation visuelle des LEDs en phase TAKEOFF
static uint32_t last_takeoff_led_tick = 0; // Garde en mémoire le dernier tick système de l'animation de décollage
static uint8_t takeoff_led_step = 0;       // Compteur d'étape de l'animation de décollage (de 0 à 5)

// Variables de consigne pour l'altitude du drone
static uint16_t target_altitude_mm = 300;  // Altitude fixe cible en millimètres (30 cm)

// PROTOTYPES DES FONCTIONS SÉCURISÉES ET LOCALES
static void Lightning_LED(void);
static void Filter_IMU(int16_t raw_x, int16_t raw_y, int16_t *mean_x, int16_t *mean_y);
static void Stabilize_Drone(int16_t filtered_x, int16_t filtered_y, int16_t base_speed);

/**
 * @brief Initialisation matérielle des composants du drone
 */
void Drone_Init(void)
{
    // 1. Initialisation de l'IMU (MPU6050) en I2C
    status = MPU6050_Init(&h_mpu, NULL, 0, MPU6050_Device_0, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_2000s);
    if (status == MPU6050_Result_Ok) {
        printf("[DRONE] IMU Initialisée.\n");
    }

    // 2. Initialisation du capteur ultrason HC-SR04
    if (BSP_HCSR04_add(&id_ultrason, GPIOB, GPIO_PIN_3, GPIOA, GPIO_PIN_7) == HAL_OK) {
        printf("[DRONE] Ultrason Initialisé.\n");
    }

    // 3. Initialisation des signaux PWM matériels
    Motors_Init();

    // 4. Configuration des broches GPIO pour les LEDs et le Buzzer
    BSP_GPIO_pin_config(LED_V_PORT, LED_V_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    BSP_GPIO_pin_config(LED_J_PORT, LED_J_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    BSP_GPIO_pin_config(LED_B_PORT, LED_B_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    BSP_GPIO_pin_config(BUZZER_PORT, BUZZER_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    current_state = INIT;
}

/**
 * @brief Fonction principale contenant la Machine à états
 */
void Drone_Process(void)
{
    uint16_t distance_h_mm = 0;
    uint8_t command = 0;

    int16_t filtered_X = 0;
    int16_t filtered_Y = 0;

    // TÂCHES ASYNCHRONES DE GESTION DES CAPTEURS
    BSP_HCSR04_process_main();
    MPU6050_ReadAll(&h_mpu);
    BSP_HCSR04_get_value(id_ultrason, &distance_h_mm);

    // Filtrage numérique
    Filter_IMU(h_mpu.Accelerometer_X, h_mpu.Accelerometer_Y, &filtered_X, &filtered_Y);

    // Réception Bluetooth
    if (BSP_UART_data_ready(UART1_ID))
    {
        command = BSP_UART_getc(UART1_ID);

        if (command == 'E') {
            current_state = STATE_EMERGENCY;
            printf("[CRITICAL] EMERGENCY ! Arrêt d'urgence activé.\n");
        }
    }

    // MACHINE À ÉTATS PRINCIPALE
    switch (current_state)
    {
        case INIT:
            target_altitude_mm = 300;
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

        case STATE_ARMING:
            HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);

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

            if (abs((int)filtered_X) > 750 || abs((int)filtered_Y) > 750) {
                printf("[ARMING ERROR] Drone incliné ! Retour à INIT.\n");
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
                HAL_Delay(300);
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
                current_state = INIT;
                break;
            }

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

        case STATE_TAKEOFF:
            {
                static uint32_t last_ramp_tick = 0;
                uint32_t current_takeoff_tick = HAL_GetTick();

                if (current_takeoff_tick - last_ramp_tick >= 35) {
                   if (takeoff_speed < HOVER_SPEED_APPROX) {
                       takeoff_speed += TAKEOFF_RAMP_STEP;
                   }
                   last_ramp_tick = current_takeoff_tick;
                }

                Stabilize_Drone(filtered_X, filtered_Y, takeoff_speed);
                Lightning_LED();

                if (distance_h_mm > 0 && distance_h_mm >= target_altitude_mm) {
                    current_state = STATE_FLIGHT;
                    flight_speed = takeoff_speed;
                    printf("[STATE] TAKEOFF -> FLY ! Altitude stable à %d mm\n", distance_h_mm);
                }
            }
            break;

        case STATE_FLIGHT:
            if (command == '+') {
                target_altitude_mm += 50;
                if (target_altitude_mm > 1000) target_altitude_mm = 1000;
                printf("[FLIGHT] Altitude cible : %d mm\n", target_altitude_mm);
            }
            else if (command == '-') {
                target_altitude_mm -= 50;
                if (target_altitude_mm < 200) target_altitude_mm = 200;
                printf("[FLIGHT] Altitude cible : %d mm\n", target_altitude_mm);
            }
            else if (command == 'L') {
                current_state = STATE_LANDING;
                printf("[STATE] FLY -> LANDING\n");
                break;
            }

            if (distance_h_mm > 0) {
                int16_t error = (int16_t)target_altitude_mm - (int16_t)distance_h_mm;
                int16_t correction = error;

                if (correction > 100)  correction = 100;
                if (correction < -100) correction = -100;

                flight_speed = HOVER_SPEED_APPROX + correction;
            }

            Stabilize_Drone(filtered_X, filtered_Y, flight_speed);
            takeoff_speed = flight_speed;

            if (distance_h_mm > 0 && distance_h_mm < 150) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            }

            HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
            break;

        case STATE_LANDING:
            {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
                uint32_t current_landing_tick = HAL_GetTick();
                static uint32_t last_landing_ramp_tick = 0;

                if (current_landing_tick - last_landing_ramp_tick >= 350) {
                   if (takeoff_speed > 0) {
                       takeoff_speed -= TAKEOFF_RAMP_STEP;
                       if (takeoff_speed < 0) takeoff_speed = 0;
                   }
                   last_landing_ramp_tick = current_landing_tick;
                }

                Stabilize_Drone(filtered_X, filtered_Y, takeoff_speed);
                Lightning_LED();

                if ((distance_h_mm > 0 && distance_h_mm <= 40) || takeoff_speed == 0) {
                    current_state = INIT;
                    printf("[STATE] LANDING -> INIT (Drone Posé)\n");
                }
            }
            break;

        case STATE_EMERGENCY:
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
    if (HAL_GetTick() - last_ultrasonic_tick >= 60) {
        BSP_HCSR04_run_measure(id_ultrason);
        last_ultrasonic_tick = HAL_GetTick();
    }
}

/**
 * @brief  Gestion interne de l'animation des LEDs
 */
static void Lightning_LED(void){
    uint32_t tk_tick = HAL_GetTick();
    if (tk_tick - last_takeoff_led_tick >= TAKEOFF_LED_DELAY) {
        last_takeoff_led_tick = tk_tick;
        takeoff_led_step = (takeoff_led_step + 1) % 6;

        switch (takeoff_led_step) {
            case 0:
                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
                break;
            case 1:
                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
                break;
            case 2:
                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
                break;
            case 3:
                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
                break;
            case 4:
                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
                break;
            case 5:
                HAL_GPIO_WritePin(LED_V_PORT, LED_V_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_J_PORT, LED_J_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
                break;
        }
    }
}

/**
 * @brief  Filtre numérique passe-bas à moyenne glissante
 */
static void Filter_IMU(int16_t raw_x, int16_t raw_y, int16_t *mean_x, int16_t *mean_y)
{
    buffer_X[filter_index] = raw_x;
    buffer_Y[filter_index] = raw_y;

    filter_index = (filter_index + 1) % FILTER_SIZE;

    int32_t sum_x = 0;
    int32_t sum_y = 0;

    for (int8_t i = 0; i < FILTER_SIZE; i++) {
        sum_x += buffer_X[i];
        sum_y += buffer_Y[i];
    }

    // Calcul de la moyenne arithmétique
    *mean_x = (int16_t)(sum_x / FILTER_SIZE);
    *mean_y = (int16_t)(sum_y / FILTER_SIZE);
}

/**
 * @brief  Stabilisation auto
 */
static void Stabilize_Drone(int16_t filtered_x, int16_t filtered_y, int16_t base_speed)
{
    int16_t Kp_x = 3;
    int16_t Kp_y = 3;

    int16_t correction_x = (filtered_x / 10) * Kp_x;
    int16_t correction_y = (filtered_y / 10) * Kp_y;

    if (correction_x > 150)  correction_x = 150;
    if (correction_x < -150) correction_x = -150;
    if (correction_y > 150)  correction_y = 150;
    if (correction_y < -150) correction_y = -150;

    int16_t m0 =   base_speed - correction_x + correction_y;
    int16_t m1 = -(base_speed + correction_x + correction_y);
    int16_t m2 = base_speed + correction_x - correction_y;
    int16_t m3 =   -(base_speed - correction_x - correction_y);

    Motors_SetSpeed(0, m0);
    Motors_SetSpeed(1, m1);
    Motors_SetSpeed(2, m2);
    Motors_SetSpeed(3, m3);
}
