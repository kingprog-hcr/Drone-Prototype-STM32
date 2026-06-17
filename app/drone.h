/**
 *******************************************************************************
 * @file    drone.h
 * @brief   Déclarations pour la machine à états principale du drone.
 * Ce fichier définit l'architecture des états et les constantes de base.
 *******************************************************************************
 */

#ifndef DRONE_H_
#define DRONE_H_

// Inclusion de la bibliothèque HAL STM32G4 pour l'accès aux registres et fonctions matérielles
#include "stm32g4xx_hal.h"

// CONSTANTES DE CONFIGURATION DU DRONE
// Vitesse PWM brute (0 à 1000) envoyée aux moteurs dès l'étape d'armement pour contrôle visuel
#define MOTOR_ARM_SPEED     300

// Vitesse de base approximative permettant d'approcher le vol stationnaire (Hover)
#define HOVER_SPEED_APPROX  750

// Pas d'incrémentation ou de décrémentation de la vitesse PWM lors des rampes automatiques
#define TAKEOFF_RAMP_STEP   4

// Nombre d'échantillons utilisés pour le filtre à moyenne glissante de l'IMU
#define FILTER_SIZE         15

/* Délai en millisecondes entre chaque étape de l'animation des LEDs lors du décollage */
#define TAKEOFF_LED_DELAY   150


// les états possibles du drone
typedef enum
{
    INIT,            // État initial : Moteurs coupés, attente de la commande d'armement
    STATE_ARMING,    // Armement : Rotation lente de sécurité et vérification de l'assiette à plat
    STATE_TAKEOFF,   // Décollage : Augmentation automatique et progressive de la puissance PWM
    STATE_FLIGHT,    // Vol : Maintien autonome de l'altitude par ultrason et de l'assiette par IMU
    STATE_LANDING,   // Atterrissage : Diminution progressive de la poussée jusqu'au contact sol
    STATE_EMERGENCY  // Urgence : Coupure immédiate et absolue de tous les moteurs
} drone_state_e;

// PROTOTYPES DES FONCTIONS PUBLIQUES
/**
 * @brief  Initialise l'ensemble des modules matériels liés au drone (IMU, Ultrason, GPIO, PWM).
 */
void Drone_Init(void);

/**
 * @brief  Fonction principale de traitement de la machine à états (à appeler en boucle dans le main).
 */
void Drone_Process(void);

#endif /* DRONE_H_ */
