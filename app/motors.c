#include <stm32g4_timer.h>
#include <stdint.h>
#include <stdlib.h>
#include <motors.h>

// Tableau global contenant la configuration matérielle de chaque moteur
const Motor_t MY_MOTORS[4] = {
    // Moteur 0
    { .timer_fin = TIMER3_ID, .channel_fin = TIM_CHANNEL_1, .timer_rin = TIMER3_ID, .channel_rin = TIM_CHANNEL_3 },
    // Moteur 1
    { .timer_fin = TIMER4_ID, .channel_fin = TIM_CHANNEL_2, .timer_rin = TIMER4_ID, .channel_rin = TIM_CHANNEL_1 },
    // Moteur 2
    { .timer_fin = TIMER1_ID, .channel_fin = TIM_CHANNEL_4, .timer_rin = TIMER2_ID, .channel_rin = TIM_CHANNEL_2 },
    // Moteur 3
    { .timer_fin = TIMER2_ID, .channel_fin = TIM_CHANNEL_1, .timer_rin = TIMER3_ID, .channel_rin = TIM_CHANNEL_2 }
};

void Motors_Init(void) {


    // 1. Configurer les TIMERs pour une période de 50 us (sans interruption)
	BSP_TIMER_run_us(TIMER1_ID, 50, 0); //  Moteur 2
	BSP_TIMER_run_us(TIMER2_ID, 50, 0); //  Moteur 3 et 2
	BSP_TIMER_run_us(TIMER3_ID, 50, 0); // Moteur 0 et 3
	BSP_TIMER_run_us(TIMER4_ID, 50, 0); // Moteur 1



    // 2. Activer la PWM sur les TIMERs,en fonction du canal pour chaque moteur, puissance 0, sans remap, sans canal négatif
	//Moteur 0
	 BSP_TIMER_enable_PWM(TIMER3_ID, TIM_CHANNEL_3, 0, 0, 0);  //Rin
	 BSP_TIMER_enable_PWM(TIMER3_ID, TIM_CHANNEL_1, 0, 1, 0); // Fin

	 //Moteur 1
	 BSP_TIMER_enable_PWM(TIMER4_ID, TIM_CHANNEL_1, 0, 1, 0);  //Rin
	 BSP_TIMER_enable_PWM(TIMER4_ID, TIM_CHANNEL_2, 0, 0, 0); // Fin

	 //Moteur 2
	 BSP_TIMER_enable_PWM(TIMER2_ID, TIM_CHANNEL_2, 0, 0, 0);  //Rin
	 BSP_TIMER_enable_PWM(TIMER1_ID, TIM_CHANNEL_4, 0, 0, 0); // Fin

	//Moteur 3
	 BSP_TIMER_enable_PWM(TIMER3_ID, TIM_CHANNEL_2, 0, 0, 0); // Rin
	 BSP_TIMER_enable_PWM(TIMER2_ID, TIM_CHANNEL_1, 0, 0, 0); // Fin


}


void Motors_SetSpeed(uint8_t motor_id, int16_t vitesse) {
    // Sécurité 1 : On vérifie que l'index du moteur est valide
    if (motor_id > 3) return;

    // Sécurité 2 : Saturation de la vitesse entre -1000 et 1000 (Pas besoin car c'est gere par BSP_TIMER_set_duty)
//    if (vitesse > 1000)  vitesse = 1000;
//    if (vitesse < -1000) vitesse = -1000;

    // On récupère un pointeur vers la configuration du moteur choisi
    const Motor_t *motor = &MY_MOTORS[motor_id];

    if (vitesse > 0) {
        // Marche avant : Puissance sur Fin, 0 sur Rin
        BSP_TIMER_set_duty(motor->timer_fin, motor->channel_fin, vitesse);
        BSP_TIMER_set_duty(motor->timer_rin, motor->channel_rin, 0);
    }
    else if (vitesse < 0) {
        // Marche arrière : 0 sur Fin, Puissance absolue sur Rin
        BSP_TIMER_set_duty(motor->timer_fin, motor->channel_fin, 0);
        BSP_TIMER_set_duty(motor->timer_rin, motor->channel_rin, abs(vitesse));
    }
    else {
        // Arrêt : 0 partout
        BSP_TIMER_set_duty(motor->timer_fin, motor->channel_fin, 0);
        BSP_TIMER_set_duty(motor->timer_rin, motor->channel_rin, 0);
    }
}
