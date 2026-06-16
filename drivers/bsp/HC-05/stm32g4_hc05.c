/*
 * stm32g4_hc05.c
 *
 *  Created on: Sep 30, 2024
 *      Author: Nirgal
 */

#include "stm32g4_uart.h"


/*
 * Cette fonction est utilis�e pour permettre le dialogue en un PC reli� via la port s�rie virtuel de la sonde de d�bogage (via l'UART2)
 *   et un module HC-05 reli� � l'UART1 (PA9, PA10).
 *
 *  Son usage est g�n�ralement temporaire, le temps de la configuration du module HC-05 en mode AT.
 *
 *  Cette fonction doit �tre appel�e en d�but de main() apr�s HAL_init();
 *  Elle est blocante !
 */
void HC05_set_echo_for_AT_mode(void)
{
	BSP_UART_init(UART1_ID, 9600);//38400);		//vitesse n�cessaire pour la configuration du HC-05 en mode AT
	BSP_UART_init(UART2_ID, 115200);
	uint8_t c;
	volatile bool config_mode = true;
	while(config_mode)
	{
		if(BSP_UART_data_ready(UART1_ID))
		{
			c = BSP_UART_getc(UART1_ID);
			BSP_UART_putc(UART2_ID, c);
		}
		if (BSP_UART_data_ready(UART2_ID))
		{
			c = BSP_UART_getc(UART2_ID);
			BSP_UART_putc(UART1_ID, c);
		}

		//pour sortir de cette boucle, il suffit de passer config_mode � false � l'aide du d�bogueur.
	}

	BSP_UART_init(UART1_ID, 9600);	//vitesse du HC-05 en mode de communication normale (si vous avez configur� cette vitesse ainsi).
}
