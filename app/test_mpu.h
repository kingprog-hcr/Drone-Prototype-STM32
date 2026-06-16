/**
 *******************************************************************************
 * @file    test_mpu.h
 * @brief   Déclaration de la fonction de test unitaire du MPU6050
 *******************************************************************************
 */

#ifndef TEST_MPU_H_
#define TEST_MPU_H_

/**
 * @brief Lance le test unitaire du MPU6050.
 * /!\ Cette fonction est bloquante et contient sa propre boucle while(1)
 */
void test_MPU6050_run(void);

#endif /* TEST_MPU_H_ */
