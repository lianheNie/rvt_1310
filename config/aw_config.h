/*
 * aw_config.h
 *
 *  Created on: 2020��9��21��
 *      Author: admin
 */

#ifndef CONFIG_AW_CONFIG_H_
#define CONFIG_AW_CONFIG_H_

#define IS_USE_4X4_PACKAGE  // 4X4���b

////////////////////////////////////////9///////////////////////////////////
#define AW_EASYLINK_SELF_ADDRESS 303//1009//1001//37 // 22 //21������//1006  //10//303  //97  //99 �й���    //5002  //1932          // 1203  1103        //   36 �豸��ַ
#define AW_EASYLINK_PAIR_DEFAULT_ADDRESS 0                 //�豸��Ե�ַ                  AW_STR(AW_EASYLINK_SELF_ADDRESS)

#define IS_USE_LSM6DSM
#define IS_USE_ACC_FIFO
#define IS_USE_LSM6DSM_SPI

#define IS_USE_EASYLINK_ACK  //����433MHzͨ��Ӧ�����

#define IS_USE_RETRANSIMIT  //��������ת������

#define IS_USE_NVS  //�����ڲ�����flash
#define IS_USE_PAIR  //������Ի���
#define IS_USE_EASYLINK  //����EASYLINK(433MHz)

#endif /* CONFIG_AW_CONFIG_H_ */
