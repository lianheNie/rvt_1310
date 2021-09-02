/*
 * aw_config.h
 *
 *  Created on: 2020年9月21日
 *      Author: admin
 */

#ifndef CONFIG_AW_CONFIG_H_
#define CONFIG_AW_CONFIG_H_

#define IS_USE_4X4_PACKAGE  // 4X4封裝

////////////////////////////////////////9///////////////////////////////////
#define AW_EASYLINK_SELF_ADDRESS 303//1009//1001//37 // 22 //21（坏）//1006  //10//303  //97  //99 有功放    //5002  //1932          // 1203  1103        //   36 设备地址
#define AW_EASYLINK_PAIR_DEFAULT_ADDRESS 0                 //设备配对地址                  AW_STR(AW_EASYLINK_SELF_ADDRESS)

#define IS_USE_LSM6DSM
#define IS_USE_ACC_FIFO
#define IS_USE_LSM6DSM_SPI

#define IS_USE_EASYLINK_ACK  //启用433MHz通信应答机制

#define IS_USE_RETRANSIMIT  //启用网关转发机制

#define IS_USE_NVS  //启用内部储存flash
#define IS_USE_PAIR  //启用配对机制
#define IS_USE_EASYLINK  //启用EASYLINK(433MHz)

#endif /* CONFIG_AW_CONFIG_H_ */
