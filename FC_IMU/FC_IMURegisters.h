#ifndef __FC_IMU_REGISTERS_H__
#define __FC_IMU_REGISTERS_H__

// ************ Self Test Registers ************
#define  SELF_TEST_X          0x0D
#define  SELF_TEST_Y          0x0E
#define  SELF_TEST_Z          0x0F
#define  SELF_TEST_A          0x10

// ********* Sensor Config Registers ************
#define  CONFIG               0x1A
// --------------- Gyroscope ----------------------------------
//    7     6     5     4:3      2    1    0
// | STx | STy | STz | FS_SEL | -- | -- | -- |
// 
// FS_SEL (deg/s): 0 -> 250 | 1 -> 500 | 2 -> 1000 | 3 -> 2000
// ------------------------------------------------------------
#define  GYRO_CONFIG          0x1B
// ------------- Accelerometer -----------------------
//     7    6     5     4:3      2    1    0
// | STx | STy | STz | FS_SEL | -- | -- | -- |
// 
// FS_SEL (+/- Xg): 0 -> 2 | 1 -> 4 | 2 -> 8 | 3 -> 16
// ---------------------------------------------------
#define  ACCEL_CONFIG         0x1C

// ************ Power Management Registers ************
// ----------------- PWR_MGMT_1 ------------------------
//    7       6       5     4        3          2:0
// | RST | SLEEP | CYCLE | -- | TMP_DISABLE | CLK_SEL |
// ----------------------------------------------------
#define  PWR_MGMT_1           0x6B
#define  PWR_MGMT_2           0x6C

// ************ FIFO Registers ************
// ------------------------- FIFO_EN ----------------------
//     7      6      5      4     3      2      1      0    
// | TEMP | GyrX | GyrY | GyrZ | Acc | Slv2 | Slv1 | Slv0 |
// --------------------------------------------------------
#define  FIFO_EN              0x23
#define  FIFO_COUNT_H         0x72
#define  FIFO_COUNT_L         0x73
#define  FIFO_R_W             0x74

// ************ Interrupt Registers ************
#define  INT_PIN_CFG          0x37
#define  INT_ENABLE           0x38
#define  INT_STATUS           0x3A

// ************ Sensor Output Registers ************
// Accelerometer
#define  ACCEL_XOUT_H         0x3B
#define  ACCEL_XOUT_L         0x3C
#define  ACCEL_YOUT_H         0x3D
#define  ACCEL_YOUT_L         0x3E
#define  ACCEL_ZOUT_H         0x3F
#define  ACCEL_ZOUT_L         0x40
#define  TEMP_OUT_H           0x41
#define  TEMP_OUT_L           0x42
#define  GYRO_XOUT_H          0x43
#define  GYRO_XOUT_L          0x44
#define  GYRO_YOUT_H          0x45
#define  GYRO_YOUT_L          0x46
#define  GYRO_ZOUT_H          0x47
#define  GYRO_ZOUT_L          0x48

#endif