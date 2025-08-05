#ifndef ICM_42670_REGISTERS
#define ICM_42670_REGISTERS

// USER BANk0 registers
#define MCLK_RDY          0x00
#define DEVICE_CONFIG     0x01
#define SIGNAL_PATH_RESET 0x02
#define DRIVE_CONFIG1     0x03
#define DRIVE_CONFIG2     0x04
#define DRIVE_CONFIG3     0x05
#define INT_CONFIG        0x06
#define TEMP_DATA1        0x09
#define TEMP_DATA0        0x0A
#define ACCEL_DATA_X1     0x0B
#define ACCEL_DATA_X0     0x0C
#define ACCEL_DATA_Y1     0x0D
#define ACCEL_DATA_Y0     0x0E
#define ACCEL_DATA_Z1     0x0F
#define ACCEL_DATA_Z0     0x10
#define GYRO_DATA_X1      0x11
#define GYRO_DATA_X0      0x12
#define GYRO_DATA_Y1      0x13
#define GYRO_DATA_Y0      0x14
#define GYRO_DATA_Z1      0x15
#define GYRO_DATA_Z0      0x16
#define TMST_FSYNCH       0x17
#define TMST_FSYNCL       0x18
#define APEX_DATA4        0x1D
#define APEX_DATA5        0x1E
#define PWR_MGMT0         0x1F
#define GYRO_CONFIG0      0x20
#define ACCEL_CONFIG0     0x21
#define TEMP_CONFIG0      0x22
#define GYRO_CONFIG1      0x23
#define ACCEL_CONFIG1     0x24
#define APEX_CONFIG0      0x25
#define APEX_CONFIG1      0x26
#define WOM_CONFIG        0x27
#define FIFO_CONFIG1      0x28
#define FIFO_CONFIG2      0x29
#define FIFO_CONFIG3      0x2A
#define INT_SOURCE0       0x2B
#define INT_SOURCE1       0x2C
#define INT_SOURCE3       0x2D
#define INT_SOURCE4       0x2E
#define FIFO_LOST_PKT0    0x2F
#define FIFO_LOST_PKT1    0x30
#define APEX_DATA0        0x31
#define APEX_DATA1        0x32
#define APEX_DATA2        0x33
#define APEX_DATA3        0x34
#define INTF_CONFIG0      0x35
#define INTF_CONFIG1      0x36
#define INT_STATUS_DRDY   0x39
#define INT_STATUS        0x3A
#define INT_STATUS2       0x3B
#define INT_STATUS3       0x3C
#define FIFO_COUNTH       0x3D
#define FIFO_COUNTL       0x3E
#define FIFO_DATA         0x3F
#define WHO_AM_I          0x75
#define BLK_SEL_W         0x79
#define MADDR_W           0x7A
#define M_W               0x7B
#define BLK_SEL_R         0x7C
#define MADDR_R           0x7D
#define M_R               0x7E

// USER BANK MREG1 registers
#define TMST_CONFIG1    0x00
#define FIFO_CONFIG5    0x01
#define FIFO_CONFIG6    0x02
#define FSYNC_CONFIG    0x03
#define INT_CONFIG0     0x04
#define INT_CONFIG1     0x05
#define SENSOR_CONFIG3  0x06
#define ST_CONFIG       0x13
#define SELFTEST        0x14
#define INTF_CONFIG6    0x23
#define INTF_CONFIG10   0x25
#define INTF_CONFIG7    0x28
#define OTP_CONFIG      0x2B
#define INT_SOURCE6     0x2F
#define INT_SOURCE7     0x30
#define INT_SOURCE8     0x31
#define INT_SOURCE9     0x32
#define INT_SOURCE10    0x33
#define APEX_CONFIG2    0x44
#define APEX_CONFIG3    0x45
#define APEX_CONFIG4    0x46
#define APEX_CONFIG5    0x47
#define APEX_CONFIG9    0x48
#define APEX_CONFIG10   0x49
#define APEX_CONFIG11   0x4A
#define ACCEL_WDM_X_THR 0x4B
#define ACCEL_WDM_Y_THR 0x4C
#define ACCEL_WDM_Z_THR 0x4D
#define OFFSET_USER0    0x4E
#define OFFSET_USER1    0x4F
#define OFFSET_USER2    0x50
#define OFFSET_USER3    0x51
#define OFFSET_USER4    0x52
#define OFFSET_USER5    0x53
#define OFFSET_USER6    0x54
#define OFFSET_USER7    0x55
#define ST_STATUS1      0x63
#define ST_STATUS2      0x64
#define FDR_CONFIG      0x66
#define APEX_CONFIG12   0x67

// USER BANK MREG2 registers
#define OTP_CTRL7 0x06

// USER BANK MREG3 registers
#define XA_ST_DATA  0x00
#define YA_ST_DATA  0x01
#define ZA_ST_DATA  0x02
#define XG_ST_DATA  0x03
#define YG_ST_DATA  0x04
#define ZG_ST_DATA  0x05

#endif