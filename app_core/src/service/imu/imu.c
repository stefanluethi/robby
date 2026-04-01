#include "imu.h"

#include "custom_mems_conf.h"
#include "lsm6dso.h"

static LSM6DSO_Object_t driver;

void IMU_init(void)
{
    LSM6DSO_IO_t io_ctx;

    io_ctx.BusType     = LSM6DSO_I2C_BUS;
    io_ctx.Address     = LSM6DSO_I2C_ADD_L;
    io_ctx.Init        = CUSTOM_LSM6DSO_0_I2C_Init;
    io_ctx.DeInit      = CUSTOM_LSM6DSO_0_I2C_DeInit;
    io_ctx.ReadReg     = CUSTOM_LSM6DSO_0_I2C_ReadReg;
    io_ctx.WriteReg    = CUSTOM_LSM6DSO_0_I2C_WriteReg;
    io_ctx.GetTick     = BSP_GetTick;

    LSM6DSO_RegisterBusIO(&driver, &io_ctx);
    LSM6DSO_Init(&driver);
    LSM6DSO_ACC_SetOutputDataRate(&driver, 104.0f);
    LSM6DSO_ACC_SetFullScale(&driver, 4);

    LSM6DSO_GYRO_SetOutputDataRate(&driver, 104.0f);
    LSM6DSO_GYRO_SetFullScale(&driver, 2000);

    LSM6DSO_ACC_Enable(&driver);
    LSM6DSO_GYRO_Enable(&driver);
}

void IMU_process(void)
{
    LSM6DSO_Axes_t accel;
    LSM6DSO_Axes_t gyro;

    LSM6DSO_ACC_GetAxes(&driver, &accel);
    LSM6DSO_GYRO_GetAxes(&driver, &gyro);
}
