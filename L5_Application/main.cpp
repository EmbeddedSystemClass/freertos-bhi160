
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "utilities.h"
#include "lib/BHIfw.h"
#include "lib/bhy_uc_driver.h"
#include "i2c2.hpp"
#include "i2c_base.hpp"
#include "stdio.h"

static struct bhy_t bhy;

int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
void bhy_delay_msec(u32 msec);

int8_t bhy_initialize_support(void)
{
    uint8_t tmp_retry = RETRY_NUM;

    bhy.bus_write = &sensor_i2c_write;
    bhy.bus_read = &sensor_i2c_read;
    bhy.delay_msec  = &bhy_delay_msec;
    bhy.device_addr = BHY_I2C_SLAVE_ADDRESS;

    bhy_init(&bhy);

    return 0;

    bhy_set_reset_request(BHY_RESET_ENABLE);

    while(tmp_retry--)
    {
        bhy_get_product_id(&bhy.product_id);

        if(PRODUCT_ID_7183 == bhy.product_id)
        {
            return BHY_SUCCESS;
        }

        bhy_delay_msec(BHY_PARAMETER_ACK_DELAY);
    }

    return BHY_PRODUCT_ID_ERROR;
}

int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size){
    addr = (addr << 1);
    bool ok = I2C2::getInstance().writeRegisters(addr, reg, p_buf, size);
    if(ok){
        return 0;
    }
    return -1;
}

int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size){
    addr = (addr << 1);
    bool ok = I2C2::getInstance().readRegisters(addr, reg, p_buf, size);
    if(ok){
        //printf("Success in Read\n");
        //printf("0x%x\n", *p_buf);
        return 0;
    }
    return -1;
}

void bhy_delay_msec(u32 msec){
    delay_ms(msec);
}


int main(void)
{
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    BHY_RETURN_FUNCTION_TYPE a = bhy_driver_init(bhy_firmware_image);
    // printf("result of driver init is: %d\n", a);

    BHY_RETURN_FUNCTION_TYPE b = bhy_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, VS_WAKEUP, 25, 0, VS_FLUSH_NONE, 0, 0);
    //printf("result of enable_virtual_sensor is: %d\n", b);

//    BHY_RETURN_FUNCTION_TYPE c = bhy_install_sensor_callback (VS_TYPE_ACCELEROMETER, VS_WAKEUP,
//                                                              void (*sensor_callback)(bhy_data_generic_t *, bhy_virtual_sensor_t));
//    printf("result of install_sensor_callback is: %d\n", c);

    //int8_t a = bhy_initialize_support();

    /* test if write working properly
    uint8_t buf = 0x02;
    uint16_t size = 1;
    sensor_i2c_write(0x28, 0x56, &buf, size);
    sensor_i2c_read(0x28, 0x56, &buf, size);
    */


    scheduler_start(); ///< This shouldn't return
    return -1;
}
