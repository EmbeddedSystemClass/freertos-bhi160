
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "utilities.h"
#include "lib/BHIfw.h"
#include "lib/bhy_uc_driver.h"
#include "i2c2.hpp"
#include "i2c_base.hpp"
#include "gpio.hpp"
#include "stdio.h"

/********************************************************************************/
/*                                       MACROS                                 */
/********************************************************************************/
/* should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18
#define TICKS_IN_ONE_SECOND            32000.0F

uint32_t bhy_timestamp = 0;
uint8_t fifo[FIFO_SIZE];
static struct bhy_t bhy;



static void sensors_callback_acc(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    float   time_stamp    = 0;
    uint8_t sensor_type   = 0;
    int16_t x_raw         = 0;
    int16_t y_raw         = 0;
    int16_t z_raw         = 0;
    float   x_data        = 0;
    float   y_data        = 0;
    float   z_data        = 0;

    /* Since a timestamp is always sent before every new data, and that the callbacks   */
    /* are called while the parsing is done, then the system timestamp is always equal  */
    /* to the sample timestamp. (in callback mode only)                                 */
    time_stamp = (float)(bhy_timestamp) / TICKS_IN_ONE_SECOND;

    printf("sensor_id = %d\n", sensor_id);

    switch(sensor_id)
    {
        case VS_ID_ACCELEROMETER:
        case VS_ID_ACCELEROMETER_WAKEUP:
            x_raw  = sensor_data->data_vector.x;
            y_raw  = sensor_data->data_vector.y;
            z_raw  = sensor_data->data_vector.z;
            /* The resolution is  15bit ,the default range is 4g, actual acceleration equals: raw_data/(exp(2,15) == 32768) */
            x_data = (float)x_raw / 32768.0f * 4.0f;
            y_data = (float)y_raw / 32768.0f * 4.0f;
            z_data = (float)z_raw / 32768.0f * 4.0f;

            printf("Time:%6.3fs acc %f %f %f\n", time_stamp, x_data, y_data, z_data);
            break;

        default:
            printf("unknown id = %d\n", sensor_id);
            break;
    }
}


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
    vTaskDelay(msec);
}



void bhy_printf(const u8 * string)
{
//    trace_log("%s",string);
}

/*!
 * @brief This function is used to run bhy hub
 */
void demo_sensor(void){
    GPIO int_pin(P2_0);
    int_pin.setAsInput();
    int8_t ret;

    /* BHY Variable*/
    uint8_t                    *fifoptr           = NULL;
    uint8_t                    bytes_left_in_fifo = 0;
    uint16_t                   bytes_remaining    = 0;
    uint16_t                   bytes_read         = 0;
    bhy_data_generic_t         fifo_packet;
    bhy_data_type_t            packet_type;
    BHY_RETURN_FUNCTION_TYPE   result;
//    int8_t                    bhy_mapping_matrix_init[3*3]   = {0};
//    int8_t                    bhy_mapping_matrix_config[3*3] = {0,1,0,-1,0,0,0,0,1};
    struct cus_version_t      bhy_cus_version;

    /* init the bhy chip */
    if(bhy_driver_init(bhy_firmware_image))
    {
        printf("Fail to init bhy\n");
    }

    /* wait for the bhy trigger the interrupt pin go down and up again */
    while (int_pin.read());

    while (!int_pin.read());

    /* install the callback function for parse fifo data */
    if(bhy_install_sensor_callback(VS_TYPE_ACCELEROMETER, VS_WAKEUP, sensors_callback_acc))
    {
        printf("Fail to install sensor callback\n");
    }

    /* enables the virtual sensor */
    if(bhy_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, VS_WAKEUP, 10, 0, VS_FLUSH_NONE, 0, 0))
    {
        printf("Fail to enable sensor id=%d\n", VS_TYPE_ACCELEROMETER);
    }

    /* continuously read and parse the fifo */
     while(1)
     {
         /* wait until the interrupt fires */
         /* unless we already know there are bytes remaining in the fifo */
         while (!int_pin.read() && !bytes_remaining)
         {
         }

         bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
         bytes_read           += bytes_left_in_fifo;
         fifoptr              = fifo;
         packet_type          = BHY_DATA_TYPE_PADDING;

         do
         {
             /* this function will call callbacks that are registered */
             result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

             /* prints all the debug packets */
             if (packet_type == BHY_DATA_TYPE_DEBUG)
             {
                 bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
             }

             /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
             /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
             /* packet */
         } while ((result == BHY_SUCCESS) && (bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)));

         bytes_left_in_fifo = 0;

         if (bytes_remaining)
         {
             /* shifts the remaining bytes to the beginning of the buffer */
             while (bytes_left_in_fifo < bytes_read)
             {
                 fifo[bytes_left_in_fifo++] = *(fifoptr++);
             }
         }
     }
}

int main(void)
{
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    demo_sensor();

    scheduler_start(); ///< This shouldn't return
    return -1;
}
