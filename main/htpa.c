#include <stdio.h>
#include "esp_log.h"
#include "htpa.h"
#include "driver/i2c.h"
#include "esp_types.h"
#include "sensordef_32x32.h"
#include "lookuptable.h"
#include <math.h>
#include <string.h>

#define TAG "HTPA"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, arraytype;
int8_t globaloff;
uint8_t mbit_user, bias_user, clk_user, bpa_user, pu_user;
uint16_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
int16_t thgrad[32][32];
int16_t thoffset[32][32];
int32_t vddcompgrad[8][32];
int16_t vddcompoff[8][32];
uint16_t pij[32][32];
uint16_t deadpixadr[24];
uint8_t deadpixmask[12];
int32_t pixcij_int32[32][32];
uint32_t id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;
uint16_t addr_i = 0x0740;
extern xSemaphoreHandle HTPAReadySemaphore;
extern xSemaphoreHandle HtpaSemaphore;
extern cJSON *rootJSON;
uint8_t TRIAL_COUNT = 1.0 ;
struct characteristics
{
    uint8_t number_row;    // number of raws
    uint8_t number_col;    // number of column
    uint8_t number_blocks; // number of blocks (top + down)
    uint16_t number_pixel; // number of pixel
};

struct characteristics sensor = {32, 32, 8, 1024};
uint8_t data_top_block0[258], data_top_block1[258], data_top_block2[258], data_top_block3[258];
uint8_t data_bottom_block0[258], data_bottom_block1[258], data_bottom_block2[258], data_bottom_block3[258];
uint8_t electrical_offset_top[258], electrical_offset_bottom[258];
uint16_t eloffset[8][32];
double ptat_top_block0, ptat_top_block1, ptat_top_block2, ptat_top_block3;
double ptat_bottom_block0, ptat_bottom_block1, ptat_bottom_block2, ptat_bottom_block3;
uint16_t vdd_top_block0, vdd_top_block1, vdd_top_block2, vdd_top_block3;
uint16_t vdd_bottom_block0, vdd_bottom_block1, vdd_bottom_block2, vdd_bottom_block3;
uint16_t data_pixel[32][32];
uint8_t statusreg;

// CALCULATED VALUES
double ptat_av_uint16;
double vdd_av_uint16;
double ambient_temperature;
double vij_pixc_int32[32][32];
double temp_pix_uint32[32][32];
double average_temp[32][32];
double vij_comp_int32[32][32];
double vij_comp_s_int32[32][32];
double vij_vddcomp_int32[32][32];

// OTHER
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;
char var = 'm';

static gpio_num_t i2c_gpio_sda = 1;
static gpio_num_t i2c_gpio_scl = 2;
static uint32_t i2c_frequency = 400000;
static i2c_port_t i2c_port = I2C_NUM_0;

void i2cSensorCommand(uint8_t CONF_REGISTER, uint8_t command)
{
    i2c_cmd_handle_t cmd_handler = i2c_cmd_link_create();
    i2c_master_start(cmd_handler);

    i2c_master_write_byte(cmd_handler, SENSOR_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);

    i2c_master_write_byte(cmd_handler, CONF_REGISTER, ACK_CHECK_EN);

    i2c_master_write_byte(cmd_handler, command, ACK_CHECK_EN);

    i2c_master_stop(cmd_handler);
    i2c_master_cmd_begin(i2c_port, cmd_handler, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handler);
}

uint8_t read_EEPROM_byte(uint8_t deviceaddress, uint16_t eeaddress)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | WRITE_BIT, 1);
    i2c_master_write_byte(cmd, eeaddress >> 8, 1);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | READ_BIT, 1);

    uint8_t data;
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}
uint8_t i2cSensorReadCommand(uint16_t sensorRegester, uint8_t *data, uint16_t length)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_ADDRESS << 1) | WRITE_BIT, 1);
    i2c_master_write_byte(cmd, sensorRegester, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_ADDRESS << 1) | READ_BIT, 1);

    if (length > 1)
    {
        i2c_master_read(cmd, data, length - 1, 0);
    }
    i2c_master_read_byte(cmd, data + length - 1, 1);

    i2c_master_stop(cmd);
    i2c_set_timeout(i2c_port, 0x1F);

    i2c_master_cmd_begin(i2c_port, cmd, 0x1F);
    i2c_cmd_link_delete(cmd);
    return 1;
}
void calculate_pixcij()
{

    for (int m = 0; m < 32; m++)
    {
        for (int n = 0; n < 32; n++)
        {

            // calc sensitivity coefficients (see datasheet, chapter: 11.5 Object Temperature)
            pixcij_int32[m][n] = (int32_t)pixcmax - (int32_t)pixcmin;
            pixcij_int32[m][n] = pixcij_int32[m][n] / 65535;
            pixcij_int32[m][n] = pixcij_int32[m][n] * pij[m][n];
            pixcij_int32[m][n] = pixcij_int32[m][n] + pixcmin;
            pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0 * epsilon / 100;
            pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0 * globalgain / 10000;
        }
    }
}

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = i2c_frequency,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    return i2c_param_config(i2c_port, &conf);
}
void pixel_masking()
{

    uint8_t number_neighbours[24];
    uint32_t temp_defpix[24];

    for (int i = 0; i < nrofdefpix; i++)
    {
        number_neighbours[i] = 0;
        temp_defpix[i] = 0;

        // top half

        if (deadpixadr[i] < 512)
        {

            if ((deadpixmask[i] & 1) == 1)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32)];
            }

            if ((deadpixmask[i] & 2) == 2)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) + 1];
            }

            if ((deadpixmask[i] & 4) == 4)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) + 1];
            }

            if ((deadpixmask[i] & 8) == 8)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) + 1];
            }

            if ((deadpixmask[i] & 16) == 16)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32)];
            }

            if ((deadpixmask[i] & 32) == 32)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) - 1];
            }

            if ((deadpixmask[i] & 64) == 64)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) - 1];
            }

            if ((deadpixmask[i] & 128) == 128)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) - 1];
            }
        }

        // bottom half
        else
        {

            if ((deadpixmask[i] & 1 << 0) == 1 << 0)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32)];
            }

            if ((deadpixmask[i] & 2) == 2)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) + 1];
            }

            if ((deadpixmask[i] & 4) == 4)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) + 1];
            }

            if ((deadpixmask[i] & 8) == 8)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) + 1];
            }

            if ((deadpixmask[i] & 16) == 16)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32)];
            }

            if ((deadpixmask[i] & 32) == 32)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) - 1];
            }

            if ((deadpixmask[i] & 64) == 64)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) - 1];
            }

            if ((deadpixmask[i] & 128) == 128)
            {
                number_neighbours[i]++;
                temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) - 1];
            }
        }

        temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
        temp_pix_uint32[deadpixadr[i] >> 5][deadpixadr[i] % 32] = temp_defpix[i];
    }
}

/********************************************************************
   Function:        print_eeprom_value()

   Description:     print all needed values in their saved form

   Dependencies:
 *******************************************************************/
void print_eeprom_value()
{

    printf("\n\n\n---PRINT EEPROM (VALUE)---\n");
    printf("\nHINT: Here values longer than 8 bit are printed in their first block.\n");
    printf("\n\nEEPROM 32x32\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

    // line
    for (int i = 0; i < 75; i++)
    {
        printf("- ");
    }
    // HEADER
    // 1st line
    printf("\n");
    printf("HEADER\t0x00");
    printf("\t|\t");
    printf("%f", pixcmin);
    printf("\t\t\t");
    printf("%f", pixcmax);
    printf("\t\t\t");
    printf("%u", gradscale);
    printf("\t\t\t");
    printf("%u", tablenumber);
    printf("\t\t");
    printf("%u", epsilon);
    // 2nd line
    printf("\n");
    printf("HEADER\t0x10");
    printf("\t|\t\t\t\t\t\t\t\t\t\t\t");
    printf("%u", mbit_calib);
    printf("\t");
    printf("%u", bias_calib);
    printf("\t");
    printf("%u", clk_calib);
    printf("\t");
    printf("%u", bpa_calib);
    printf("\t");
    printf("%u", pu_calib);
    // 3rd line
    printf("\n");
    printf("HEADER\t0x20");
    printf("\t|\t\t\t");
    printf("%u", arraytype);
    printf("\t\t\t\t");
    printf("%u", vddth1);
    printf("\t\t");
    printf("%u", vddth2);
    // 4th line
    printf("\n");
    printf("HEADER\t0x30");
    printf("\t|\t\t\t\t\t");
    printf("%f", ptatgr_float);
    printf("\t\t\t\t");
    printf("%f", ptatoff_float);
    printf("\t\t\t\t");
    printf("%u", ptatth1);
    printf("\t\t");
    printf("%u", ptatth2);
    // 5th line
    printf("\n");
    printf("HEADER\t0x40");
    printf("\t|\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t");
    printf("%u", vddscgrad);
    printf("\t");
    printf("%u", vddscoff);
    // 6th line
    printf("\n");
    printf("HEADER\t0x50");
    printf("\t|\t\t\t\t\t");
    printf("%u", globaloff);
    printf("\t");
    printf("%u", globalgain);
    // 7th line
    printf("\n");
    printf("HEADER\t0x60");
    printf("\t|\t");
    printf("%u", mbit_user);
    printf("\t");
    printf("%u", bias_user);
    printf("\t");
    printf("%u", clk_user);
    printf("\t");
    printf("%u", bpa_user);
    printf("\t");
    printf("%u", pu_user);
    // 8th line
    printf("\n");
    printf("HEADER\t0x70");
    printf("\t|\t\t\t\t\t");
    printf("%u", id);
    printf("\t\t\t\t\t\t\t\t\t\t\t");
    printf("%u", nrofdefpix);

    // OTHER (16bit)
    for (int i = 0x0080; i <= 0x00AF; i = i + 2)
    {

        if (i % 16 == 0)
        {
            printf("\n");
            printf("DEADPIX\t0x");
            printf("%x", i);
            printf("\t|\t");
        }
        else
        {
            printf("\t\t");
        }
        printf("%x", read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i));
    }

    // OTHER (8bit)
    for (int i = 0x00B0; i <= 0x033F; i++)
    {

        if (i % 16 == 0)
        {
            printf("\n");
            if (i < 0x00D0)
            {
                printf("DEADPIX\t0x");
            }
            else
            {
                printf("FREE\t0x");
            }
            printf("%x", i);
            printf("\t|\t");
        }
        else
        {
            printf("\t");
        }
        printf("%x", read_EEPROM_byte(EEPROM_ADDRESS, i));
    }

    // OTHER (16bit)
    for (int i = 0x0340; i <= 0x1F3F; i = i + 2)
    {

        if (i % 16 == 0)
        {
            printf("\n");

            if (i < 0x0540)
            {
                printf("VDDGRAD\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x0740)
            {
                printf("VDDOFF\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x0F40)
            {
                printf("THGRAD\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x1740)
            {
                printf("THOFF\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x1F40)
            {
                printf("Pij\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
        }

        else
        {
            printf("\t\t");
        }

        if (i >= 0x0340 && i < 0x1740)
        {
            printf("%d", (int32_t)(read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i)));
        }
        else
        {
            printf("%d", read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i));
        }
    }

    printf("\n\n\n\n ");
}

void print_eeprom_hex()
{

    printf("\n\n\n---PRINT EEPROM (HEX)---\n");
    printf("\n\nEEPROM 32x32\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

    // line
    for (int i = 0; i < 75; i++)
    {
        printf("- ");
    }

    for (int i = 0; i <= 0x13FF; i++)
    {

        if (i % 16 == 0)
        {
            printf("\n");

            if (i < 0x0080)
            {
                printf("HEADER\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x00D0)
            {
                printf("DEADPIX\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x0340)
            {
                printf("FREE\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x0540)
            {
                printf("VDDGRAD\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x0740)
            {
                printf("VDDOFF\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x0F40)
            {
                printf("THGRAD\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x1740)
            {
                printf("THOFF\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
            else if (i < 0x1F40)
            {
                printf("Pij\t0x");
                printf("%x", i);
                printf("\t|\t");
            }
        }
        else
        {
            printf("\t");
        }

        printf("%x \n", read_EEPROM_byte(EEPROM_ADDRESS, i));
    }
}

void calc_average_temp()
{
    for (int m = 0; m < 32; m++)
    {
        for (int n = 0; n < 32; n++)
        {
            average_temp[m][n] = average_temp[m][n] + temp_pix_uint32[m][n];
        }
    }
}
void print_pixel_temps_average()
{

    rootJSON = cJSON_CreateObject();

    char array2string[15 * 32 * 32 + 6] = {0};

    for (int m = 0; m < 32; m++)
    {
        for (int n = 0; n < 32; n++)
        {
            // ESP_LOGE(TAG, " %f",temp_pix_uint32[m][n] );

            int len = snprintf(NULL, 0, "%.4f, ", (((temp_pix_uint32[m][n]) ) / 10.0000) - 273.2);
            char *result = malloc(len + 1);
            if (m == 31 && n == 31)
            {
                snprintf(result, len + 3, "%.4f", (((temp_pix_uint32[m][n]) ) / 10.0000) - 273.2);
            }
            else
            {
                snprintf(result, len + 3, "%.4f,", (((temp_pix_uint32[m][n]) ) / 10.0000) - 273.2);
            }
            strcat(array2string, result);
            free(result);
        }
    }
        // memset(average_temp, 0,   32 * 32 * sizeof(double)  );
// memset(array, 0, sizeof average_temp);

    cJSON_AddStringToObject(rootJSON, "array", array2string);
}

void print_pixel_temps()
{

    rootJSON = cJSON_CreateObject();

    char array2string[15 * 32 * 32 + 6] = {0};

    for (int m = 0; m < 32; m++)
    {
        for (int n = 0; n < 32; n++)
        {
            // ESP_LOGE(TAG, " %f",temp_pix_uint32[m][n] );

            int len = snprintf(NULL, 0, "%.4f, ", ((temp_pix_uint32[m][n]) / 10.0000) - 273.2);
            char *result = malloc(len + 1);
            if (m == 31 && n == 31)
            {
                snprintf(result, len + 3, "%.4f", ((temp_pix_uint32[m][n]) / 10.0000) - 273.2);
            }
            else
            {
                snprintf(result, len + 3, "%.4f,", ((temp_pix_uint32[m][n]) / 10.0000) - 273.2);
            }
            strcat(array2string, result);
            free(result);
        }
    }

    cJSON_AddStringToObject(rootJSON, "array", array2string);
}

void print_pixel_temps2()
{

    rootJSON = cJSON_CreateObject();

    char array2string[15 * 32 * 32 + 6] = {0};

    for (int m = 0; m < 32; m++)
    {
        for (int n = 0; n < 32; n++)
        {
            // ESP_LOGE(TAG, " %f",temp_pix_uint32[m][n] );

            int len = snprintf(NULL, 0, "%.4f, ", ((temp_pix_uint32[m][n]) / 10.0000) - 273.2);
            char *result = malloc(len + 1);
            if (m == 31 && n == 31)
            {
                snprintf(result, len + 3, "%.4f", ((temp_pix_uint32[m][n]) / 10.0000) - 273.2);
            }
            else
            {
                snprintf(result, len + 3, "%.4f,", ((temp_pix_uint32[m][n]) / 10.0000) - 273.2);
            }
            strcat(array2string, result);
            free(result);
        }
        printf("%s\n", array2string);
        memset(array2string, 0, 15 * 32 * 32 + 6);
    }

    //   ESP_LOGI(TAG, "-----");

    cJSON_AddStringToObject(rootJSON, "array", array2string);
}

void calculate_pixel_temp()
{

    int64_t vij_pixc_and_pcscaleval;
    int64_t vdd_calc_steps;
    uint16_t table_row, table_col;
    int32_t vx, vy, ydist, dta;

    // find column of lookup table
    for (int i = 0; i < NROFTAELEMENTS; i++)
    {
        if (ambient_temperature > XTATemps[i])
        {
            table_col = i;
        }
    }
    dta = ambient_temperature - XTATemps[table_col];
    ydist = (int32_t)ADEQUIDISTANCE;

    for (int m = 0; m < sensor.number_row; m++)
    {
        for (int n = 0; n < sensor.number_col; n++)
        {

            // --- THERMAL OFFSET ---
            // compensate thermal drifts (see datasheet, chapter: 11.2 Thermal Offset)

            vij_comp_int32[m][n] = (data_pixel[m][n] - (thgrad[m][n] * ptat_av_uint16) / gradscale_div - thoffset[m][n]);

            // --- ELECTRICAL OFFSET
            // compensate electrical offset (see datasheet, chapter: 11.3 Electrical Offset)
            // top half
            if (m < sensor.number_row / 2)
            {
                vij_comp_s_int32[m][n] = vij_comp_int32[m][n] - eloffset[m % 4][n];
            }
            // bottom half
            else
            {
                vij_comp_s_int32[m][n] = vij_comp_int32[m][n] - eloffset[m % 4 + 4][n];
            }

            // --- VDD ---
            // select VddCompGrad and VddCompOff for pixel m,n:
            // top half
            if (m < sensor.number_row / 2)
            {
                vddcompgrad_n = vddcompgrad[m % 4][n];
                vddcompoff_n = vddcompoff[m % 4][n];
            }
            // bottom half
            else
            {
                vddcompgrad_n = vddcompgrad[m % 4 + 4][n];
                vddcompoff_n = vddcompoff[m % 4 + 4][n];
            }
            // compensate vdd (see datasheet, chapter: 11.4 Vdd Compensation)
            vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
            vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
            vdd_calc_steps = vdd_calc_steps + vddcompoff_n;

            vdd_calc_steps = vdd_calc_steps * (vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16 - ptatth1));
            vdd_calc_steps = vdd_calc_steps / vddscoff_div;
            vij_vddcomp_int32[m][n] = vij_comp_s_int32[m][n] - vdd_calc_steps;

            // --- SENSITIVITY ---
            // multiply sensitivity coeff for each pixel (see datasheet, chapter: 11.5 Object Temperature)
            vij_pixc_and_pcscaleval = (int64_t)vij_vddcomp_int32[m][n] * (int64_t)PCSCALEVAL;

            vij_pixc_int32[m][n] = (double)(vij_pixc_and_pcscaleval / (int64_t)pixcij_int32[m][n]);

            // --- LOOKUPTABLE ---
            // find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter: 11.7 Look-up table)
            table_row = vij_pixc_int32[m][n] + TABLEOFFSET;
            table_row = table_row >> ADEXPBITS;
            // bilinear interpolation

            vx = ((((double)TempTable[table_row][table_col + 1] - (double)TempTable[table_row][table_col]) * (double)dta) / (double)TAEQUIDISTANCE) + (double)TempTable[table_row][table_col];

            vy = ((((double)TempTable[table_row + 1][table_col + 1] - (double)TempTable[table_row + 1][table_col]) * (double)dta) / (double)TAEQUIDISTANCE) + (double)TempTable[table_row + 1][table_col];
            temp_pix_uint32[m][n] = (double)((vy - vx) * ((double)(vij_pixc_int32[m][n] + TABLEOFFSET) - (double)YADValues[table_row]) / ydist + (double)vx);

            // --- GLOBAL OFFSET ---
            temp_pix_uint32[m][n] = temp_pix_uint32[m][n] + globaloff;
        }
    }
}
void sort_data()
{

    uint32_t sum;

    for (int n = 0; n < sensor.number_col; n++)
    {

        // --- PIXEL DATA TOP HALF ---
        // block 0
        data_pixel[0][n] = data_top_block0[2 * n + 2] << 8 | data_top_block0[2 * n + 3];
        data_pixel[1][n] = data_top_block0[2 * (n + 32) + 2] << 8 | data_top_block0[2 * (n + 32) + 3];
        data_pixel[2][n] = data_top_block0[2 * (n + 64) + 2] << 8 | data_top_block0[2 * (n + 64) + 3];
        data_pixel[3][n] = data_top_block0[2 * (n + 96) + 2] << 8 | data_top_block0[2 * (n + 96) + 3];

        // block 1
        data_pixel[4][n] = data_top_block1[2 * n + 2] << 8 | data_top_block1[2 * n + 3];
        data_pixel[5][n] = data_top_block1[2 * (n + 32) + 2] << 8 | data_top_block1[2 * (n + 32) + 3];
        data_pixel[6][n] = data_top_block1[2 * (n + 64) + 2] << 8 | data_top_block1[2 * (n + 64) + 3];
        data_pixel[7][n] = data_top_block1[2 * (n + 96) + 2] << 8 | data_top_block1[2 * (n + 96) + 3];

        // block 2
        data_pixel[8][n] = data_top_block2[2 * n + 2] << 8 | data_top_block2[2 * n + 3];
        data_pixel[9][n] = data_top_block2[2 * (n + 32) + 2] << 8 | data_top_block2[2 * (n + 32) + 3];
        data_pixel[10][n] = data_top_block2[2 * (n + 64) + 2] << 8 | data_top_block2[2 * (n + 64) + 3];
        data_pixel[11][n] = data_top_block2[2 * (n + 96) + 2] << 8 | data_top_block2[2 * (n + 96) + 3];

        // block 3
        data_pixel[12][n] = data_top_block3[2 * n + 2] << 8 | data_top_block3[2 * n + 3];
        data_pixel[13][n] = data_top_block3[2 * (n + 32) + 2] << 8 | data_top_block3[2 * (n + 32) + 3];
        data_pixel[14][n] = data_top_block3[2 * (n + 64) + 2] << 8 | data_top_block3[2 * (n + 64) + 3];
        data_pixel[15][n] = data_top_block3[2 * (n + 96) + 2] << 8 | data_top_block3[2 * (n + 96) + 3];

        // --- PIXEL DATA BOTTOM HALF ---
        // block 3
        data_pixel[16][n] = data_bottom_block3[192 + 2 * n + 2] << 8 | data_bottom_block3[192 + 2 * n + 3];
        data_pixel[17][n] = data_bottom_block3[128 + 2 * n + 2] << 8 | data_bottom_block3[128 + 2 * n + 3];
        data_pixel[18][n] = data_bottom_block3[64 + 2 * n + 2] << 8 | data_bottom_block3[64 + 2 * n + 3];
        data_pixel[19][n] = data_bottom_block3[0 + 2 * n + 2] << 8 | data_bottom_block3[0 + 2 * n + 3];

        // block 2
        data_pixel[20][n] = data_bottom_block2[192 + 2 * n + 2] << 8 | data_bottom_block2[192 + 2 * n + 3];
        data_pixel[21][n] = data_bottom_block2[128 + 2 * n + 2] << 8 | data_bottom_block2[128 + 2 * n + 3];
        data_pixel[22][n] = data_bottom_block2[64 + 2 * n + 2] << 8 | data_bottom_block2[64 + 2 * n + 3];
        data_pixel[23][n] = data_bottom_block2[0 + 2 * n + 2] << 8 | data_bottom_block2[0 + 2 * n + 3];

        // block 1
        data_pixel[24][n] = data_bottom_block1[192 + 2 * n + 2] << 8 | data_bottom_block1[192 + 2 * n + 3];
        data_pixel[25][n] = data_bottom_block1[128 + 2 * n + 2] << 8 | data_bottom_block1[128 + 2 * n + 3];
        data_pixel[26][n] = data_bottom_block1[64 + 2 * n + 2] << 8 | data_bottom_block1[64 + 2 * n + 3];
        data_pixel[27][n] = data_bottom_block1[0 + 2 * n + 2] << 8 | data_bottom_block1[0 + 2 * n + 3];

        // block 0
        data_pixel[28][n] = data_bottom_block0[192 + 2 * n + 2] << 8 | data_bottom_block0[192 + 2 * n + 3];
        data_pixel[29][n] = data_bottom_block0[128 + 2 * n + 2] << 8 | data_bottom_block0[128 + 2 * n + 3];
        data_pixel[30][n] = data_bottom_block0[64 + 2 * n + 2] << 8 | data_bottom_block0[64 + 2 * n + 3];
        data_pixel[31][n] = data_bottom_block0[0 + 2 * n + 2] << 8 | data_bottom_block0[0 + 2 * n + 3];

        // --- ELECTRICAL OFFSET ---
        // top half
        eloffset[0][n] = electrical_offset_top[2 * n + 2] << 8 | electrical_offset_top[2 * n + 3];
        eloffset[1][n] = electrical_offset_top[2 * (n + 32) + 2] << 8 | electrical_offset_top[2 * (n + 32) + 3];
        eloffset[2][n] = electrical_offset_top[2 * (n + 64) + 2] << 8 | electrical_offset_top[2 * (n + 64) + 3];
        eloffset[3][n] = electrical_offset_top[2 * (n + 96) + 2] << 8 | electrical_offset_top[2 * (n + 96) + 3];
        // bottom half
        eloffset[4][n] = electrical_offset_bottom[2 * (n + 96) + 2] << 8 | electrical_offset_bottom[2 * (n + 96) + 3];
        eloffset[5][n] = electrical_offset_bottom[2 * (n + 64) + 2] << 8 | electrical_offset_bottom[2 * (n + 64) + 3];
        eloffset[6][n] = electrical_offset_bottom[2 * (n + 32) + 2] << 8 | electrical_offset_bottom[2 * (n + 32) + 3];
        eloffset[7][n] = electrical_offset_bottom[2 * n + 2] << 8 | electrical_offset_bottom[2 * n + 3];
    }

    // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
    sum = ptat_top_block0 + ptat_top_block1 + ptat_top_block2 + ptat_top_block3 + ptat_bottom_block0 + ptat_bottom_block1 + ptat_bottom_block2 + ptat_bottom_block3;
    ptat_av_uint16 = sum / 8;
    // printf("ptat: %d %d %d %d %d %d %d %d\n",ptat_top_block0 , ptat_top_block1 , ptat_top_block2 , ptat_top_block3 , ptat_bottom_block0 , ptat_bottom_block1 , ptat_bottom_block2 , ptat_bottom_block3);
    // calculate ambient_temperature (datasheet, chapter: 11.1 Ambient Temperature )
    ambient_temperature = ptat_av_uint16 * ptatgr_float + ptatoff_float;
    // printf("ambient_temperature:%f ptat_av_uint16: %f \n", ambient_temperature, ptat_av_uint16);

    // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
    sum = vdd_top_block0 + vdd_top_block1 + vdd_top_block2 + vdd_top_block3 + vdd_bottom_block0 + vdd_bottom_block1 + vdd_bottom_block2 + vdd_bottom_block3;
    vdd_av_uint16 = sum / 8;
    // printf("vdd_av_uint16:%f \n", vdd_av_uint16);

    // printf("vdd: %d %d %d %d %d %d %d %d",vdd_top_block0 , vdd_top_block1 , vdd_top_block2 , vdd_top_block3 , vdd_bottom_block0 , vdd_bottom_block1 , vdd_bottom_block2 , vdd_bottom_block3);
}
void read_pixel_data()
{

    // --- BLOCK 0 with PTAT ---

    // change block in configuration register (to block0)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
    //   i2cSensorCommand(  CONFIGURATION_REGISTER, 0x09 );
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x09);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS);

    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);

    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }

    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block0, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block0, 258);

    // --- BLOCK 1 with PTAT ---

    // change block in configuration register (to block1)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  1  |   1   |    0     |   0   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x19);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    // ESP_LOGI(TAG, "read_pixel_data5");

    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }
    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block1, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block1, 258);

    // --- BLOCK 2 with PTAT ---

    // change block in configuration register (to block1)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  1  |  0  |   1   |    0     |   0   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x29);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }
    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block2, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block2, 258);

    // --- BLOCK 3 with PTAT ---

    // change block in configuration register (to block1)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  1  |  1  |   1   |    0     |   0   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x39);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }
    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block3, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block3, 258);

    // SAVE PTAT
    ptat_top_block0 = data_top_block0[0] << 8 | data_top_block0[1];
    ptat_top_block1 = data_top_block1[0] << 8 | data_top_block1[1];
    ptat_top_block2 = data_top_block2[0] << 8 | data_top_block2[1];
    ptat_top_block3 = data_top_block3[0] << 8 | data_top_block3[1];
    ptat_bottom_block0 = data_bottom_block0[0] << 8 | data_bottom_block0[1];
    ptat_bottom_block1 = data_bottom_block1[0] << 8 | data_bottom_block1[1];
    ptat_bottom_block2 = data_bottom_block2[0] << 8 | data_bottom_block2[1];
    ptat_bottom_block3 = data_bottom_block3[0] << 8 | data_bottom_block3[1];

    // --- BLOCK 0 with VDD ---

    // change block in configuration register (to block0)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   1   |    1     |   0   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x09 + 0x04);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }
    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block0, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block0, 258);

    // --- BLOCK 1 with VDD ---

    // change block in configuration register (to block1)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  1  |   1   |    1     |   0   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x19 + 0x04);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }
    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block1, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block1, 258);

    // --- BLOCK 2 with VDD ---

    // change block in configuration register (to block1)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  1  |  0  |   1   |    1     |   0   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x29 + 0x04);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }
    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block2, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block2, 258);

    // --- BLOCK 3 with VDD ---

    // change block in configuration register (to block1)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  1  |  1  |   1   |    1     |   0   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x39 + 0x04);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }
    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&data_top_block3, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&data_bottom_block3, 258);

    // SAVE VDD
    vdd_top_block0 = data_top_block0[0] << 8 | data_top_block0[1];
    vdd_top_block1 = data_top_block1[0] << 8 | data_top_block1[1];
    vdd_top_block2 = data_top_block2[0] << 8 | data_top_block2[1];
    vdd_top_block3 = data_top_block3[0] << 8 | data_top_block3[1];
    vdd_bottom_block0 = data_bottom_block0[0] << 8 | data_bottom_block0[1];
    vdd_bottom_block1 = data_bottom_block1[0] << 8 | data_bottom_block1[1];
    vdd_bottom_block2 = data_bottom_block2[0] << 8 | data_bottom_block2[1];
    vdd_bottom_block3 = data_bottom_block3[0] << 8 | data_bottom_block3[1];

    // --- EL.OFFSET ---

    // change block in configuration register (to block0)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
    i2cSensorCommand(CONFIGURATION_REGISTER, 0x0B);

    // wait for end of conversion bit (~27ms)
    vTaskDelay(30 / portTICK_PERIOD_MS); // poll when 90% done
    i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    while ((statusreg & 0x01) == 0)
    {
        i2cSensorReadCommand(STATUS_REGISTER, (uint8_t *)&statusreg, 1);
    }

    i2cSensorReadCommand(TOP_HALF, (uint8_t *)&electrical_offset_top, 258);
    i2cSensorReadCommand(BOTTOM_HALF, (uint8_t *)&electrical_offset_bottom, 258);
}
esp_err_t eeprom_read(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | WRITE_BIT, 1);
    i2c_master_write_byte(cmd, eeaddress >> 8, 1);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | READ_BIT, 1);

    if (size > 1)
    {
        i2c_master_read(cmd, data, size - 1, 0);
    }
    i2c_master_read_byte(cmd, data + size - 1, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
void read_eeprom()
{
    uint8_t b[3];
    bw = (read_EEPROM_byte(EEPROM_ADDRESS, E_BW2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_BW1)) / 100;
    id = read_EEPROM_byte(EEPROM_ADDRESS, E_ID4) << 24 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID3) << 16 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID1);
    mbit_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_MBIT_CALIB);
    bias_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_BIAS_CALIB);
    clk_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_CLK_CALIB);
    bpa_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_BPA_CALIB);
    pu_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_CALIB);
    // mbit_user = 12 16bit
    // bias_user = 5
    // clk_user = 21
    // pu_user = 136
    // bpa_user = 3
    // REFCAL = 2
    mbit_user = read_EEPROM_byte(EEPROM_ADDRESS, E_MBIT_USER);
    bias_user = read_EEPROM_byte(EEPROM_ADDRESS, E_BIAS_USER);
    clk_user = read_EEPROM_byte(EEPROM_ADDRESS, E_CLK_USER);
    bpa_user = read_EEPROM_byte(EEPROM_ADDRESS, E_BPA_USER);
    pu_user = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_USER);
    vddth1 = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH1_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH1_1);
    vddth2 = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH2_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH2_1);
    vddscgrad = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDSCGRAD);
    vddscoff = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDSCOFF);
    ptatth1 = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH1_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH1_1);
    ptatth2 = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH2_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH2_1);
    nrofdefpix = read_EEPROM_byte(EEPROM_ADDRESS, E_NROFDEFPIX);
    gradscale = read_EEPROM_byte(EEPROM_ADDRESS, E_GRADSCALE);
    tablenumber = read_EEPROM_byte(EEPROM_ADDRESS, E_TABLENUMBER2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_TABLENUMBER1);
    arraytype = read_EEPROM_byte(EEPROM_ADDRESS, E_ARRAYTYPE);
    b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_1);
    b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_2);
    b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_3);
    b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_4);

    ptatgr_float = *(float *)b;

    b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_1);
    b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_2);
    b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_3);
    b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_4);
    ptatoff_float = *(float *)b;

    b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_1);
    b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_2);
    b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_3);
    b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_4);
    pixcmin = *(float *)b;

    b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_1);
    b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_2);
    b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_3);
    b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_4);
    pixcmax = *(float *)b;

    epsilon = read_EEPROM_byte(EEPROM_ADDRESS, E_EPSILON);

    globaloff = read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALOFF);
    globalgain = read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALGAIN_1);

    // --- DeadPixAdr ---
    for (int i = 0; i < nrofdefpix; i++)
    {
        deadpixadr[i] = read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXADR + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXADR + 2 * i);
        if (deadpixadr[i] > 512)
        { // adaptedAdr:
            deadpixadr[i] = 1024 + 512 - deadpixadr[i] + 2 * (deadpixadr[i] % 32) - 32;
        }
    }
    // --- DeadPixMask ---
    for (int i = 0; i < nrofdefpix; i++)
    {
        deadpixmask[i] = read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXMASK + i);
    }

    // --- Thgrad_ij ---
    int m = 0;
    int n = 0;
    addr_i = 0x0740; // start address
    // top half
    for (int i = 0; i < 512; i++)
    {
        addr_i = 0x0740 + 2 * i;
        thgrad[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 2 * i);
        n++;
        if (n == 32)
        {
            n = 0;
            m++;
        }
    }
    // bottom half
    for (int i = 0; i < sensor.number_col; i++)
    {

        thgrad[31][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * i);
        thgrad[30][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 1 * 64 + 2 * i);
        thgrad[29][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * 64 + 2 * i);
        thgrad[28][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 3 * 64 + 2 * i);

        thgrad[27][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * i);
        thgrad[26][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 1 * 64 + 2 * i);
        thgrad[25][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * 64 + 2 * i);
        thgrad[24][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 3 * 64 + 2 * i);

        thgrad[23][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * i);
        thgrad[22][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 1 * 64 + 2 * i);
        thgrad[21][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * 64 + 2 * i);
        thgrad[20][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 3 * 64 + 2 * i);

        thgrad[19][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * i);
        thgrad[18][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 1 * 64 + 2 * i);
        thgrad[17][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * 64 + 2 * i);
        thgrad[16][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 3 * 64 + 2 * i);
    }

    // --- ThOffset_ij ---
    m = 0;
    n = 0;
    // top half
    for (int i = 0; i < 512; i++)
    {
        addr_i = 0x0F40 + 2 * i;
        thoffset[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 2 * i);
        n++;
        if (n == 32)
        {
            n = 0;
            m++;
        }
    }

    // bottom half
    for (int i = 0; i < sensor.number_col; i++)
    {
        thoffset[31][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * i);
        thoffset[30][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 1 * 64 + 2 * i);
        thoffset[29][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * 64 + 2 * i);
        thoffset[28][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 3 * 64 + 2 * i);

        thoffset[27][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * i);
        thoffset[26][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 1 * 64 + 2 * i);
        thoffset[25][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * 64 + 2 * i);
        thoffset[24][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 3 * 64 + 2 * i);

        thoffset[23][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * i);
        thoffset[22][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 1 * 64 + 2 * i);
        thoffset[21][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * 64 + 2 * i);
        thoffset[20][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 3 * 64 + 2 * i);

        thoffset[19][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * i);
        thoffset[18][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 1 * 64 + 2 * i);
        thoffset[17][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * 64 + 2 * i);
        thoffset[16][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 3 * 64 + 2 * i);
    }

    //---VddCompGrad---

    // top half
    for (int i = 0; i < sensor.number_col; i++)
    {
        // top half
        vddcompgrad[0][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (2 * i) + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (2 * i));
        vddcompgrad[1][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (1 * 64) + (2 * i) + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (1 * 64) + (2 * i));
        vddcompgrad[2][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (2 * 64) + (2 * i) + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (2 * 64) + (2 * i));
        vddcompgrad[3][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (3 * 64) + (2 * i) + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + (3 * 64) + (2 * i));
        // bottom half (backwards)
        vddcompgrad[7][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 4 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 4 * 64 + 2 * i);
        vddcompgrad[6][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 5 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 5 * 64 + 2 * i);
        vddcompgrad[5][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 6 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 6 * 64 + 2 * i);
        vddcompgrad[4][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 7 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 7 * 64 + 2 * i);
    }

    //---VddCompOff---

    // top half
    for (int i = 0; i < sensor.number_col; i++)
    {
        // top half
        vddcompoff[0][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * i);
        vddcompoff[1][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 1 * 64 + 2 * i);
        vddcompoff[2][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * 64 + 2 * i);
        vddcompoff[3][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 3 * 64 + 2 * i);
        // bottom half
        vddcompoff[7][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 4 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 4 * 64 + 2 * i);
        vddcompoff[6][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 5 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 5 * 64 + 2 * i);
        vddcompoff[5][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 6 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 6 * 64 + 2 * i);
        vddcompoff[4][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 7 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 7 * 64 + 2 * i);
    }

    // --- P_ij ---
    m = 0;
    n = 0;
    // top half
    for (int i = 0; i < 512; i++)
    {
        addr_i = 0x0F40 + 2 * i;
        pij[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 2 * i);
        n++;
        if (n == 32)
        {
            n = 0;
            m++;
        }
    }

    // bottom half
    for (int i = 0; i < sensor.number_col; i++)
    {
        pij[31][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * i);
        pij[30][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 1 * 64 + 2 * i);
        pij[29][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * 64 + 2 * i);
        pij[28][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 3 * 64 + 2 * i);

        pij[27][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * i);
        pij[26][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 1 * 64 + 2 * i);
        pij[25][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * 64 + 2 * i);
        pij[24][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 3 * 64 + 2 * i);

        pij[23][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * i);
        pij[22][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 1 * 64 + 2 * i);
        pij[21][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * 64 + 2 * i);
        pij[20][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 3 * 64 + 2 * i);

        pij[19][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * i);
        pij[18][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 1 * 64 + 2 * i);
        pij[17][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * 64 + 2 * i);
        pij[16][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 3 * 64 + 2 * i);
    }
}

void write_calibration_settings_to_sensor()
{

    // i2cSensorCommand(TRIM_REGISTER1, mbit_calib);
    i2cSensorCommand(TRIM_REGISTER1, mbit_calib);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    i2cSensorCommand(TRIM_REGISTER2, bias_calib);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    i2cSensorCommand(TRIM_REGISTER3, bias_calib);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    i2cSensorCommand(TRIM_REGISTER4, clk_calib);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    i2cSensorCommand(TRIM_REGISTER5, bpa_calib);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    i2cSensorCommand(TRIM_REGISTER6, bpa_calib);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    i2cSensorCommand(TRIM_REGISTER7, pu_calib);
}
void initHTPAData(){
     ESP_LOGI(TAG, "..... initHTPAData ....");
    i2c_frequency = CLOCK_EEPROM;
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    read_eeprom();
    i2c_driver_delete(i2c_port);
      i2c_frequency = CLOCK_SENSOR;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
     i2cSensorCommand(CONFIGURATION_REGISTER, 0x01);
    write_calibration_settings_to_sensor();
    gradscale_div = pow(2, gradscale);
    vddscgrad_div = pow(2, vddscgrad);
    vddscoff_div = pow(2, vddscoff);
    calculate_pixcij();
}
void captureHTPAData()
{
    ESP_LOGI(TAG, "..... captureHTPAData ....");
    // i2c_frequency = CLOCK_EEPROM;
    // i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    // i2c_master_driver_initialize();
    // read_eeprom();
    // i2c_driver_delete(i2c_port);

    // i2c_frequency = CLOCK_SENSOR;

    // i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    // i2c_master_driver_initialize();

    // i2cSensorCommand(CONFIGURATION_REGISTER, 0x01);
    // write_calibration_settings_to_sensor();
    // gradscale_div = pow(2, gradscale);
    // vddscgrad_div = pow(2, vddscgrad);
    // vddscoff_div = pow(2, vddscoff);
    // calculate_pixcij();
    if (tablenumber != TABLENUMBER)
    {
        printf("\n\nHINT:\tConnected sensor does not match the selected look up table.");
        printf("\n\tThe calculated temperatures could be wrong!");
        printf("\n\tChange device in sensordef_32x32.h to sensor with tablenumber ");
        printf("%u", tablenumber);
    }
    else
    {

        ESP_LOGI(TAG, "read_pixel_data: ");
        int trials = 0;
         while (trials < TRIAL_COUNT)
        {
            read_pixel_data();

            sort_data();

            calculate_pixel_temp();

            pixel_masking();
            // calc_average_temp();
             trials++;
        }
       

            print_pixel_temps_average();

                trials = 0;
      

        // i2cSensorCommand(CONFIGURATION_REGISTER, 0x00); // wakeup & start command
        //                                                 // print_eeprom_value();
        // i2c_driver_delete(i2c_port);

        xSemaphoreGive(HTPAReadySemaphore);

        //  vTaskDelay(3000 / portTICK_PERIOD_MS);
        //  xSemaphoreGive(HtpaSemaphore);
    }
}
void captureHTPAData_1()
{
    ESP_LOGI(TAG, "..... captureHTPAData ....");
    i2c_frequency = CLOCK_EEPROM;
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    read_eeprom();
    i2c_driver_delete(i2c_port);

    i2c_frequency = CLOCK_SENSOR;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    i2cSensorCommand(CONFIGURATION_REGISTER, 0x01);
    write_calibration_settings_to_sensor();
    gradscale_div = pow(2, gradscale);
    vddscgrad_div = pow(2, vddscgrad);
    vddscoff_div = pow(2, vddscoff);
    calculate_pixcij();
    if (tablenumber != TABLENUMBER)
    {
        printf("\n\nHINT:\tConnected sensor does not match the selected look up table.");
        printf("\n\tThe calculated temperatures could be wrong!");
        printf("\n\tChange device in sensordef_32x32.h to sensor with tablenumber ");
        printf("%u", tablenumber);
    }
    else
    {

        ESP_LOGI(TAG, "read_pixel_data: ");

        read_pixel_data();

        sort_data();

        calculate_pixel_temp();

        pixel_masking();

        print_pixel_temps();

        i2cSensorCommand(CONFIGURATION_REGISTER, 0x00); // wakeup & start command
                                                        // print_eeprom_value();
        i2c_driver_delete(i2c_port);

        xSemaphoreGive(HTPAReadySemaphore);

        //  vTaskDelay(3000 / portTICK_PERIOD_MS);
        //  xSemaphoreGive(HtpaSemaphore);
    }
}
