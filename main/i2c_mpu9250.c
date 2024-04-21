/*
    ESP32 code for the MPU-9250 using I2C to communicate (the MPU-9250 has a 3 axis accelerator, 3 axis gyro and a 3 axis magnetometer)
    This code prints out the values from all three MPU-9250 sensors to the console.

    RECOMMENDATION: Power the MPU-9250 with 5 volts from the ESP32 (the MPU-9250 magnetometer can freeze, if using 3.3 volts). 

    Link to the MPU-9250 module used for this code development:
        https://www.amazon.com/dp/B01I1J0Z7Y?psc=1&ref=ppx_yo2ov_dt_b_product_details

    MPU-9250 hardware specs:
        Power voltage: 3.3V to 5V   (I recommend using 5 volts)
        Communication mode: I2C     (SPI is also supported but NOT by this code)
        Gyro range: +/-250, +/-500, +/-1000, +/-2000dps
        Accelerator range: +/-2G, +/-4G, +/-8G, +/-16G
        Magnetometer range: +/-4800uF
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
// #include "driver/i2c_master.h"       //use this new driver instead of driver/i2c.h
#include "esp_system.h"

#define I2C_MASTER_SCL_IO    19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    18               /*!< gpio number for I2C master data  */

#define I2C_MASTER_NUM       I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   400000           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0         /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0         /*!< I2C master do not need buffer */
#define MPU9250_SENSOR_ADDR  0x68             /*!< Slave address of the MPU9250 sensor */
#define AK8963_SENSOR_ADDR   0x0C             /*!< Slave address of the AK8963 magnetometer */
#define WRITE_BIT            I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT             I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN         0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS        0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL              0x0              /*!< I2C ack value */
#define NACK_VAL             0x1              /*!< I2C nack value */

// MPU9250 and AK8963 Registers
#define MPU9250_REG_PWR_MGMT_1   0x6B
#define MPU9250_REG_ACCEL_XOUT_H 0x3B
#define MPU9250_REG_GYRO_XOUT_H  0x43
#define AK8963_REG_CNTL1         0x0A
#define AK8963_REG_HXL           0x03
#define AK8963_MODE_CONTINUOUS_2 0x06  /*!< 100Hz continuous measurement mode */
#define AK8963_BIT_16            0x10  /*!< Output 16-bit magnetometer data */

// Sensor register addresses within the MPU9250 and the AK8963 sensor (which is the magnetometer component of the MPU9250 system).
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define AK8963_CNTL1 0x0A


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Write a byte to the given register
 */
static esp_err_t mpu9250_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read a sequence of bytes from a sensor register
 */
static esp_err_t mpu9250_read_bytes(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    // Start a command link transaction for the address setting phase
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);  // Send the device address with the write option
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);                     // Send the register address
    i2c_master_stop(cmd);                                                   // Stop after writing the register address
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);  // Execute the command
    i2c_cmd_link_delete(cmd);                                               // Clean up command link

    if (ret != ESP_OK) {
        return ret;  // Return error if address setting phase failed
    }

    // Start another command link transaction for the reading phase
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | READ_BIT, ACK_CHECK_EN);  // Send the device address with the read option
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);                       // Read data bytes with ACK
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);                    // Read the last byte with NACK
    i2c_master_stop(cmd);                                                   // Stop after reading the data
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);  // Execute the command
    i2c_cmd_link_delete(cmd);                                               // Clean up command link

    return ret;  // Return the result of the reading phase
}

// static esp_err_t mpu9250_read_bytes(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
//     int i2c_master_port = I2C_MASTER_NUM;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, sensor_addr << 1 | READ_BIT, ACK_CHECK_EN);
//     if (len > 1) {
//         i2c_master_read(cmd, data, len - 1, ACK_VAL);
//     }
//     i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

/**
 * @brief Task to continuously read sensor data
 */
void mpu9250_task(void *arg) {
    uint8_t sensor_data[14];    // Buffer for accelerometer and gyroscope
    uint8_t mag_data[7];        // Buffer for magnetometer
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;

    // Initialize MPU9250
    esp_err_t ret = mpu9250_write_byte(MPU9250_SENSOR_ADDR, MPU9250_REG_PWR_MGMT_1, 0x00);  // Wake up the MPU9250
    if (ret != ESP_OK) {
        printf("1) Failed to initialize MPU9250 : %s\n", esp_err_to_name(ret));
    }
    // Set up the AK8963
    ret = mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, AK8963_BIT_16 | AK8963_MODE_CONTINUOUS_2);

    if (ret != ESP_OK) {
        printf("2) Failed to initialize MPU9250 : %s\n", esp_err_to_name(ret));
    }

    /*
    Converting Raw Sensor Data:
    To convert raw data from these sensors to standard units, you would typically do the following:

    Gyroscope: Convert ADC counts to degrees per second using the sensitivity setting (e.g., for a ±500 °/s setting, each count could represent 500/32768 degrees per second per count).
    Accelerometer: Convert ADC counts to G-forces or m/s² using the sensitivity setting (e.g., for a ±2g setting, each count could represent 2/32768 g per count).
    Magnetometer: Convert ADC counts to microteslas using a factory-calibrated sensitivity that can often be found in the sensor's datasheet or calibration registers.
    */
    while (1) {
        // Read accelerometer and gyroscope data
        mpu9250_read_bytes(MPU9250_SENSOR_ADDR, MPU9250_REG_ACCEL_XOUT_H, sensor_data, 14);

        // Parse accelerometer data
        accel_x = (sensor_data[0] << 8) | sensor_data[1];
        accel_y = (sensor_data[2] << 8) | sensor_data[3];
        accel_z = (sensor_data[4] << 8) | sensor_data[5];

        // Parse gyroscope data
        gyro_x = (sensor_data[8] << 8) | sensor_data[9];
        gyro_y = (sensor_data[10] << 8) | sensor_data[11];
        gyro_z = (sensor_data[12] << 8) | sensor_data[13];

        // Read magnetometer data
        mpu9250_read_bytes(AK8963_SENSOR_ADDR, AK8963_REG_HXL, mag_data, 7);

        // Parse magnetometer data (it is little endian) and convert from unsigned to signed values
        mag_x = (int16_t)((uint16_t)mag_data[1] << 8 | (uint16_t)mag_data[0]);
        mag_y = (int16_t)((uint16_t)mag_data[3] << 8 | (uint16_t)mag_data[2]);
        mag_z = (int16_t)((uint16_t)mag_data[5] << 8 | (uint16_t)mag_data[4]);

        // mag_x = (mag_data[1] << 8) | mag_data[0];  // Little endian
        // mag_y = (mag_data[3] << 8) | mag_data[2];  // Little endian
        // mag_z = (mag_data[5] << 8) | mag_data[4];  // Little endian

        printf("Accel: X=%6d, Y=%6d, Z=%6d, Gyro: X=%6d, Y=%6d, Z=%6d, Mag: X=%6d, Y=%6d, Z=%6d\n",
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


/**
 * @brief Set the full-scale range of the accelerometer. Accelerator range: +/-2G, +/-4G, +/-8G, +/-16G.
 * @param fs_sel Full-scale range selection (0=±2g, 1=±4g, 2=±8g, 3=±16g)
 * Default = 0 (±2g Full-Scale Range)
 *  0b00: ±2g
 *  0b01: ±4g
 *  0b10: ±8g
 *  0b11: ±16g
 * 
 * Example: set_accel_range(3); //sets +/-16g full scale range
 */
static esp_err_t set_accel_range(uint8_t fs_sel) {
    if (fs_sel > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = mpu9250_write_byte(MPU9250_SENSOR_ADDR, ACCEL_CONFIG, fs_sel << 3);
    if (ret != ESP_OK) {
        printf("Failed to set_accel_range: %s\n", esp_err_to_name(ret));
    }
    return ret;
}


/**
 * @brief Set the full-scale range of the gyroscope. Gyro range: +/-250, +/-500, +/-1000, +/-2000dps.
 * @param fs_sel Full-scale range selection (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)
 *  0b00: ±250 degrees per second
 *  0b01: ±500 degrees per second
 *  0b10: ±1000 degrees per second
 *  0b11: ±2000 degrees per second
 * 
 * Example: set_gyro_range(2);  //sets gyroscope to ±1000 degrees per second full scale range
 */
static esp_err_t set_gyro_range(uint8_t fs_sel) {
    if (fs_sel > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = mpu9250_write_byte(MPU9250_SENSOR_ADDR, GYRO_CONFIG, fs_sel << 3);
    if (ret != ESP_OK) {
        printf("Failed to set_gyro_range: %s\n", esp_err_to_name(ret));
    }
    return ret;

}


/**
 * @brief Set the mode of the AK8963 magnetometer. Magnetometer range: +/-4800uF
 * @param mode Magnetometer mode (e.g., 0x02 for single measurement, 0x06 for continuous mode 2)
 *  0x00: Power-down mode
 *  0x01: Single measurement mode
 *  0x02: Continuous measurement mode 1 (8 Hz)
 *  0x06: Continuous measurement mode 2 (100 Hz)
 *  0x08: External trigger measurement mode
 *  0x0F: Self-test mode
 *  0x0B: Fuse ROM access mode
 * 
 * Example: set_ak8963_mode(0x06);  //set the AK8963 to continuous measurement mode 2
 */
static esp_err_t set_ak8963_mode(uint8_t mode) {

    // Reset AK8963
    mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, 0x01); // Reset device
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to allow reset to complete
    // Reinitialize AK8963
    esp_err_t ret = mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, AK8963_BIT_16 | AK8963_MODE_CONTINUOUS_2);

    // esp_err_t ret = mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_CNTL1, mode);
    if (ret != ESP_OK) {
        printf("Failed to set_ak8963_mode: %s\n", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * Enables the magnetometer bypass mode
 */
static esp_err_t enable_bypass(void) {
    return mpu9250_write_byte(MPU9250_SENSOR_ADDR, 0x37, 0x02);  // INT_PIN_CFG register set to enable bypass mode
}


void app_main(void) {
    // Initialize the I2C bus for communication
    i2c_master_init();

    // Set accelerometer to default range ±2g
    set_accel_range(0);  // 0b00 corresponds to ±2g

    // Set gyroscope to default range ±250 degrees/s
    set_gyro_range(0);  // 0b00 corresponds to ±250°/s

    // Set AK8963 to default mode (Continuous Measurement Mode 1 at 8 Hz)
    enable_bypass(); // Call this after initializing the I2C and before setting the AK8963 mode
    set_ak8963_mode(AK8963_MODE_CONTINUOUS_2);  

    // Create the task to handle MPU9250 data reading and processing
    xTaskCreate(mpu9250_task, "mpu9250_task", 4096, NULL, 10, NULL);
}
