/*
    ESP32 code for the MPU-9250 using I2C to communicate (the MPU-9250 has a 3 axis accelerator, 3 axis gyro and a 3 axis magnetometer)
    This code prints out the values from all three MPU-9250 sensors to the console.
    Note that there is no magnetometer calibration code. Therefore, the accuracy of magnetometer sensor data is limited. 

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
// #include "driver/i2c_master.h"           //the newer I2C driver
#include "esp_system.h"
#include <math.h>

// Magnetic declination in decimal degrees (adjust this value based on your location)
const float magnetic_declination = 12.85;      //in San Jose, CA

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


/**
 * @brief Set the full-scale range of the accelerometer.
 * @param fs_sel Full-scale range selection (0=±2g, 1=±4g, 2=±8g, 3=±16g)
 * Example: set_accel_range(3); // Sets to ±16g full-scale range
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


// Function to get accelerometer sensitivity
float get_accel_sensitivity() {
    uint8_t accel_sensitivity_setting;
    mpu9250_read_bytes(MPU9250_SENSOR_ADDR, ACCEL_CONFIG, &accel_sensitivity_setting, 1);

    switch (accel_sensitivity_setting & 0x18) {
        case 0x00: return 2.0 / 32768.0;
        case 0x08: return 4.0 / 32768.0;
        case 0x10: return 8.0 / 32768.0;
        case 0x18: return 16.0 / 32768.0;
        default:   return 2.0 / 32768.0;  // Default sensitivity
    }
}


/**
 * @brief Set the full-scale range of the gyroscope.
 * @param fs_sel Full-scale range selection (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)
 * Example: set_gyro_range(2); // Sets gyroscope to ±1000°/s full-scale range
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

// Function to get gyroscope sensitivity
float get_gyro_sensitivity() {
    uint8_t gyro_sensitivity_setting;
    mpu9250_read_bytes(MPU9250_SENSOR_ADDR, GYRO_CONFIG, &gyro_sensitivity_setting, 1);

    switch (gyro_sensitivity_setting & 0x18) {
        case 0x00:  // ±250 °/s
            return 250.0 / 32768.0;
        case 0x08:  // ±500 °/s
            return 500.0 / 32768.0;
        case 0x10:  // ±1000 °/s
            return 1000.0 / 32768.0;
        case 0x18:  // ±2000 °/s
            return 2000.0 / 32768.0;
        default:
            return 250.0 / 32768.0;  // Default sensitivity
    }
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
 * @param high_reolution - boolean set to  true for 16 bit resolution, else set to 14 bit
 * 
 * Example: set_ak8963_mode(0x06, true);  //set the AK8963 to continuous measurement mode 2 at 16 bit resolution
 */
static esp_err_t set_mag_mode_and_resolution(uint8_t mode, bool high_resolution) {
    uint8_t mode_and_resolution = mode | 0x00; // Default to 14-bit resolution
    if (high_resolution) {
        mode_and_resolution = mode | 0x10; // Set to 16-bit resolution
    }

    // Reset AK8963
    mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, 0x01); // Reset device
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to allow reset to complete

    // Set mode and resolution
    esp_err_t ret = mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, mode_and_resolution);
    if (ret != ESP_OK) {
        printf("Failed to set_mag_mode_and_resolution: %s\n", esp_err_to_name(ret));
    }
    return ret;
}

// static esp_err_t set_mag_mode(uint8_t mode) {
//     // Reset AK8963
//     mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, 0x01); // Reset device
//     vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to allow reset to complete
//     // Set mode after reset
//     esp_err_t ret = mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, mode);
//     if (ret != ESP_OK) {
//         printf("Failed to set_mag_mode: %s\n", esp_err_to_name(ret));
//     }
//     return ret;
// }


/////////
//NOTE: There is no magnetometer set_range() function because the AK8963 operates at a fixed range of ±4800 µT (microteslas).
/////////

// Function to get magnetometer sensitivity
float get_mag_sensitivity() {
    uint8_t resolution_setting;
    mpu9250_read_bytes(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, &resolution_setting, 1);

    // Determine if using 16-bit output (0x10) or defaulting to 14-bit
    if ((resolution_setting & 0x10) == 0x10) {
        return 0.15; // Sensitivity for 16-bit output in µT/LSB
    } else {
        return 0.6; // Sensitivity for 14-bit output in µT/LSB (default)
    }
}

// float get_mag_sensitivity() {
//     uint8_t mag_sensitivity_setting;
//     mpu9250_read_bytes(AK8963_SENSOR_ADDR, AK8963_CNTL1, &mag_sensitivity_setting, 1);

//     return (mag_sensitivity_setting - 128.0) / 256.0 + 1.0;
// }

/**
 * Enables the magnetometer bypass mode
 */
static esp_err_t enable_mag_bypass(void) {
    return mpu9250_write_byte(MPU9250_SENSOR_ADDR, 0x37, 0x02);  // INT_PIN_CFG register set to enable bypass mode
}


/**
 * @brief Task to continuously read and print out the MPU9250 sensor data
 * 
 * Euler angles:
 * Heading (or Yaw): Compass direction where the device is pointing horizontally.
 * Elevation (or Pitch): Angle of tilt up or down relative to the Earth's surface, pointing the nose up or down.
 * Bank (or Roll): Angle of rotation about the device's forward (x) axis, which is the left or right tilt.
 * 
 */
void mpu9250_task(void *arg) {
    uint8_t sensor_data[14];    // Buffer for accelerometer and gyroscope
    uint8_t mag_data[7];        // Buffer for magnetometer
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;
    
    float accel_sensitivity = get_accel_sensitivity();
    float gyro_sensitivity = get_gyro_sensitivity();

    while (1) {
        // Read accelerometer, gyroscope, and magnetometer data
        mpu9250_read_bytes(MPU9250_SENSOR_ADDR, MPU9250_REG_ACCEL_XOUT_H, sensor_data, 14);
        mpu9250_read_bytes(AK8963_SENSOR_ADDR, AK8963_REG_HXL, mag_data, 7);

        // Parse data from sensors
        accel_x = (sensor_data[0] << 8) | sensor_data[1];
        accel_y = (sensor_data[2] << 8) | sensor_data[3];
        accel_z = (sensor_data[4] << 8) | sensor_data[5];
        gyro_x = (sensor_data[8] << 8) | sensor_data[9];
        gyro_y = (sensor_data[10] << 8) | sensor_data[11];
        gyro_z = (sensor_data[12] << 8) | sensor_data[13];
        mag_x = (int16_t)((uint16_t)mag_data[1] << 8 | (uint16_t)mag_data[0]);
        mag_y = (int16_t)((uint16_t)mag_data[3] << 8 | (uint16_t)mag_data[2]);
        mag_z = (int16_t)((uint16_t)mag_data[5] << 8 | (uint16_t)mag_data[4]);

        // Convert raw data to logical values with units
        float accel_x_g = accel_x * accel_sensitivity;
        float accel_y_g = accel_y * accel_sensitivity;
        float accel_z_g = accel_z * accel_sensitivity;
        float gyro_x_dps = gyro_x * gyro_sensitivity; // Angular velocity in degrees per second
        float gyro_y_dps = gyro_y * gyro_sensitivity; // Angular velocity in degrees per second
        float gyro_z_dps = gyro_z * gyro_sensitivity; // Angular velocity in degrees per second

        // Calculate roll and pitch
        float roll = atan2(accel_y_g, accel_z_g) * (180.0 / M_PI);
        float pitch = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * (180.0 / M_PI);

        // Calculate tilt compensated magnetic field components
        float mag_x_comp = mag_x * cos(pitch) + mag_z * sin(pitch);
        float mag_y_comp = mag_x * sin(roll) * sin(pitch) + mag_y * cos(roll) - mag_z * sin(roll) * cos(pitch);

        // Calculate heading
        float heading = atan2(-mag_y_comp, mag_x_comp) * (180.0 / M_PI);
        heading += magnetic_declination; // Adjust heading for local magnetic declination
        if (heading > 360) heading -= 360;
        if (heading < 0) heading += 360; // Normalize heading between 0 and 360 degrees

        printf("Accel: X=%.2f G, Y=%.2f G, Z=%.2f G, Gyro: X=%.2f deg/s, Y=%.2f deg/s, Z=%.2f deg/s, Heading=%.2f°, Pitch=%.2f°, Roll=%.2f°\n",
               accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps, heading, pitch, roll);
                
        // float mag_sensitivity = get_mag_sensitivity();  // Each LSB corresponds to 0.15 µT typical value for AK8963 in 16-bit output mode
        // float mag_x_uT = mag_x * mag_sensitivity; // Magnetic field strength in microtesla
        // float mag_y_uT = mag_y * mag_sensitivity; // Magnetic field strength in microtesla
        // float mag_z_uT = mag_z * mag_sensitivity; // Magnetic field strength in microtesla
        // printf("Accel: X=%.2f G, Y=%.2f G, Z=%.2f G, Gyro: X=%.2f deg/s, Y=%.2f deg/s, Z=%.2f deg/s\n",
        //        accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
        // printf("Mag: X=%.2f uT, Y=%.2f uT, Z=%.2f uT, Heading=%.2f°, Elevation=%.2f°, Bank=%.2f°\n",
        //        mag_x_uT, mag_y_uT, mag_z_uT, heading, pitch, roll);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}



void app_main(void) {
    // Initialize the I2C bus for communication
    i2c_master_init();

    // Set accelerometer to default range ±2g
    set_accel_range(0);  // 0b00 corresponds to ±2g

    // Set gyroscope to default range ±250 degrees/s
    set_gyro_range(0);  // 0b00 corresponds to ±250°/s

    // Set magnetometer
    enable_mag_bypass(); // Call this after initializing the I2C and before setting the AK8963 mode
    // Set AK8963 to Continuous Measurement Mode 2
    set_mag_mode_and_resolution(AK8963_MODE_CONTINUOUS_2, true);   //true means high resolution (16 bit)

    // Create the task to handle MPU9250 data reading and processing
    xTaskCreate(mpu9250_task, "mpu9250_task", 4096, NULL, 10, NULL);
}
