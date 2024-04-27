/*
    ESP32 code for the MPU-9250 using I2C to communicate (the MPU-9250 has a 3 axis accelerator, 
    3 axis gyro and a 3 axis magnetometer)
    
    This code prints out the values from all three MPU-9250 sensors to the console.

    There is NO code for sensor fusion or to properly handle dynamic/acceleration effects. 

    I have recently added code to allow you to perform a sensor calibration procedure. However, 
    the accuracy of the magnetic heading results are still poor. 

    POWER RECOMMENDATION: Power the MPU-9250 with 5 volts from the ESP32 (the MPU-9250 magnetometer 
    can freeze, if using 3.3 volts). 

    Link to the ESP32 module used:
        https://www.amazon.com/gp/product/B0718T232Z/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1

    Link to the MPU-9250 module used:
        https://www.amazon.com/dp/B01I1J0Z7Y?psc=1&ref=ppx_yo2ov_dt_b_product_details

    MPU-9250 hardware specs:
        Power voltage: 3.3V to 5V   (I recommend using 5 volts)
        Communication mode: I2C     (SPI is also supported but NOT by this code)
        Gyro range: +/-250, +/-500, +/-1000, +/-2000dps
        Accelerator range: +/-2G, +/-4G, +/-8G, +/-16G
        Magnetometer range: +/-4800uF


    ***NOTE: Understanding Magnetometer Bypass Mode

        The MPU-9250 is a complex module that integrates a 3-axis accelerometer, a 3-axis gyroscope, 
        and a 3-axis magnetometer (AK8963). The accelerometer and gyroscope are directly accessible 
        via the primary I2C bus of the MPU-9250, but the AK8963 magnetometer communicates through 
        a secondary I2C interface that is typically managed by the MPU-9250's main controller.

        Here’s why the `enable_mag_bypass()` function is necessary:

        1. Bypass Enable**: By writing `0x02` to the `INT_PIN_CFG` register (`0x37`) of the MPU-9250, 
        you activate the bypass mode. This mode configures the MPU-9250 to pass through the I2C master 
        signals directly to the I2C slave bus. This is essential because it allows the external 
        microcontroller (in your case, the ESP32) to communicate directly with the AK8963 magnetometer.

        2. Direct Communication**: Without enabling bypass mode, the AK8963 magnetometer is not accessible 
        on the primary I2C bus used by the MPU-9250. Enabling bypass mode effectively disconnects the 
        MPU-9250 from acting as a master controller to the AK8963, allowing direct I2C communication from 
        your main microcontroller to the magnetometer.

        3. Operational Necessity**: If you don’t activate this bypass mode, any attempts to communicate 
        with the AK8963 will fail because the MPU-9250 will not forward your I2C commands from the primary 
        to the secondary I2C bus where the AK8963 resides. Consequently, you won’t be able to configure 
        the magnetometer, read its data, or perform any other operation on the AK8963.

        ### Practical Implications:
        Enabling bypass mode is particularly important in setups where you need fine control over the 
        magnetometer or when you are using libraries or code that assume direct access to all three sensors 
        (accelerometer, gyroscope, and magnetometer) via the same I2C bus. This is a common scenario in 
        DIY projects, robotics, and other applications involving orientation tracking or navigation.

        Therefore, calling `enable_mag_bypass()` is not just a supplementary step; it's a fundamental 
        requirement to ensure that your MPU-9250’s magnetometer provides usable data.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
// #include "driver/i2c_master.h"           //the newer I2C driver 
#include "esp_system.h"
#include <math.h>
#include "esp_log.h"
#include <string.h>

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

// MPU9250 Registers 
#define MPU9250_REG_PWR_MGMT_1   0x6B
#define MPU9250_REG_ACCEL_XOUT_H 0x3B
#define MPU9250_REG_GYRO_XOUT_H  0x43

#define GYRO_RANGE_250_DPS          0
#define GYRO_RANGE_500_DPS          1
#define GYRO_RANGE_1000_DPS         2
#define GYRO_RANGE_2000_DPS         3

#define ACCELEROMETER_RANGE_2G      0      
#define ACCELEROMETER_RANGE_4G      1      
#define ACCELEROMETER_RANGE_8G      2      
#define ACCELEROMETER_RANGE_16G     3

//AK8963 Registers (Magnetometer)
#define AK8963_REG_CNTL1         0x0A
#define AK8963_REG_HXL           0x03
#define AK8963_BIT_14            0x00  /*!< Output 14-bit magnetometer data */
#define AK8963_BIT_16            0x10  /*!< Output 16-bit magnetometer data */
//AK8963 Modes
#define AK8963_MODE_POWER_DOWN              0x00    //Power-down mode
#define AK8963_MODE_SINGLE_MEASURE          0x01    //Single measurement mode
#define AK8963_MODE_CONTINUOUS_8_HZ         0x02    //Continuous measurement mode 1 (8 Hz)
#define AK8963_MODE_CONTINUOUS_100_HZ       0x06    //Continuous measurement mode 2 (100 Hz)
#define AK8963_MODE_EXTERNAL_TRIGGER        0x08    //External trigger measurement mode
#define AK8963_MODE_SELF_TEST               0x0F    //Self-test mode
#define AK8963_MODE_FUSE_ROM_ACCESS         0x0B    //Fuse ROM access mode


// Sensor register addresses within the MPU9250 and the AK8963 sensor (which is the magnetometer component of the MPU9250 system).
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define AK8963_CNTL1 0x0A

//used during magnetometer calibration to assure sufficient range coverage
#define NUM_LATITUDE_BINS 6
#define NUM_LONGITUDE_BINS 12
#define MIN_POINTS_PER_BIN 10

//=============================================================
//================== CONFIGURATION CONSTANTS ==================  
//=============================================================

static const bool DEBUG = true;

static const bool REQUIRE_CALIBRATION               = true;

// Magnetic declination in decimal degrees (adjust this value based on your location)
static const float CONFIG_MAGNETIC_DECLINATION      = 0.0;  //12.85; //in San Jose, CA

static const float CONFIG_MAGNETOMETER_RESOLUTION   = AK8963_BIT_16;

static const float CONFIG_MAGNETOMETER_MODE         = AK8963_MODE_CONTINUOUS_100_HZ;

static const float CONFIG_ACCELEROMETER_RANGE       = ACCELEROMETER_RANGE_2G;

static const float CONFIG_GYRO_RANGE                = GYRO_RANGE_250_DPS;

static const char *TAG = "SensorTask";


//struct to hold sensor calibration data
typedef struct {
    int16_t accel_offset[3];
    int16_t gyro_offset[3];
    int16_t mag_offset[3];
    float mag_scale[3];  // Use float for scale to handle fractional values
} SensorCalibration;

// Utility function to determine which bin a vector belongs to based on its spherical coordinates
void get_spherical_bin(int16_t x, int16_t y, int16_t z, int *lat_bin, int *long_bin) {
    double r = sqrt(x*x + y*y + z*z);
    double theta = acos(z / r); // polar angle
    double phi = atan2(y, x); // azimuthal angle

    // Normalize angles into bin indices
    *lat_bin = (int)(NUM_LATITUDE_BINS * theta / M_PI);
    *long_bin = (int)(NUM_LONGITUDE_BINS * (phi + M_PI) / (2 * M_PI));

    // if (DEBUG){
    //     printf("mx: %d, my: %d, mz: %d, r: %f, theta: %f, phi: %f, lat_bin: %d, long_bin: %d\n",
    //            x, y, z, r, theta, phi, *lat_bin, *long_bin);            
    // }

}

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


/** 
 * Function to get accelerometer sensitivity
 */ 
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

/**
 * Function to get gyroscope sensitivity
 */ 
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
static esp_err_t set_mag_mode_and_resolution(uint8_t mode, uint8_t resolution) {
    // Combine mode and resolution into one byte
    uint8_t mode_and_resolution = mode | resolution;

    // Reset AK8963
    esp_err_t ret = mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, 0x01); // Reset device
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset AK8963: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to allow reset to complete

    // Set mode and resolution
    ret = mpu9250_write_byte(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, mode_and_resolution);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set mode and resolution: %s", esp_err_to_name(ret));
    }
    return ret;
}

/////////
//NOTE: There is no magnetometer set_range() function because the AK8963 operates at a fixed range of ±4800 µT (microteslas).
/////////

/** 
 * Function to get the magnetometer sensitivity in µT/LSB
 * 
 * For the AK8963, the sensitivity values are determined through factory calibration 
 * and are specified in the datasheet as constants for each mode:
 *  16-bit mode: 0.15 microteslas per least significant bit (µT/LSB)
 *  14-bit mode: 0.6 microteslas per least significant bit (µT/LSB)
*/
float get_mag_sensitivity() {
    uint8_t resolution_setting;
    mpu9250_read_bytes(AK8963_SENSOR_ADDR, AK8963_REG_CNTL1, &resolution_setting, 1);

    // Determine if using 16-bit output (0x10) or defaulting to 14-bit
    if ((resolution_setting & AK8963_BIT_16) == AK8963_BIT_16) {
        return 0.15; // Sensitivity for 16-bit output in µT/LSB
    } else {
        return 0.6; // Sensitivity for 14-bit output in µT/LSB (default)
    }
}

/**
 * Enables the magnetometer bypass mode
 */
static esp_err_t enable_mag_bypass(void) {
    // activate bypass mode by writing `0x02` to `INT_PIN_CFG` register (`0x37`) of the MPU-9250 
    return mpu9250_write_byte(MPU9250_SENSOR_ADDR, 0x37, 0x02);
}

/**
 * @brief function that continuously gets sensor data, converts it to logical data, and prints it on the console
 */
void mpu9250_task(void *calibration) {
    SensorCalibration *calib = (SensorCalibration *)calibration;
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

        // Parse and calibrate data from sensors
        accel_x = ((sensor_data[0] << 8) | sensor_data[1]) - calib->accel_offset[0];
        accel_y = ((sensor_data[2] << 8) | sensor_data[3]) - calib->accel_offset[1];
        accel_z = ((sensor_data[4] << 8) | sensor_data[5]) - calib->accel_offset[2];
        gyro_x = ((sensor_data[8] << 8) | sensor_data[9]) - calib->gyro_offset[0];
        gyro_y = ((sensor_data[10] << 8) | sensor_data[11]) - calib->gyro_offset[1];
        gyro_z = ((sensor_data[12] << 8) | sensor_data[13]) - calib->gyro_offset[2];
        mag_x = (((int16_t)((uint16_t)mag_data[1] << 8 | (uint16_t)mag_data[0])) - calib->mag_offset[0]) * calib->mag_scale[0];
        mag_y = (((int16_t)((uint16_t)mag_data[3] << 8 | (uint16_t)mag_data[2])) - calib->mag_offset[1]) * calib->mag_scale[1];
        mag_z = (((int16_t)((uint16_t)mag_data[5] << 8 | (uint16_t)mag_data[4])) - calib->mag_offset[2]) * calib->mag_scale[2];

        // Convert raw data to logical values with units
        float accel_x_g = accel_x * accel_sensitivity;
        float accel_y_g = accel_y * accel_sensitivity;
        float accel_z_g = accel_z * accel_sensitivity;
        float gyro_x_dps = gyro_x * gyro_sensitivity;
        float gyro_y_dps = gyro_y * gyro_sensitivity;
        float gyro_z_dps = gyro_z * gyro_sensitivity;

        // Calculate roll and pitch
        float roll = atan2(accel_y_g, accel_z_g) * (180.0 / M_PI);
        float pitch = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * (180.0 / M_PI);

        // Calculate tilt compensated magnetic field components
        float mag_x_comp = mag_x * cos(pitch) + mag_z * sin(pitch);
        float mag_y_comp = mag_x * sin(roll) * sin(pitch) + mag_y * cos(roll) - mag_z * sin(roll) * cos(pitch);

        // Calculate heading using tilt compensated values
        float heading = atan2(-mag_y_comp, mag_x_comp) * (180.0 / M_PI);
        heading += CONFIG_MAGNETIC_DECLINATION; // Adjust heading for local magnetic declination
        if (heading > 360) heading -= 360;
        if (heading < 0) heading += 360; // Normalize heading between 0 and 360 degrees

        printf("Accel: X=%7.2f G, Y=%7.2f G, Z=%7.2f G   |   Gyro: X=%7.2f deg/s, Y=%7.2f deg/s, Z=%7.2f deg/s   |   Heading=%7.2f°, Pitch=%7.2f°, Roll=%7.2f°\n",
               accel_x_g, accel_y_g, accel_z_g, 
               gyro_x_dps, gyro_y_dps, gyro_z_dps, 
               heading, pitch, roll);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void calibrate_magnetometer(SensorCalibration *calib) {

    ESP_LOGI(TAG, "=======================================================================================");
    ESP_LOGI(TAG, "Rotate the sensor device through its full range of motion for magnetometer calibration.");
    ESP_LOGI(TAG, "=======================================================================================");
    
    int bins[NUM_LATITUDE_BINS][NUM_LONGITUDE_BINS] = {0};
    int16_t mag_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t mag_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
    uint8_t sensor_data[7];
    bool sufficient_coverage = false;
    int total_samples = 0;

    while (!sufficient_coverage) {
        mpu9250_read_bytes(AK8963_SENSOR_ADDR, AK8963_REG_HXL, sensor_data, 7);
        int16_t mx = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
        int16_t my = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
        int16_t mz = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);

        if (DEBUG){
            printf("Raw MAG Data - mx: %d, my: %d, mz: %d\n", mx, my, mz);
        }

        // Update min/max
        if (mx < mag_min[0]) mag_min[0] = mx;
        if (my < mag_min[1]) mag_min[1] = my;
        if (mz < mag_min[2]) mag_min[2] = mz;
        if (mx > mag_max[0]) mag_max[0] = mx;
        if (my > mag_max[1]) mag_max[1] = my;
        if (mz > mag_max[2]) mag_max[2] = mz;

        int lat_bin, long_bin;
        get_spherical_bin(mx, my, mz, &lat_bin, &long_bin);
        bins[lat_bin][long_bin]++;
        total_samples++;

        // Check for sufficient coverage
        sufficient_coverage = true;
        for (int i = 0; i < NUM_LATITUDE_BINS; i++) {
            for (int j = 0; j < NUM_LONGITUDE_BINS; j++) {
                if (bins[i][j] < MIN_POINTS_PER_BIN) {
                    sufficient_coverage = false;
                    break;
                }
            }
            if (!sufficient_coverage) break;
        }
        
        printf("\rTotal samples: %d, Segment (%d,%d) count: %d", total_samples, lat_bin, long_bin, bins[lat_bin][long_bin]);
        fflush(stdout);

        if (total_samples % 50 == 0) {
            printf("\nChecking coverage... ");
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Delay to limit sample rate


        // ESP_LOGI(TAG, "=======================================================================================");
        // ESP_LOGI(TAG, "Rotate the sensor device through its full range of motion for magnetometer calibration.");
        // ESP_LOGI(TAG, "=======================================================================================");

        // int mag_samples = 500;
        // for (int i = 0; i < mag_samples; i++) {
        //     mpu9250_read_bytes(AK8963_SENSOR_ADDR, AK8963_REG_HXL, sensor_data, 7);
        //     int16_t mx = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
        //     int16_t my = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
        //     int16_t mz = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);

        //     mag_total[0] += mx;
        //     mag_total[1] += my;
        //     mag_total[2] += mz;

        //     if (mx < mag_min[0]) mag_min[0] = mx;
        //     if (my < mag_min[1]) mag_min[1] = my;
        //     if (mz < mag_min[2]) mag_min[2] = mz;

        //     if (mx > mag_max[0]) mag_max[0] = mx;
        //     if (my > mag_max[1]) mag_max[1] = my;
        //     if (mz > mag_max[2]) mag_max[2] = mz;

        //     int countdown = mag_samples - i; // Decrementing countdown
        //     if (countdown % 10 == 0){
        //         printf("\rCalibration samples remaining: %d    ", countdown);
        //         fflush(stdout); // Ensure the buffer is flushed immediately         
        //     }

        //     vTaskDelay(pdMS_TO_TICKS(50));
        // }
        // printf("\n"); // Add a newline after the loop completes
    }

    // Compute calibration parameters
    for (int i = 0; i < 3; i++) {
        calib->mag_offset[i] = (mag_min[i] + mag_max[i]) / 2; // Midpoint offset
        int16_t range = mag_max[i] - mag_min[i];
        calib->mag_scale[i] = (range > 0) ? (4912.0 / range) : 0; // Example Earth's magnetic field strength in nT
    }

    printf("\nCalibration complete with sufficient coverage. Offsets: %d, %d, %d\n",
           calib->mag_offset[0], calib->mag_offset[1], calib->mag_offset[2]);
}

/**
 * Function that gets sensor data during calibration procedures. 
 * This data is used to calculate sensor offsets and scaling factors
 * and is used to make subsequent sensor corrections at runtime. 
 */
void calibrate_sensors(SensorCalibration *calib) {
    ESP_LOGI(TAG, "====================================================================================");
    ESP_LOGI(TAG, "Orient the sensor device with the x-y plane level and keep it still for calibration.");
    ESP_LOGI(TAG, "====================================================================================");

    uint8_t sensor_data[14];
    int32_t accel_total[3] = {0};
    int32_t gyro_total[3] = {0};
    // int32_t mag_total[3] = {0};
    int16_t mag_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t mag_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};

    int samples = 100;

    for (int i = 0; i < samples; i++) {
        mpu9250_read_bytes(MPU9250_SENSOR_ADDR, MPU9250_REG_ACCEL_XOUT_H, sensor_data, 14);
        accel_total[0] += (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
        accel_total[1] += (int16_t)((sensor_data[2] << 8) | sensor_data[3]);
        accel_total[2] += (int16_t)((sensor_data[4] << 8) | sensor_data[5]);

        gyro_total[0] += (int16_t)((sensor_data[8] << 8) | sensor_data[9]);
        gyro_total[1] += (int16_t)((sensor_data[10] << 8) | sensor_data[11]);
        gyro_total[2] += (int16_t)((sensor_data[12] << 8) | sensor_data[13]);

        int countdown = samples - i; // Decrementing countdown
        if (countdown % 10 == 0){
            printf("\rCalibration samples remaining: %d    ", countdown);
            fflush(stdout); // Ensure the buffer is flushed immediately         
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("\n"); // Add a newline after the loop completes

    //============================
    calibrate_magnetometer(calib);
    //============================

    //the MPU9250 has a 16 bit ADC (32768 sensor counts)
    int acceleromter_range =  pow(2.0, (CONFIG_ACCELEROMETER_RANGE + 1));
    int16_t lsb_per_g = 32768 / acceleromter_range;  // Calculate LSB per G based on configured range

    calib->accel_offset[0] = accel_total[0] / samples;
    calib->accel_offset[1] = accel_total[1] / samples;
    calib->accel_offset[2] = accel_total[2] / samples - lsb_per_g;  // Adjust for 1G in z-axis

    calib->gyro_offset[0] = gyro_total[0] / samples;
    calib->gyro_offset[1] = gyro_total[1] / samples;
    calib->gyro_offset[2] = gyro_total[2] / samples;

    // Calculate magnetometer scale factors based on resolution
    float base_range = (CONFIG_MAGNETOMETER_RESOLUTION == AK8963_BIT_16) ? 32768.0f : 8192.0f;
    for (int i = 0; i < 3; i++) {
        calib->mag_offset[i] = (mag_min[i] + mag_max[i]) / 2;
        int16_t range = mag_max[i] - mag_min[i];
        calib->mag_scale[i] = (range > 0) ? (base_range / range) : 0;
    }

    ESP_LOGI(TAG, "Calibration complete. Offsets: Accel(%d, %d, %d), Gyro(%d, %d, %d), Mag(%d, %d, %d)",
             calib->accel_offset[0], calib->accel_offset[1], calib->accel_offset[2],
             calib->gyro_offset[0], calib->gyro_offset[1], calib->gyro_offset[2],
             calib->mag_offset[0], calib->mag_offset[1], calib->mag_offset[2]);
}



void app_main(void) {
    // Initialize the I2C bus for communication
    i2c_master_init();

    // Set accelerometer to default range ±2g
    set_accel_range(CONFIG_ACCELEROMETER_RANGE);  // 0b00 corresponds to ±2g

    // Set gyroscope to default range ±250 degrees/s
    set_gyro_range(CONFIG_GYRO_RANGE);  // 0b00 corresponds to ±250°/s

    // Set magnetometer bypass mode (***NOTE: See the note below this code for a further explanation
    enable_mag_bypass(); // Call this after initializing the I2C and before setting the AK8963 mode

    // Set the magnetometer mode and bit depth (resolution)
    set_mag_mode_and_resolution(CONFIG_MAGNETOMETER_MODE, CONFIG_MAGNETOMETER_RESOLUTION);

    SensorCalibration calibrationData;
    memset(&calibrationData, 0, sizeof(SensorCalibration)); // Initialize to zero
    if (REQUIRE_CALIBRATION){
        calibrate_sensors(&calibrationData); // Call the calibration function
    } else {
        ESP_LOGI(TAG, "Skipping Calibration");
    }

    mpu9250_task(&calibrationData);
}
