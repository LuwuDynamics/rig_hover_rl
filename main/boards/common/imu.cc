#include "imu.h"
#include "config.h"
#include "board_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <cstdio>
#include <cmath>

#define TAG "QMI8658C"

// 与 QMI8658A 规格书（Rev A）一致的寄存器与器件地址
#define QMI8658C_I2C_ADDRESS_ALT 0x6A
#define QMI8658C_I2C_ADDRESS     0x6B
#define QMI8658C_REG_WHO_AM_I    0x00
#define QMI8658C_REG_REVISION_ID 0x01
#define QMI8658C_REG_CTRL1       0x02
#define QMI8658C_REG_CTRL2       0x03
#define QMI8658C_REG_CTRL3       0x04
#define QMI8658C_REG_CTRL5       0x06
#define QMI8658C_REG_CTRL7       0x08
#define QMI8658C_REG_CTRL8       0x09
#define QMI8658C_REG_CTRL9       0x0A
#define QMI8658C_REG_CAL1_L      0x0B
#define QMI8658C_REG_STATUSINT   0x2D
#define QMI8658C_REG_STATUS0     0x2E
#define QMI8658C_REG_ACC_X_L     0x35
#define QMI8658C_REG_GZ_H        0x40
#define QMI8658C_REG_RESET_FLAG  0x4D
#define QMI8658C_REG_RESET       0x60
#define QMI8658C_WHOAMI_EXPECTED 0x05
#define QMI8658C_RESET_CMD       0xB0
#define QMI8658C_RESET_OK_FLAG   0x80
#define QMI8658C_CTRL9_ACK       0x00
#define QMI8658C_CTRL9_AHB_CG    0x12

#define QMI8658C_CALIBRATION_COUNT 400

#define I2C_FREQ_HZ 400000

// ---------------------- 状态 ----------------------
static i2c_master_bus_handle_t bus_handle = nullptr;
static i2c_master_dev_handle_t dev_handle = nullptr;
static bool imu_initialized = false;
static uint8_t imu_i2c_addr = QMI8658C_I2C_ADDRESS;
static uint8_t imu_revision_id = 0;
static bool imu_sync_sample = false;

// 导出的姿态数据（变量名保持与原先 imu.cc 一致）
float roll = 0.0f;
float pitch = 0.0f;
float pit_gyro = 0.0f;
float _pit_gyro = 0.0f;
float yaw = 0.0f;

float accel_x = 0.0f;
float accel_y = 0.0f;
float accel_z = 0.0f;
float gyro_x = 0.0f;
float gyro_y = 0.0f;
float gyro_z = 0.0f;
float dq = 0.0f;
bool imu_is_initialized() {
    return imu_initialized;
}

// 校准状态（对应 QMI8658C.cpp 中 qmi_countInit / isIMUInit / imu.*_bias）
static float acc_bias[3] = {0};
static float gyro_bias[3] = {0};
static uint16_t qmi_countInit = 0;


// 与 QMI8658C.cpp 文件内变量对齐（本文件内 static，避免与工程其它翻译单元冲突）
static double qmi_d_imu_time = 0;
static int64_t qmi_last_imu_time = 0;
static float pit_g = 0.0f;
static float gyro_yaw = 0.0f;


#if defined(CONFIG_BOARD_TYPE_HOVER)
#include "head_imu_comp.h"
static portMUX_TYPE attitude_mux = portMUX_INITIALIZER_UNLOCKED;
static ImuAttitude attitude{};
static float head_mount_pitch_deg = hover_rl::kDefaultHeadMountPitchDeg;
static float head_mount_roll_deg = hover_rl::kDefaultHeadMountRollDeg;

ImuAttitude imu_get_attitude() {
    portENTER_CRITICAL(&attitude_mux);
    const ImuAttitude sample = attitude;
    portEXIT_CRITICAL(&attitude_mux);
    return sample;
}

void imu_set_head_mount_deg(float pitch_deg, float roll_deg) {
    if (!std::isfinite(pitch_deg) || !std::isfinite(roll_deg)) {
        return;
    }
    const float clamped_p = hover_rl::ClampMountDeg(pitch_deg);
    const float clamped_r = hover_rl::ClampMountDeg(roll_deg);
    portENTER_CRITICAL(&attitude_mux);
    head_mount_pitch_deg = clamped_p;
    head_mount_roll_deg = clamped_r;
    portEXIT_CRITICAL(&attitude_mux);
}

void imu_set_gyro_rp_bias(float bx, float bz) {
    if (!std::isfinite(bx) || !std::isfinite(bz)) {
        return;
    }
    const float c0 = hover_rl::GyroBiasEstimator::ClampBias(bx);
    const float c2 = hover_rl::GyroBiasEstimator::ClampBias(bz);
    portENTER_CRITICAL(&attitude_mux);
    gyro_bias[0] = c0;
    gyro_bias[2] = c2;
    portEXIT_CRITICAL(&attitude_mux);
}

void imu_get_gyro_bias(float bias_dps[3]) {
    portENTER_CRITICAL(&attitude_mux);
    bias_dps[0] = gyro_bias[0];
    bias_dps[1] = gyro_bias[1];
    bias_dps[2] = gyro_bias[2];
    portEXIT_CRITICAL(&attitude_mux);
}

// Hover only: the same filtered gyro channel that is integrated into `yaw` below (yaw = yaw - rate * dt), so the sign
// convention is "positive = yaw increasing". The bias is only valid once the start-up calibration has finished (isIMUInit).
float imu_yaw_rate_dps() {
    portENTER_CRITICAL(&attitude_mux);
    const float by = gyro_bias[1];
    portEXIT_CRITICAL(&attitude_mux);
    return -(gyro_yaw - by);
}
#endif

// ---------------------- 内部函数 ----------------------
static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    if (!dev_handle) return ESP_ERR_INVALID_STATE;
    uint8_t data[2] = {reg, val};
    return i2c_master_transmit(dev_handle, data, 2, pdMS_TO_TICKS(50));
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    if (!dev_handle) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(dev_handle, &reg, 1, buf, len, pdMS_TO_TICKS(50));
}

// Execute a CTRL9 host command using STATUSINT.bit7 handshake (QMI8658A §5.10.1 / §5.10.4).
// Caller must set CTRL8.bit7 = 1 and populate CAL1..CAL4 (if WCtrl9) before calling.
static bool qmi_ctrl9_cmd(uint8_t cmd) {
    if (write_reg(QMI8658C_REG_CTRL9, cmd) != ESP_OK) {
        return false;
    }
    uint8_t status_int = 0;
    bool done = false;
    for (int i = 0; i < 50; ++i) {
        if (read_regs(QMI8658C_REG_STATUSINT, &status_int, 1) == ESP_OK && (status_int & 0x80) != 0) {
            done = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!done) {
        write_reg(QMI8658C_REG_CTRL9, QMI8658C_CTRL9_ACK);
        return false;
    }
    if (write_reg(QMI8658C_REG_CTRL9, QMI8658C_CTRL9_ACK) != ESP_OK) {
        return false;
    }
    for (int i = 0; i < 50; ++i) {
        if (read_regs(QMI8658C_REG_STATUSINT, &status_int, 1) == ESP_OK && (status_int & 0x80) == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

#if defined(CONFIG_BOARD_TYPE_HOVER)
// Hover: the robot is held by hand while it calibrates, so it is never perfectly still and never perfectly upright. The gyro bias only needs the
// angular rate to be small (any orientation will do), so: accept a sample while all three gyro axes are quiet (< kStillDps) and the accelerometer reads
// about 1 g (not shaken), start the average again when the hand moves the robot, and stop waiting for a quiet moment after kGiveUpMs
// (then the bias stays 0 and the robot carries on). The original code averaged whatever it saw, moving or not.
static void calibration_bias_qmi8658c(void) {
    static const float kStillDps = 15.0f;
    static const float kMaxGyroBiasDps = 12.0f;
    static const float kMaxGyroSpanDps = 4.0f;
    static const uint64_t kGiveUpMs = 20000;
    static float g_min[3] = {}, g_max[3] = {};
    const uint64_t ms_since_boot = esp_timer_get_time() / 1000;
    if (ms_since_boot <= 4000 || isIMUInit != 0) {
        return;
    }
    const float g = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z) / 9.8f;
    bool still = fabsf(gyro_x) < kStillDps && fabsf(gyro_y) < kStillDps && fabsf(gyro_z) < kStillDps && g > 0.8f && g < 1.2f;
    if (still && qmi_countInit > 0) {
        const float cur[3] = {gyro_x, gyro_y, gyro_z};
        for (int i = 0; i < 3; i++) {
            const float lo = cur[i] < g_min[i] ? cur[i] : g_min[i];
            const float hi = cur[i] > g_max[i] ? cur[i] : g_max[i];
            if (hi - lo > kMaxGyroSpanDps) {
                still = false;
                break;
            }
        }
    }
    if (still) {
        const float cur[3] = {gyro_x, gyro_y, gyro_z};
        if (qmi_countInit == 0) {
            for (int i = 0; i < 3; i++) {
                acc_bias[i] = 0;
                gyro_bias[i] = 0;
                g_min[i] = g_max[i] = cur[i];
            }
        } else {
            for (int i = 0; i < 3; i++) {
                if (cur[i] < g_min[i]) g_min[i] = cur[i];
                if (cur[i] > g_max[i]) g_max[i] = cur[i];
            }
        }
        acc_bias[0] += accel_x;
        acc_bias[1] += accel_y;
        acc_bias[2] += accel_z;
        gyro_bias[0] += gyro_x;
        gyro_bias[1] += gyro_y;
        gyro_bias[2] += gyro_z;
        qmi_countInit++;
    } else {
        qmi_countInit = 0;      // the robot was moved: start again
    }
    const bool timeout = ms_since_boot > kGiveUpMs;
    if (qmi_countInit >= QMI8658C_CALIBRATION_COUNT || (timeout && qmi_countInit >= 50)) {
        float final_bias[3] = {};
        for (int i = 0; i < 3; i++) {
            acc_bias[i] /= (float)qmi_countInit;
            final_bias[i] = gyro_bias[i] / (float)qmi_countInit;
            if (fabsf(final_bias[i]) > kMaxGyroBiasDps) {
                ESP_LOGW(TAG, "gyro_bias[%d]=%.2f dps > %.1f, treat as motion, set to 0", i, final_bias[i], kMaxGyroBiasDps);
                final_bias[i] = 0.0f;
            }
        }
        portENTER_CRITICAL(&attitude_mux);
        for (int i = 0; i < 3; i++) gyro_bias[i] = final_bias[i];
        portEXIT_CRITICAL(&attitude_mux);
        ESP_LOGW(TAG, "IMU calibrated from %u quiet samples, gyro bias %.2f %.2f %.2f dps", (unsigned)qmi_countInit, final_bias[0], final_bias[1], final_bias[2]);
        isIMUInit = 1;
    } else if (timeout) {
        portENTER_CRITICAL(&attitude_mux);
        for (int i = 0; i < 3; i++) {
            acc_bias[i] = 0;
            gyro_bias[i] = 0;
        }
        portEXIT_CRITICAL(&attitude_mux);
        ESP_LOGW(TAG, "IMU calibration: no quiet moment within %u s, gyro bias left at 0", (unsigned)(kGiveUpMs / 1000));
        isIMUInit = 1;
    }
}
#else
static void calibration_bias_qmi8658c(void) {
    if (qmi_countInit == 0) {
        for (int i = 0; i < 3; i++) {
            acc_bias[i] = 0;
            gyro_bias[i] = 0;
        }
    }
    const uint64_t ms_since_boot = esp_timer_get_time() / 1000;
    if (ms_since_boot > 4000 && isIMUInit == 0) {
        if (qmi_countInit <= QMI8658C_CALIBRATION_COUNT) {
            acc_bias[0] += accel_x;
            acc_bias[1] += accel_y;
            acc_bias[2] += accel_z;
            gyro_bias[0] += gyro_x;
            gyro_bias[1] += gyro_y;
            gyro_bias[2] += gyro_z;
            qmi_countInit++;
        } else {
            static const float kMaxGyroBiasDps = 5.0f;
            for (int i = 0; i < 3; i++) {
                acc_bias[i] /= (float)QMI8658C_CALIBRATION_COUNT;
                gyro_bias[i] /= (float)QMI8658C_CALIBRATION_COUNT;
                if (fabsf(gyro_bias[i]) > kMaxGyroBiasDps) {
                    ESP_LOGW(TAG, "gyro_bias[%d]=%.2f dps > %.1f, treat as motion, set to 0",
                             i, gyro_bias[i], kMaxGyroBiasDps);
                    gyro_bias[i] = 0.0f;
                }
            }
            isIMUInit = 1;
        }
    }
}
#endif

// ---------------------- 初始化 ----------------------
void imu_init() {
    if (imu_initialized) {
        ESP_LOGW(TAG, "IMU already initialized");
        return;
    }

    ESP_LOGI(TAG, "Initializing QMI8658C on SDA=%d, SCL=%d", IMU_I2C_SDA, IMU_I2C_SCL);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = IMU_I2C_SDA,
        .scl_io_num = IMU_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    imu_i2c_addr = QMI8658C_I2C_ADDRESS;
    if (i2c_master_probe(bus_handle, imu_i2c_addr, pdMS_TO_TICKS(200)) != ESP_OK) {
        imu_i2c_addr = QMI8658C_I2C_ADDRESS_ALT;
        if (i2c_master_probe(bus_handle, imu_i2c_addr, pdMS_TO_TICKS(200)) != ESP_OK) {
            ESP_LOGW(TAG, "QMI8658C not found at 0x%02X / 0x%02X", QMI8658C_I2C_ADDRESS,
                     QMI8658C_I2C_ADDRESS_ALT);
            i2c_del_master_bus(bus_handle);
            bus_handle = nullptr;
            return;
        }
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = imu_i2c_addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        bus_handle = nullptr;
        return;
    }

    auto cleanup_i2c = [&]() {
        if (dev_handle) {
            i2c_master_bus_rm_device(dev_handle);
            dev_handle = nullptr;
        }
        if (bus_handle) {
            i2c_del_master_bus(bus_handle);
            bus_handle = nullptr;
        }
    };

    // Wait >= 15 ms System Turn-On Time (Table 7/8 p.17-18) before first access.
    vTaskDelay(pdMS_TO_TICKS(15));
    uint8_t id = 0;
    ret = read_regs(QMI8658C_REG_WHO_AM_I, &id, 1);
    if (ret != ESP_OK || id != QMI8658C_WHOAMI_EXPECTED) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X (expected 0x%02X)", id, QMI8658C_WHOAMI_EXPECTED);
        cleanup_i2c();
        return;
    }

    // Soft reset so ESP32 warm reboot never inherits stale IMU register state (Table 27 p.40 / §7.4 p.51).
    if (write_reg(QMI8658C_REG_RESET, QMI8658C_RESET_CMD) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));
        uint8_t reset_flag = 0;
        if (read_regs(QMI8658C_REG_RESET_FLAG, &reset_flag, 1) == ESP_OK &&
            reset_flag != QMI8658C_RESET_OK_FLAG) {
            ESP_LOGW(TAG, "Soft reset flag 0x4D=0x%02X (expected 0x80)", reset_flag);
        }
    } else {
        ESP_LOGW(TAG, "Soft reset write failed, continuing init");
    }

    imu_revision_id = 0;
    read_regs(QMI8658C_REG_REVISION_ID, &imu_revision_id, 1);

    // Configure CTRL1/CTRL8/CTRL2/CTRL3/CTRL5 BEFORE enabling sensors in CTRL7 (§13.5 p.71 / §7.2 p.49).
    bool cfg_ok = true;
    cfg_ok &= (write_reg(QMI8658C_REG_CTRL1, 0x40) == ESP_OK);  // ADDR_AI=1, BE=0 (LE), SensorDisable=0
    cfg_ok &= (write_reg(QMI8658C_REG_CTRL8, 0x80) == ESP_OK);  // CTRL9 handshake via STATUSINT.bit7
    cfg_ok &= (write_reg(QMI8658C_REG_CTRL2, 0x04) == ESP_OK);  // Accel +-2g, 448.4 Hz (6DOF)
    cfg_ok &= (write_reg(QMI8658C_REG_CTRL3, 0x64) == ESP_OK);  // Gyro +-1024 dps, 448.4 Hz (6DOF)
    cfg_ok &= (write_reg(QMI8658C_REG_CTRL5, 0x00) == ESP_OK);  // LPF off (matches RL training)
    if (!cfg_ok) {
        ESP_LOGE(TAG, "Failed to write QMI8658 config registers");
        cleanup_i2c();
        return;
    }

    // Disable internal AHB clock gating before enabling SyncSample (required for I2C per §13.2.1 p.70).
    imu_sync_sample = false;
    if (write_reg(QMI8658C_REG_CAL1_L, 0x01) == ESP_OK &&
        qmi_ctrl9_cmd(QMI8658C_CTRL9_AHB_CG)) {
        imu_sync_sample = true;
    } else {
        ESP_LOGW(TAG, "CTRL9 AHB clock gating disable failed (rev=0x%02X); using Non-SyncSample + STATUS0",
                 imu_revision_id);
    }

    const uint8_t ctrl7_val = imu_sync_sample ? 0x83 : 0x03;
    if (write_reg(QMI8658C_REG_CTRL7, ctrl7_val) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable QMI8658 sensors (CTRL7=0x%02X)", ctrl7_val);
        cleanup_i2c();
        return;
    }

    // Gyro Turn-On Time = 150 ms + 3/ODR ≈ 157 ms (Table 8 p.18 / §7.3 p.51).
    vTaskDelay(pdMS_TO_TICKS(160));

    // Verify SyncSample Avail asserts after warm-up, then consume and unlock the first frame.
    if (imu_sync_sample) {
        uint8_t status_int = 0;
        if (read_regs(QMI8658C_REG_STATUSINT, &status_int, 1) == ESP_OK && (status_int & 0x01) != 0) {
            if ((status_int & 0x02) == 0) {
                esp_rom_delay_us(15);  // Table 40 p.70: Data_Lock_Delay = 12 us @ 448.4 Hz
            }
            uint8_t discard[12] = {0};
            read_regs(QMI8658C_REG_ACC_X_L, discard, 12);
        } else {
            ESP_LOGW(TAG, "SyncSample Avail=0 after 160 ms (STATUSINT=0x%02X); fallback to Non-SyncSample",
                     status_int);
            imu_sync_sample = false;
            write_reg(QMI8658C_REG_CTRL7, 0x03);
        }
    }

#if defined(CONFIG_BOARD_TYPE_HOVER)
    qmi_last_imu_time = 0;  // The first calibrated sample seeds the filter.
#else
    qmi_last_imu_time = esp_timer_get_time();
#endif
    imu_initialized = true;
    ESP_LOGI(TAG, "QMI8658 initialized at 0x%02X rev=0x%02X sync=%d",
             imu_i2c_addr, imu_revision_id, imu_sync_sample ? 1 : 0);
}

// ---------------------- 读取数据 ----------------------
float d_time = 0.0f;
int64_t last_time = 0;

void imu_read_once() {
    if (!imu_initialized || !dev_handle) {
        return;
    }

    if (imu_sync_sample) {
        // QMI8658A §13.2.3 Locking Mechanism Data Reading Process (p.70).
        uint8_t status_int = 0;
        if (read_regs(QMI8658C_REG_STATUSINT, &status_int, 1) != ESP_OK) {
            return;
        }
        if ((status_int & 0x01) == 0) {
            // Defensive unlock if Locked was ever left high without Avail.
            if (status_int & 0x02) {
                uint8_t dummy = 0;
                read_regs(QMI8658C_REG_GZ_H, &dummy, 1);
            }
            return;
        }
        if ((status_int & 0x02) == 0) {
            esp_rom_delay_us(15);  // Table 40 p.70: 12 us Data_Lock_Delay @ 448.4 Hz
        }
    } else {
        // Fallback Non-SyncSample: check STATUS0 (aDA & gDA) to avoid duplicate reads.
        uint8_t status0 = 0;
        if (read_regs(QMI8658C_REG_STATUS0, &status0, 1) != ESP_OK) {
            return;
        }
        if ((status0 & 0x03) != 0x03) {
            return;
        }
    }

    uint8_t data[12] = {0};
    if (read_regs(QMI8658C_REG_ACC_X_L, data, 12) != ESP_OK) {
        return;
    }

    int16_t accx = (int16_t)(data[1] << 8 | data[0]);
    int16_t accy = (int16_t)(data[3] << 8 | data[2]);
    int16_t accz = (int16_t)(data[5] << 8 | data[4]);
    int16_t gyrox = (int16_t)(data[7] << 8 | data[6]);
    int16_t gyroy = (int16_t)(data[9] << 8 | data[8]);
    int16_t gyroz = (int16_t)(data[11] << 8 | data[10]);

    /* 与 QMI8658C.cpp 相同的量程与轴映射，写入原 imu.cc 变量名 */
    accel_x = -(float)accx / 16384.0f * 9.8f;
    accel_y = -(float)accy / 16384.0f * 9.8f;
    accel_z = (float)accz / 16384.0f * 9.8f;

    gyro_y = (float)gyrox / 32768.0f * 1024.0f;
    gyro_x = (float)gyroy / 32768.0f * 1024.0f;
    gyro_z = (float)gyroz / 32768.0f * 1024.0f;

    calibration_bias_qmi8658c();

    // TARS publishes live attitude from the first valid sample.  Its
    // background calibration flag still turns to OK once the sample window
    // completes, but the status screen no longer sits at 0.0 meanwhile.
#if !defined(CONFIG_BOARD_TYPE_TARS)
    if (!isIMUInit) {
        return;
    }
#endif

    // gyro_x -= gyro_bias[0];
    // gyro_y -= gyro_bias[1];
    // gyro_z -= gyro_bias[2];

    const int64_t current_time = esp_timer_get_time();
    const int64_t elapsed_us = current_time - qmi_last_imu_time;
#if defined(CONFIG_BOARD_TYPE_HOVER)
    const bool reseed = qmi_last_imu_time == 0 || elapsed_us > kImuMaxGapUs;
#else
    constexpr bool reseed = false;
#endif
    qmi_d_imu_time = reseed ? 0.0 : elapsed_us * 0.000001;
    d_time = (float)qmi_d_imu_time;
    last_time = current_time;

#if defined(CONFIG_BOARD_TYPE_HOVER)
    float mount_pitch_deg = hover_rl::kDefaultHeadMountPitchDeg;
    float mount_roll_deg = hover_rl::kDefaultHeadMountRollDeg;
    float gb0 = 0.0f, gb1 = 0.0f, gb2 = 0.0f;
    portENTER_CRITICAL(&attitude_mux);
    mount_pitch_deg = head_mount_pitch_deg;
    mount_roll_deg = head_mount_roll_deg;
    gb0 = gyro_bias[0];
    gb1 = gyro_bias[1];
    gb2 = gyro_bias[2];
    portEXIT_CRITICAL(&attitude_mux);

    const float gx_comp = gyro_x - gb0;
    const float gy_comp = gyro_y - gb1;
    const float gz_comp = gyro_z - gb2;
    const hover_rl::HeadImuProjection proj = hover_rl::ProjectHeadImu(
        accel_x, accel_y, accel_z, gx_comp, gy_comp, gz_comp,
        q_head, mount_pitch_deg, mount_roll_deg);
    pit_gyro = proj.pitch_gyro_dps;
    const float pit_acc = proj.pitch_acc_deg;
    const float acc_roll = proj.roll_acc_deg;
#else
    float pit_acc = 0.0f;
    pit_gyro = -(cos((q_head + 90.0) / 180.0 * PI) * gyro_x +
                 sin((q_head + 90.0) / 180.0 * PI) * gyro_z);
    pit_acc = atan((sin((q_head + 90.0) / 180.0 * PI) * accel_y +
                    cos((q_head + 90.0) / 180.0 * PI) * accel_z) /
                   accel_x) *
              57.2958f;
    const float acc_roll = atan((sin((q_head + 90.0) / 180.0 * PI) * accel_z +
                 cos((q_head + 90.0) / 180.0 * PI) * accel_y) /
                accel_x) *
           57.2958f;
#endif

#if defined(CONFIG_BOARD_TYPE_HOVER)
    if (!std::isfinite(pit_acc) || !std::isfinite(acc_roll) || !std::isfinite(pit_gyro)) {
        return;
    }
#endif
    roll = acc_roll;
    if (reseed) {
        pitch = pit_acc;
        dq = pit_gyro;
        gyro_yaw = gyro_y;
    } else {
        dq = 0.6f * dq + 0.4f * pit_gyro;
        gyro_yaw = 0.5f * gyro_yaw + 0.5f * gyro_y;
#if defined(CONFIG_BOARD_TYPE_HOVER)
        // Off-axis head IMU picks up centripetal/tangential accel during fast yaw rotation
        // (head turns or body spins). Keep pit_freq untouched below 30 dps so normal standing
        // never loses accel anchoring, and attenuate above 30 dps to reject spin spikes.
        const float gy_excess = fabsf(gy_comp) > 30.0f ? (fabsf(gy_comp) - 30.0f) : 0.0f;
        const float pit_freq = 1.0f / (fabsf(pit_gyro) + 25.0f * gy_excess + 200.0f);
#else
        const float pit_freq = 1.0f / (fabsf(pit_gyro) + 200.0f);
#endif
        if (fabsf(pit_acc) < 100.0f && fabsf(pit_gyro) < 1000.0f) {
            pitch = pit_freq * pit_acc + (1.0f - pit_freq) * (pitch + pit_gyro * (float)qmi_d_imu_time);
        }
    }
    pit_g = pit_g + pit_gyro * (float)qmi_d_imu_time;
    qmi_last_imu_time = current_time;
    // yaw 无磁力计绝对校正，纯陀螺积分。gyro_y 通道的静止零偏由开机校准
    // 窗口算出（gyro_bias[1]），必须减去——否则零偏原样进入积分，屏幕上
    // Y 值会持续爬升。补偿后再叠加静止死区（<1.5°/s 视为零漂丢弃）。
    // roll/pitch 有加速度计互补滤波抗漂，不受影响。
    constexpr float kYawGyroDeadZoneDps = 1.5f;
    float gyro_yaw_comp = gyro_yaw - gyro_bias[1];
    if (fabsf(gyro_yaw_comp) < kYawGyroDeadZoneDps) {
        gyro_yaw_comp = 0.0f;
    }
    if (isIMUInit) {  // bias 校准完成后才开始积分，校准窗口内 yaw 保持 0
        yaw = yaw - gyro_yaw_comp * (float)qmi_d_imu_time;
    }
#if defined(CONFIG_BOARD_TYPE_HOVER)
    const float yaw_rate_pub = -(gyro_yaw - gb1);
    portENTER_CRITICAL(&attitude_mux);
    attitude = {current_time, reseed ? current_time : attitude.continuous_since_us,
                pitch, roll, yaw, dq, yaw_rate_pub,
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z};
    portEXIT_CRITICAL(&attitude_mux);
#endif
}

// ---------------------- 清理 ----------------------

void imu_deinit() {
    if (dev_handle) {
        write_reg(QMI8658C_REG_CTRL7, 0x00);
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = nullptr;
    }
    if (bus_handle) {
        i2c_del_master_bus(bus_handle);
        bus_handle = nullptr;
    }
    imu_initialized = false;
    imu_sync_sample = false;
    imu_revision_id = 0;
#if defined(CONFIG_BOARD_TYPE_HOVER)
    portENTER_CRITICAL(&attitude_mux);
    attitude = {};
    head_mount_pitch_deg = hover_rl::kDefaultHeadMountPitchDeg;
    head_mount_roll_deg = hover_rl::kDefaultHeadMountRollDeg;
    portEXIT_CRITICAL(&attitude_mux);
#endif
    isIMUInit = 0;
    qmi_countInit = 0;
    ESP_LOGI(TAG, "IMU deinitialized");
}
