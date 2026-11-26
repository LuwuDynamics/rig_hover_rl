#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "esp32_camera.h"
// #include "camera_web_server.h"  // TODO: Migrate camera web server if needed
// #include "lulu_ble.h"  // 鏆傛椂灞忚斀锛屽惎鐢?BluFi 閰嶇綉
#include "assets/lang_config.h"
#include "board_config.h"
#include "settings.h"
#include "sdkconfig.h"
#include "ssid_manager.h"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <cmath>
#include <cstdlib>
#include <esp_heap_caps.h>

#include "display/emote_display.h"

#include <wifi_manager.h>
#include <esp_log.h>
#include <cJSON.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <nvs_flash.h>

#include "esp_lcd_gc9a01.h"
#include "robot.h"
#include "robot_action.h"
#include "gesture_player.h"
#include <ctype.h>
#include "imu.h"

#define TAG "HOVER"

// Hover 专属音效
namespace {
    // 寮€鏈洪煶鏁?(0~3s)
    extern const char engine_startup_ogg_start[] asm("_binary_engine_startup_ogg_start");
    extern const char engine_startup_ogg_end[] asm("_binary_engine_startup_ogg_end");
    static const std::string_view STARTUP_SOUND {
        static_cast<const char*>(engine_startup_ogg_start),
        static_cast<size_t>(engine_startup_ogg_end - engine_startup_ogg_start)
    };
    // 油门音效 (8s~结尾)
    extern const char engine_throttle_ogg_start[] asm("_binary_engine_throttle_ogg_start");
    extern const char engine_throttle_ogg_end[] asm("_binary_engine_throttle_ogg_end");
    static const std::string_view THROTTLE_SOUND {
        static_cast<const char*>(engine_throttle_ogg_start),
        static_cast<size_t>(engine_throttle_ogg_end - engine_throttle_ogg_start)
    };

    // 站稳后播放一声约 0.36 s 的木琴提示音，国内和海外版共用。由 sim/tools/make_stand_cue.py 合成。
    extern const char stand_ok_ogg_start[] asm("_binary_stand_ok_ogg_start");
    extern const char stand_ok_ogg_end[] asm("_binary_stand_ok_ogg_end");
    static const std::string_view STAND_OK_SOUND {
        static_cast<const char*>(stand_ok_ogg_start),
        static_cast<size_t>(stand_ok_ogg_end - stand_ok_ogg_start)
    };

#if CONFIG_FIRMWARE_REGION_OVERSEAS
    extern const char low_battery_ogg_start[] asm("_binary_hover_low_battery_en_ogg_start");
    extern const char low_battery_ogg_end[] asm("_binary_hover_low_battery_en_ogg_end");
#else
    extern const char low_battery_ogg_start[] asm("_binary_hover_low_battery_zh_ogg_start");
    extern const char low_battery_ogg_end[] asm("_binary_hover_low_battery_zh_ogg_end");
#endif
    const std::string_view LOW_BATTERY_SOUND {
        low_battery_ogg_start,
        static_cast<size_t>(low_battery_ogg_end - low_battery_ogg_start)
    };

    class LowBatteryReminder {
    public:
        bool Update(const BatteryVoltageSample& sample, int64_t now_us) {
            if (sample.received_us == 0 || sample.volts < 0.1f ||
                now_us - sample.received_us > kMaxSampleGapUs) {
                consecutive_ = 0;
                return false;
            }

            if (sample.received_us != last_sample_us_) {
                if (sample.received_us - last_sample_us_ > kMaxSampleGapUs) {
                    consecutive_ = 0;
                }
                last_sample_us_ = sample.received_us;
                const bool qualifies = played_ ? sample.volts >= kRearmVolts : sample.volts <= kLowVolts;
                consecutive_ = qualifies ? std::min(consecutive_ + 1, kConfirmSamples) : 0;
                if (played_ && consecutive_ == kConfirmSamples) {
                    played_ = false;
                    consecutive_ = 0;
                }
            }
            return !played_ && consecutive_ == kConfirmSamples;
        }

        void MarkPlayed() {
            played_ = true;
            consecutive_ = 0;
        }

    private:
        static constexpr float kLowVolts = 7.1f;
        static constexpr float kRearmVolts = 7.3f;
        static constexpr int kConfirmSamples = 3;
        static constexpr int64_t kMaxSampleGapUs = kBatteryVoltagePollIntervalUs * 3 / 2;

        int64_t last_sample_us_ = 0;
        int consecutive_ = 0;
        bool played_ = false;
    };

    // 提示音按固件区域固定语言：中文固件只用中文提示，英文固件只用英文提示
    std::string_view WifiConnectedPrompt() {
#if CONFIG_FIRMWARE_REGION_OVERSEAS
        return Lang::Sounds::OGG_WIFI_CONNECTED_EN_US();
#else
        return Lang::Sounds::OGG_WIFI_CONNECTED_ZH_CN();
#endif
    }

    // 绗竴娆℃垚鍔熻仈缃戠殑瀹屾暣鎻愮ず锛歐iFi 宸茶繛鎺?+ 淇濇寔鍨傜洿绔欏Э 15 绉?
    std::string_view FirstWifiConnectedPrompt() {
#if CONFIG_FIRMWARE_REGION_OVERSEAS
        return Lang::Sounds::OGG_WIFI_FIRST_CONNECTED_EN_US();
#else
        return Lang::Sounds::OGG_WIFI_FIRST_CONNECTED_ZH_CN();
#endif
    }

    // 长按第 2 秒的提示：即将断开网络连接，进入标定模式
    std::string_view CalibrationWarningPrompt() {
#if CONFIG_FIRMWARE_REGION_OVERSEAS
        return Lang::Sounds::OGG_CALIBRATION_WARNING_EN_US();
#else
        return Lang::Sounds::OGG_CALIBRATION_WARNING_ZH_CN();
#endif
    }

    // 璁板綍鈥滄槸鍚﹀凡缁忔垚鍔熻仈缃戣繃鈥濓紝鎺夌數/閲嶅惎鍚庝粛鐒舵湁鏁堬紱闀挎寜閲嶇疆 NVS 鍚庨噸鏂扮畻绗竴娆°€?
    // 娉ㄦ剰锛歂VS key 鏈€闀?15 涓瓧绗︺€?
    constexpr char kHoverNvsNamespace[] = "hover";
    constexpr char kFirstWifiConnectedKey[] = "first_conn_done";

    // 瑙︽懜闀挎寜杩涘叆鏍囧畾锛?0 绉掞紝鍙傝€?ARM 鐨勫渾鐜€掕鏃舵牱寮?
    constexpr int kTouchCalibrationMs = 10000;
    constexpr int kCalibrationSteps = 10;
    constexpr int kCalibrationFrameSize = 240;
    constexpr int kCalibrationCenter = kCalibrationFrameSize / 2;
    constexpr int kCalibrationOuterRadius = 118;
    constexpr int kCalibrationInnerRadius = 106;
    constexpr int kCalibrationCenterDotRadius = 16;

    // Settings 灏佽鍦?EraseAll/EraseKey 鍚庝笉浼氳嚜鍔?commit锛岃繖閲岀洿鎺ユ搷浣?NVS
    void EraseSettingsNamespace(const char* ns) {
        nvs_handle_t handle = 0;
        if (nvs_open(ns, NVS_READWRITE, &handle) == ESP_OK) {
            nvs_erase_all(handle);
            nvs_commit(handle);
            nvs_close(handle);
        }
    }
}

// 闀挎寜閲嶇疆 NVS 鐨勬椂闂撮槇鍊硷紙姣锛?
static constexpr int kLongPressResetMs = 3000;
static constexpr int kLongPressShowEmotionMs = 1000;  // 长按1秒后显示表情
static constexpr int kDoubleTapWindowMs = 400;  // 鍙屽嚮妫€娴嬬獥鍙ｏ紙姣锛?

class HoverBoard : public WifiBoard {
private:
    Button boot_button_;
    Button touch_button_;
    emote::EmoteDisplay* display_ = nullptr;  // AAF动画显示
    Esp32Camera* camera_ = nullptr;  // 初始化为nullptr
    TaskHandle_t robot_task_handle_ = nullptr;
    TaskHandle_t motor_rx_task_handle_ = nullptr;
    TaskHandle_t imu_task_handle_ = nullptr;
    int64_t button_press_start_time_ = 0;  // 鎸夐敭鎸変笅鏃堕棿鎴?
    esp_timer_handle_t long_press_timer_ = nullptr;  // 长按检测定时器
    bool nvs_reset_emotion_shown_ = false;  // 鏄惁宸叉樉绀?nvs_reset 琛ㄦ儏
    bool agent_configured_ = true;          // 服务端是否已配置智能体（激活回调更新）
    std::atomic<bool> just_provisioned_{false};
    bool activation_required_ = false;      // 鏈嶅姟绔槸鍚﹁姹傚厛婵€娲昏澶?

    // 瑙︽懜闀挎寜 / 鏍囧畾妯″紡鐘舵€?
    int64_t touch_press_start_time_ = 0;
    DeviceState touch_press_chat_state_ = kDeviceStateUnknown;
    esp_timer_handle_t touch_progress_timer_ = nullptr;
    int touch_progress_segments_ = -1;
    std::atomic<bool> touch_long_press_active_{false};
    bool calibration_warning_played_ = false;
    std::atomic<bool> startup_calibration_pending_{false};
    std::atomic<bool> calibration_mode_{false};
    std::atomic<bool> servo_calibration_ready_{false};  // 3 鍙疯埖鏈哄凡鍒?1500 骞跺嵏鍔?
    bool suppress_touch_release_ = false;
    uint16_t* calibration_frame_ = nullptr;  // 240x240 RGB565 倒计时环
    uint16_t* calibration_angle_ = nullptr;  // 鍦嗙幆鍍忕礌鐨勮搴︼紙0.1 搴︼級锛?xFFFF 琛ㄧず鐜
    std::mutex calibration_ui_mutex_;

    // 鏈嶅姟绔姹傛縺娲绘椂涓嶆竻绌洪厤缃戙€佷笉閲嶅惎锛氫繚鐣?WiFi锛岄娆¤仈缃戞彁绀烘墠鑳芥甯告挱鏀?
    virtual bool ShouldResetOnActivationRequired() override {
        activation_required_ = true;
        return false;
    }

    void ShowWifiConnectedAnimation() {
        Application::GetInstance().Schedule([this]() {
            const char* emotion = display_->GetEmotion();
            if (strcmp(emotion, "wificonfig") == 0 || strcmp(emotion, "scanning") == 0) {
                const auto state = Application::GetInstance().GetDeviceState();
                if (state == kDeviceStateStarting || state == kDeviceStateWifiConfiguring ||
                    state == kDeviceStateActivating) {
                    display_->SetEmotion("happy");
                } else if (state == kDeviceStateIdle) {
                    display_->SetEmotion("neutral");
                }
            }
        });
    }

    // 首次成功联网：播报完整提示并保持竖直站姿 15 秒。
    void NotifyWifiConnected() {
        // 标定倒计时/标定中不要插播 WiFi 提示，避免抢占第 2 秒的标定警告
        if (calibration_mode_ || touch_long_press_active_) {
            ESP_LOGW(TAG, "WiFi prompt skipped during calibration countdown");
            return;
        }
        bool first_connection = false;
        {
            Settings settings(kHoverNvsNamespace, false);
            first_connection = !settings.GetBool(kFirstWifiConnectedKey, false);
        }

        auto& app = Application::GetInstance();
        if (first_connection) {
            Settings settings(kHoverNvsNamespace, true);
            settings.SetBool(kFirstWifiConnectedKey, true);

            // 鍏堝惎鍔ㄧ珯濮夸繚鎸侊紝閬垮厤鎾斁鎻愮ず闊抽樆濉炲鑷寸珯濮胯鏃跺欢鍚?
            HoldStandStance(15000);
            app.PlaySound(FirstWifiConnectedPrompt());
            ESP_LOGW(TAG, "First WiFi connection: full prompt + 15s stand hold");
        } else {
            app.PlaySound(WifiConnectedPrompt());
            ESP_LOGW(TAG, "WiFi connected prompt played (not first connection)");
        }
    }

    // 瑙︽懜鍙屽嚮妫€娴?
    std::atomic<bool> touch_listen_requested_{false};
    TaskHandle_t prompt_task_ = nullptr;
    int64_t launch_overlay_until_us_ = 0;
    bool touch_gesture_playing_ = false;  // Owned by RunPromptTask.
    int touch_tap_count_ = 0;                    // 触摸点击计数
    esp_timer_handle_t tap_timer_ = nullptr;    // 双击检测定时器
    DeviceState tap_chat_state_ = kDeviceStateUnknown;

    // ==================== 3 鍙疯埖鏈烘爣瀹?/ 10 绉掕繘搴︾幆 ====================

    // ARM 风格倒计时：屏幕外圈一条连续的圆环，按 10 秒进度顺时针填充
    static constexpr uint8_t kDigitFont[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
        {0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
        {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
        {0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
        {0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
        {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
        {0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
        {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
        {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
        {0x06, 0x49, 0x49, 0x29, 0x1E},  // 9
    };

    void DrawCalibrationGlyph(int digit, int x, int y, int scale, uint16_t color) {
        for (int col = 0; col < 5; ++col) {
            const uint8_t bits = kDigitFont[digit][col];
            for (int row = 0; row < 7; ++row) {
                if ((bits & (1U << row)) == 0) {
                    continue;
                }
                for (int sy = 0; sy < scale; ++sy) {
                    for (int sx = 0; sx < scale; ++sx) {
                        const int px = x + col * scale + sx;
                        const int py = y + row * scale + sy;
                        if (px >= 0 && px < kCalibrationFrameSize &&
                            py >= 0 && py < kCalibrationFrameSize) {
                            calibration_frame_[py * kCalibrationFrameSize + px] = color;
                        }
                    }
                }
            }
        }
    }

    void DrawCalibrationNumber(int value, uint16_t color) {
        constexpr int scale = 8;
        int digits[2];
        int count = 1;
        if (value >= 10) {
            digits[0] = 1;
            digits[1] = 0;
            count = 2;
        } else {
            digits[0] = value < 0 ? 0 : value;
        }
        constexpr int digit_width = 5 * scale;
        constexpr int gap = scale;
        const int total_width = count * digit_width + (count - 1) * gap;
        const int x0 = kCalibrationCenter - total_width / 2;
        const int y0 = kCalibrationCenter - (7 * scale) / 2;
        for (int i = 0; i < count; ++i) {
            DrawCalibrationGlyph(digits[i], x0 + i * (digit_width + gap), y0, scale, color);
        }
    }

    void BuildCalibrationAngleMap() {
        const int inner2 = kCalibrationInnerRadius * kCalibrationInnerRadius;
        const int outer2 = kCalibrationOuterRadius * kCalibrationOuterRadius;
        for (int y = 0; y < kCalibrationFrameSize; ++y) {
            for (int x = 0; x < kCalibrationFrameSize; ++x) {
                const int dx = x - kCalibrationCenter;
                const int dy = y - kCalibrationCenter;
                const int r2 = dx * dx + dy * dy;
                uint16_t angle = 0xFFFF;
                if (r2 >= inner2 && r2 <= outer2) {
                    float deg = atan2f((float)dy, (float)dx) * 180.0f / PI + 90.0f;
                    if (deg < 0.0f) {
                        deg += 360.0f;
                    }
                    if (deg >= 360.0f) {
                        deg -= 360.0f;
                    }
                    angle = (uint16_t)(deg * 10.0f + 0.5f);
                    if (angle >= 3600) {
                        angle -= 3600;
                    }
                }
                calibration_angle_[y * kCalibrationFrameSize + x] = angle;
            }
        }
    }

    bool EnsureCalibrationFrame() {
        if (calibration_frame_ != nullptr && calibration_angle_ != nullptr) {
            return true;
        }
        calibration_frame_ = (uint16_t*)heap_caps_malloc(
            kCalibrationFrameSize * kCalibrationFrameSize * sizeof(uint16_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        calibration_angle_ = (uint16_t*)heap_caps_malloc(
            kCalibrationFrameSize * kCalibrationFrameSize * sizeof(uint16_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (calibration_frame_ == nullptr || calibration_angle_ == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate calibration frame");
            if (calibration_frame_) {
                heap_caps_free(calibration_frame_);
                calibration_frame_ = nullptr;
            }
            if (calibration_angle_) {
                heap_caps_free(calibration_angle_);
                calibration_angle_ = nullptr;
            }
            return false;
        }
        BuildCalibrationAngleMap();
        return true;
    }

    void RenderCalibrationFrame(int filled, bool active) {
        // 涓?ARM 鐨?nvs_reset 鍊掕鏃?EAF 鍙栬壊涓€鑷达細娣辫摑搴?+ 绱壊鍦嗙幆
        constexpr uint16_t kBackground = 0x0885;  // #0C102C
        constexpr uint16_t kRingTrack = 0x2929;   // #2C274F
        constexpr uint16_t kRingFill = 0x787F;    // #780FFB
        constexpr uint16_t kWhite = 0xFFFF;
        const int progress = active ? 3600 : filled * 360;
        const int dot2 = kCalibrationCenterDotRadius * kCalibrationCenterDotRadius;

        for (int y = 0; y < kCalibrationFrameSize; ++y) {
            for (int x = 0; x < kCalibrationFrameSize; ++x) {
                const int index = y * kCalibrationFrameSize + x;
                const uint16_t angle = calibration_angle_[index];
                uint16_t color = kBackground;
                if (angle != 0xFFFF) {
                    color = (angle <= progress) ? kRingFill : kRingTrack;
                }
                calibration_frame_[index] = color;
            }
        }

        if (active) {
            // 圆环填满后中心显示白点，表示可以再次触摸完成标定
            for (int y = kCalibrationCenter - kCalibrationCenterDotRadius;
                 y <= kCalibrationCenter + kCalibrationCenterDotRadius; ++y) {
                for (int x = kCalibrationCenter - kCalibrationCenterDotRadius;
                     x <= kCalibrationCenter + kCalibrationCenterDotRadius; ++x) {
                    const int dx = x - kCalibrationCenter;
                    const int dy = y - kCalibrationCenter;
                    if (dx * dx + dy * dy <= dot2) {
                        calibration_frame_[y * kCalibrationFrameSize + x] = kWhite;
                    }
                }
            }
        } else {
            DrawCalibrationNumber(10 - filled, kWhite);
        }
    }

    void ShowCalibrationProgress(int filled, bool active) {
        if (display_ == nullptr) {
            return;
        }
        int step = active ? kCalibrationSteps : filled;
        if (step < 0) {
            step = 0;
        } else if (step > kCalibrationSteps) {
            step = kCalibrationSteps;
        }
        // 使用独立图层的分段圆环，和 ARM 的倒计时样式一致
        display_->ShowProgressRing(step, kCalibrationSteps);
    }

    void HideCalibrationProgress() {
        if (display_ != nullptr) {
            display_->ShowProgressRing(0, 0);
        }
    }

    void ClearWifiAndAgentConfig() {
        SsidManager::GetInstance().Clear();
        EraseSettingsNamespace("mqtt");
        EraseSettingsNamespace("websocket");

        // 娓呮帀棣栨鑱旂綉鏍囪锛岄噸鏂伴厤缃戝悗搴斿啀娆℃挱鎶ュ畬鏁存彁绀?
        Settings hover_settings(kHoverNvsNamespace, true);
        hover_settings.SetBool(kFirstWifiConnectedKey, false);

        ESP_LOGW(TAG, "WiFi and agent config cleared");
    }

    // 杩涘害鐜瘡绉掑～鍏呬竴鍧楋紱绗?10 鍧楀～婊℃椂鑷姩杩涘叆鏍囧畾妯″紡
    void OnTouchProgressTick() {
        if (!touch_long_press_active_ || calibration_mode_) {
            return;
        }
        const int64_t elapsed_ms = esp_timer_get_time() / 1000 - touch_press_start_time_;
        int segments = (int)(elapsed_ms / 1000);
        if (segments > kCalibrationSteps) {
            segments = kCalibrationSteps;
        }
        if (segments > touch_progress_segments_) {
            touch_progress_segments_ = segments;
            ShowCalibrationProgress(touch_progress_segments_, false);
        }
        if (segments >= 2 && !calibration_warning_played_) {
            calibration_warning_played_ = true;
            ESP_LOGW(TAG, "Calibration warning scheduled at 2s, lang=%s", Lang::CodeStr());
            // 独立任务播放，避免音频队列忙时阻塞倒计时定时器
            xTaskCreate([](void*) {
                auto& app = Application::GetInstance();
                app.GetAudioService().ResetDecoder();
                app.PlaySound(CalibrationWarningPrompt());
                ESP_LOGW(TAG, "Calibration warning played at 2s, lang=%s", Lang::CodeStr());
                vTaskDelete(nullptr);
            }, "calib_warning", 4096, nullptr, 4, nullptr);
        }
        if (elapsed_ms >= kTouchCalibrationMs) {
            EnterCalibrationMode();
        }
    }

    void PrepareServoCalibration(bool reset_network) {
        if (reset_network) {
            ClearWifiAndAgentConfig();
        }
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_CALIBRATION_ENTER());
        servo_calibration_ready_ = true;
    }

    void EnterCalibrationMode() {
        if (calibration_mode_.exchange(true)) {
            return;
        }
        servo_calibration_ready_ = false;
        touch_long_press_active_ = false;
        touch_progress_segments_ = kCalibrationSteps;
        esp_timer_stop(touch_progress_timer_);

        const bool startup = startup_calibration_pending_;
        ESP_LOGW(TAG, "%s: enter calibration mode", startup ? "Startup without calibration" : "10s touch");
        EnterServoCalibration();
        if (!startup) {
            ClearSavedHeadZero();
        }

        // 标定期间显示专用标定图，路径与 ARM 一致：boards/hover/emoji/calibration.eaf
        // 先让图片在进度圈下面加载，再隐藏进度圈，避免黑屏闪一下
        if (display_) {
            display_->SetEmotionOverride("calibration", true);
            ESP_LOGW(TAG, "Calibration image shown: %s", display_->GetEmotion());
        }
        vTaskDelay(pdMS_TO_TICKS(150));
        HideCalibrationProgress();

        if (startup) {
            PrepareServoCalibration(false);
            return;
        }

        xTaskCreate([](void* arg) {
            static_cast<HoverBoard*>(arg)->PrepareServoCalibration(true);
            vTaskDelete(nullptr);
        }, "calib_enter", 4096, this, 4, nullptr);
    }

    void ShowLaunchOverlay(int duration_ms, const char* next_face) {
        std::lock_guard<std::mutex> lock(calibration_ui_mutex_);
        if (prompt_task_ && emote_set_dialog_anim(display_->GetEmoteHandle(), "launch") == ESP_OK) {
            display_->SetEmotion(next_face);
            launch_overlay_until_us_ = esp_timer_get_time() + duration_ms * 1000LL;
        } else {
            display_->SetEmotion(next_face);
            ESP_LOGW(TAG, "Launch overlay unavailable");
        }
    }

    // 鏍囧畾妯″紡涓嬪啀娆¤Е鎽革細鎶婂綋鍓嶄綅缃啓涓?3 鍙疯埖鏈轰腑浣嶅苟鎭㈠鍔涚煩
    void FinishCalibrationMode() {
        if (!calibration_mode_) {
            return;
        }
        if (!servo_calibration_ready_) {
            ESP_LOGW(TAG, "Servo calibration is preparing, ignore touch");
            return;
        }
        if (!FinishServoCalibration()) {
            ESP_LOGE(TAG, "Servo calibration failed, stay in calibration mode");
            return;
        }

        const bool startup = startup_calibration_pending_;
        servo_calibration_ready_ = false;

        if (display_) {
            display_->SetEmotionOverride(nullptr, false);
            const bool reconnect = startup && !SsidManager::GetInstance().GetSsidList().empty();
            ShowLaunchOverlay(3000, reconnect ? "scanning" : "wificonfig");
        }
        HideCalibrationProgress();
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_CALIBRATION_EXIT());
        ESP_LOGW(TAG, "Calibration completed: current position saved as servo 3 center");
        calibration_mode_ = false;

        if (startup) {
            // Resume normal startup, which reconnects or starts provisioning.
            startup_calibration_pending_ = false;
        } else if (Application::GetInstance().GetDeviceState() != kDeviceStateWifiConfiguring) {
            EnterWifiConfigMode();
        }
    }

    void OnTouchPressDown() {
        if (calibration_mode_) {
            suppress_touch_release_ = true;
            FinishCalibrationMode();
            return;
        }
        if (startup_calibration_pending_ || touch_long_press_active_) {
            return;
        }
        touch_press_start_time_ = esp_timer_get_time() / 1000;
        touch_press_chat_state_ = Application::GetInstance().GetChatTouchState();
        touch_progress_segments_ = -1;
        calibration_warning_played_ = false;
        touch_long_press_active_ = true;
        suppress_touch_release_ = false;
        esp_timer_stop(touch_progress_timer_);
        esp_timer_start_periodic(touch_progress_timer_, 500 * 1000);
        ESP_LOGI(TAG, "Touch pressed, hold 10s to enter calibration");
    }

    void OnTouchPressUp() {
        if (suppress_touch_release_) {
            suppress_touch_release_ = false;
            return;
        }
        if (calibration_mode_ || !touch_long_press_active_) {
            return;
        }

        touch_long_press_active_ = false;
        esp_timer_stop(touch_progress_timer_);
        const int64_t duration = esp_timer_get_time() / 1000 - touch_press_start_time_;
        if (duration >= kTouchCalibrationMs) {
            EnterCalibrationMode();
            return;
        }
        if (touch_progress_segments_ >= 0) {
            touch_progress_segments_ = -1;
            // The countdown is an overlay. The underlying animation may
            // have changed while the button was held.
            HideCalibrationProgress();
            ESP_LOGW(TAG, "Countdown cancelled after %d ms", (int)duration);
            return;
        }

        // 记住第一次按下时的对话状态，避免 TTS 在双击窗口内结束后误切到待机。
        touch_tap_count_++;
        if (touch_tap_count_ == 1) {
            tap_chat_state_ = touch_press_chat_state_;
            esp_timer_start_once(tap_timer_, kDoubleTapWindowMs * 1000);
        }
    }

    void OnTapTimer() {
        const int taps = touch_tap_count_;
        touch_tap_count_ = 0;
        if (touch_long_press_active_ || calibration_mode_) {
            return;
        }

        auto& app = Application::GetInstance();
        if (taps == 1) {
            const auto state = tap_chat_state_;
            if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                if (state == kDeviceStateListening) {
                    RobotSetHeadAngle(0.0f);
                }
                app.HandleChatTouch(state);
                return;
            }

            if (state == kDeviceStateIdle) {
                touch_listen_requested_.store(true);
                return;
            }
            app.Schedule([]() {
                RobotGesturePlay(static_cast<int>(hover_rl::GestureId::kNo),
                                 static_cast<int>(hover_rl::GestureId::kShy));
            });
            app.PlaySound(STARTUP_SOUND);
        } else if (taps >= 2) {
            app.PlaySound(THROTTLE_SOUND);
            stable_yaw += 360.0f;
            yaw_ctrl_time = esp_timer_get_time() / 1000.0;
            ESP_LOGI(TAG, "Double tap: spin 360° + throttle sound");
        }
    }

    void InitializeUart() {
        uart_config_t uart_cfg = {
            .baud_rate = 1000000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        ESP_LOGI(TAG, "Initialize UART");
        uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0);
        uart_param_config(UART_NUM_2, &uart_cfg);
        uart_set_pin(UART_NUM_2, MOTOR_UART_TX_PIN, MOTOR_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (startup_calibration_pending_) {
            EnterServoCalibration();
        } else {
            WriteByte_P_V(3, head_zero_pos, 1300);
            EnableServo(3, 1);
        }
    }

    void InitializeBootButton() {
        esp_rom_gpio_pad_select_gpio(GPIO_NUM_0);
        gpio_reset_pin(GPIO_NUM_0);
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << GPIO_NUM_0),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 80 * 1000 * 1000;  // 80MHz SPI
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install GC9A01 LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        // 使用默认GC9A01初始化，不使用自定义gc9107命令
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);  // 打开显示

        // 使用 EmoteDisplay 播放 AAF 动画
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        ESP_LOGI(TAG, "Using EmoteDisplay for AAF animations");
    }

    void InitializeCamera() {
        camera_config_t camera_config = {
            .pin_pwdn = CAMERA_PIN_PWDN,
            .pin_reset = CAMERA_PIN_RESET,
            .pin_xclk = CAMERA_PIN_XCLK,
            .pin_sccb_sda = CAMERA_PIN_SIOD,
            .pin_sccb_scl = CAMERA_PIN_SIOC,
            .pin_d7 = CAMERA_PIN_D7,
            .pin_d6 = CAMERA_PIN_D6,
            .pin_d5 = CAMERA_PIN_D5,
            .pin_d4 = CAMERA_PIN_D4,
            .pin_d3 = CAMERA_PIN_D3,
            .pin_d2 = CAMERA_PIN_D2,
            .pin_d1 = CAMERA_PIN_D1,
            .pin_d0 = CAMERA_PIN_D0,
            .pin_vsync = CAMERA_PIN_VSYNC,
            .pin_href = CAMERA_PIN_HREF,
            .pin_pclk = CAMERA_PIN_PCLK,
            .xclk_freq_hz = XCLK_FREQ_HZ,
            .ledc_timer = LEDC_TIMER_0,
            .ledc_channel = LEDC_CHANNEL_0,
            .pixel_format = PIXFORMAT_RGB565,
            .frame_size = FRAMESIZE_240X240,
            .jpeg_quality = 12,
            .fb_count = 2,
            .fb_location = CAMERA_FB_IN_PSRAM,
            .grab_mode = CAMERA_GRAB_LATEST,  // 始终获取最新帧
            .sccb_i2c_port = 1,  // 摄像头用GPIO 4/5，IMU用GPIO 48/14，必须用不同端口
        };

        // 先检查摄像头是否可用
        sensor_t* sensor = esp_camera_sensor_get();
        if (sensor != nullptr) {
            // 传感器已存在，不需要重复初始化
            ESP_LOGW(TAG, "Camera sensor already initialized");
            camera_ = nullptr;
            return;
        }

        camera_ = new Esp32Camera(camera_config);
        
        // 检查摄像头传感器是否成功初始化
        sensor = esp_camera_sensor_get();
        if (sensor != nullptr) {
            ESP_LOGI(TAG, "Camera initialized successfully, sensor PID: 0x%x", sensor->id.PID);
        } else {
            ESP_LOGW(TAG, "Camera initialization failed, camera tools will not be available");
            // 娉ㄦ剰: 杩欓噷涓嶅垹闄?camera_ 浠ラ伩鍏嶅鎬佺被鍨嬫瀽鏋勮鍛?
            // 鍐呭瓨娉勬紡閲忓緢灏忥紙瀵硅薄鏈韩寰堝皬锛夛紝涓斿彧浼氬彂鐢熶竴娆?
            camera_ = nullptr;
        }
    }



    void InitializeButtons() {
        // 创建双击检测定时器
        esp_timer_create_args_t tap_timer_args = {
            .callback = [](void* arg) {
                static_cast<HoverBoard*>(arg)->OnTapTimer();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "tap_timer",
        };
        esp_timer_create(&tap_timer_args, &tap_timer_);

        // 创建长按检测定时器
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto board = static_cast<HoverBoard*>(arg);
                // 妫€鏌ユ寜閿槸鍚︿粛琚寜浣?
                if (board->button_press_start_time_ > 0) {
                    ESP_LOGI(TAG, "Long press detected (>1s), showing nvs_reset emotion");
                    if (board->display_) {
                        board->display_->SetEmotion("nvs_reset");
                    }
                    board->nvs_reset_emotion_shown_ = true;
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "long_press_timer",
        };
        esp_timer_create(&timer_args, &long_press_timer_);

        // 触摸长按 10 秒进度定时器（每秒触发一次）
        esp_timer_create_args_t touch_progress_args = {
            .callback = [](void* arg) {
                static_cast<HoverBoard*>(arg)->OnTouchProgressTick();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touch_progress",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&touch_progress_args, &touch_progress_timer_);

        // 璁板綍鎸夐敭鎸変笅鏃堕棿锛屽苟鍚姩瀹氭椂鍣?
        boot_button_.OnPressDown([this]() {
            button_press_start_time_ = esp_timer_get_time() / 1000;  // 杞崲涓烘绉?
            nvs_reset_emotion_shown_ = false;
            ESP_LOGI(TAG, "Button pressed down");
            // 鍚姩1绉掑畾鏃跺櫒锛屾娴嬮暱鎸?
            esp_timer_start_once(long_press_timer_, kLongPressShowEmotionMs * 1000);
        });

        // 妫€鏌ラ暱鎸夋椂闂?
        boot_button_.OnPressUp([this]() {
            // 鍋滄瀹氭椂鍣?
            esp_timer_stop(long_press_timer_);
            
            if (button_press_start_time_ > 0) {
                int64_t press_duration = (esp_timer_get_time() / 1000) - button_press_start_time_;
                ESP_LOGI(TAG, "Button released, press duration: %d ms", press_duration);
                
                if (press_duration >= kLongPressResetMs) {
                    ESP_LOGW(TAG, "Long press detected (>3s), resetting NVS...");
                    // 鎾斁鎻愮ず闊筹紙濡傛灉鍙敤锛?
                    auto& app = Application::GetInstance();
                    app.PlaySound(THROTTLE_SOUND);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    // 清除 NVS
                    esp_err_t ret = nvs_flash_erase();
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "NVS erased successfully");
                    } else {
                        ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(ret));
                    }
                    nvs_flash_init();
                    
                    // 重启设备
                    ESP_LOGI(TAG, "Restarting device in 1 second...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else if (nvs_reset_emotion_shown_) {
                    // 未达到重置时间，但已显示了表情，恢复正常表情
                    ESP_LOGI(TAG, "Long press cancelled, restoring emotion");
                    if (display_) {
                        display_->SetEmotion("neutral");
                    }
                }
                button_press_start_time_ = 0;
                nvs_reset_emotion_shown_ = false;
            }
        });

        // 淇濇寔鍘熸湁鐨勫崟鍑诲姛鑳?
        boot_button_.OnClick([this]() {
            if (startup_calibration_pending_) {
                return;
            }
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiManager::GetInstance().IsConnected()) {
                EnterWifiConfigMode();
            }
            app.ToggleChatState();
        });

        // 触摸单击控制对话，双击旋转，长按 10 秒进入舵机标定。
        touch_button_.OnPressDown([this]() {
            OnTouchPressDown();
        });

        touch_button_.OnPressUp([this]() {
            OnTouchPressUp();
        });
    }

    void SetDogSpeed(int dog_vx, int dog_vyaw, int time) {
        ESP_LOGI(TAG, "SetDogSpeed: vx=%d, vyaw=%d, time=%d", dog_vx, dog_vyaw, time);

        vx = int(2.2 * dog_vx);
        vyaw = int(2.8 * dog_vyaw);
        if (time > 0) {
            vTaskDelay(pdMS_TO_TICKS(time));
        }
        vx = 0.0;
        vyaw = 0.0;
        ESP_LOGI(TAG, "SetDogSpeed: done");
    }


    struct Reminder {
        esp_timer_handle_t timer;
        std::string message;
    };

    static void OnReminder(void* arg) {
        std::unique_ptr<Reminder> reminder(static_cast<Reminder*>(arg));
        esp_timer_delete(reminder->timer);
#if CONFIG_FIRMWARE_REGION_OVERSEAS
        const char* prefix = "Your reminder is due: ";
#else
        const char* prefix = "你之前设置的提醒到点了，提醒内容是：";
#endif
        Application::GetInstance().SendMcpEvent("timer.reminder", prefix + reminder->message);
    }

    static void SetReminder(int seconds, const std::string& message) {
        if (message.find_first_not_of(" \t\r\n") == std::string::npos) {
            throw std::invalid_argument("Reminder message is required");
        }
        auto reminder = std::make_unique<Reminder>(Reminder{nullptr, message});
        esp_timer_create_args_t args = {};
        args.callback = &HoverBoard::OnReminder;
        args.arg = reminder.get();
        args.name = "reminder";
        if (esp_timer_create(&args, &reminder->timer) != ESP_OK) {
            throw std::runtime_error("Cannot create reminder timer");
        }
        // Transfer ownership before arming. The callback releases it on expiry.
        Reminder* pending = reminder.release();
        if (esp_timer_start_once(pending->timer, uint64_t(seconds) * 1000000ULL) != ESP_OK) {
            esp_timer_delete(pending->timer);
            delete pending;
            throw std::runtime_error("Cannot start reminder timer");
        }
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.timer.set_reminder",
            "Set a one-time spoken reminder. 当用户说‘几分钟后提醒我做某事’时使用。"
            "Supply the delay in seconds and the reminder text in the user's language.",
            PropertyList({
                Property("seconds", kPropertyTypeInteger, 1, 86400),
                Property("message", kPropertyTypeString),
            }), [](const PropertyList& properties) -> ReturnValue {
                SetReminder(properties["seconds"].value<int>(), properties["message"].value<std::string>());
                return true;
            });

        mcp_server.AddTool("self.robot.head_angle",
            "用户要求头向左或右转时调用。头部舵机角度 -75 到 75 度，正为左、负为右、0 回中。拿起或未站稳时也可转头。",
            PropertyList({
                Property("angle", kPropertyTypeInteger, -75, 75),
            }), [this](const PropertyList& properties) -> ReturnValue {
                int angle = properties["angle"].value<int>();
                ESP_LOGI(TAG, "SetHeadAngle called with angle=%d", angle);
                if (!RobotSetHeadAngle(angle)) return std::string("正在标定，头部舵机已卸力");
                return true;
            });

        mcp_server.AddTool("self.robot.move",
            "设置前进后退距离,单位厘米,正为前进,负为后退,不说具体值默认10厘米",
            PropertyList({
                Property("distance", kPropertyTypeInteger, -20, 20),
            }), [this](const PropertyList& properties) -> ReturnValue {
                int distance = properties["distance"].value<int>();
                ESP_LOGI(TAG, "SetHeadAngle called with move distance=%d", distance);
                RobotAddMove(distance / 100.0f);   // limited lead, see robot.cc
                return true;
            });

        mcp_server.AddTool("self.robot.rotate",
            "设置运动旋转角度,机身旋转注意和头部旋转区分!单位度,正为左,负为右,不说具体值默认30度",
            PropertyList({
                Property("angle", kPropertyTypeInteger, -180, 180),
            }), [this](const PropertyList& properties) -> ReturnValue {
                int angle = properties["angle"].value<int>();
                ESP_LOGI(TAG, "SetHeadAngle called with rotate angle=%d", angle);
                RobotAddRotate((float)angle);   // 下达命令才yaw闭环运动，平时yaw不闭环（见 robot.cc）
                return true;
            });

        mcp_server.AddTool("self.robot.stop",
            "立即停止前进/后退/旋转：放弃还没走完的移动和转向目标，原地停住。手机遥控松开方向键、或者用户说“停”时调用。",
            PropertyList(std::vector<Property>{}), [](const PropertyList& properties) -> ReturnValue {
                RobotStopMotion();
                return std::string("已停止");
            });

        mcp_server.AddTool("self.robot.gesture",
            "让机器人做一个动作来表达情绪或打招呼，头、身体和屏幕表情一起动，一次大约 3 到 12 秒。name 取值："
            "hello=打招呼（转身、摇头挥手、鞠躬、再转回来），wave=挥手（只动头和鞠躬，不转身），yes=点头（同意、是），no=摇头（不同意、不是），"
            "look=东张西望（好奇），happy=开心兴奋（晃动、摇头），startle=受惊（吓一跳），shy=害羞（转开、退一点），sleepy=打瞌睡，bow=鞠躬（谢谢、再见），spin=转一圈（高兴）。"
            "摇头(no)和左顾右盼(look)拿起时也能做；其他动作需站稳。电量低或动作忙时会拒绝。",
            PropertyList({
                Property("name", kPropertyTypeString)
            }), [](const PropertyList& properties) -> ReturnValue {
                std::string name = properties["name"].value<std::string>();
                for (auto& c : name) {
                    c = (char)tolower((unsigned char)c);
                }
                const hover_rl::GestureId id = hover_rl::GestureFromName(name.c_str());
                if (id == hover_rl::GestureId::kNone) {
                    return std::string("不认识这个动作，可用的有：hello wave yes no look happy startle shy sleepy bow spin");
                }
                switch ((hover_rl::GestureResult)RobotGesturePlay((int)id)) {
                    case hover_rl::GestureResult::kOk:
                        return std::string("好的，开始做动作 ") + hover_rl::GestureInfoOf(id).name;
                    case hover_rl::GestureResult::kBusy:
                        return std::string("正在做别的动作，做完再试");
                    case hover_rl::GestureResult::kNotBalancing:
                        return std::string("机器人现在没有站稳（可能被拿起或在休息），做不了动作");
                    case hover_rl::GestureResult::kNotQuiet:
                        return std::string("机器人刚站稳，还在晃，过一两秒再试");
                    case hover_rl::GestureResult::kLowBattery:
                        return std::string("电量太低，不做动作了，请先充电");
                    case hover_rl::GestureResult::kCooldown:
                        return std::string("刚做完一个动作，稍等一下");
                    default:
                        return std::string("机器人没有响应，稍后再试");
                }
            });

        mcp_server.AddTool("self.robot.set_mode",
            "切换到小车模式或云台模式。用户说「变成云台」「云台模式」「放到台座」时 mode=gimbal；"
            "用户说「变成小车」「小车模式」「恢复平衡」时 mode=car。",
            PropertyList({
                Property("mode", kPropertyTypeString)
            }), [](const PropertyList& properties) -> ReturnValue {
                auto mode = properties["mode"].value<std::string>();
                if (mode == "gimbal" || mode == "yun_tai" || mode == "云台") {
                    SetControlMode(1);
                    return std::string("已切换到云台模式");
                }
                if (mode == "car" || mode == "xiao_che" || mode == "小车") {
                    SetControlMode(0);
                    return std::string("已切换到小车模式");
                }
                return std::string("未知模式，请使用 car 或 gimbal");
            });

        mcp_server.AddTool("self.robot.set_controller",
            "切换平衡控制器，用于对比测试。mode=rl：强化学习策略控制轮子；mode=lqr：原 LQR 控制器；"
            "mode=shadow：LQR 控制，强化学习只在后台计算用于对比（可以安全验证符号）；"
            "mode=rearm：清除强化学习的偏航失控锁定。",
            PropertyList({
                Property("mode", kPropertyTypeString)
            }), [](const PropertyList& properties) -> ReturnValue {
                auto mode = properties["mode"].value<std::string>();
                if (mode == "rl") {
                    rl_mode = RL_MODE_RL;
                    return std::string("已切换到强化学习控制器");
                }
                if (mode == "lqr") {
                    rl_mode = RL_MODE_LQR;
                    return std::string("已切换到原 LQR 控制器");
                }
                if (mode == "shadow") {
                    rl_mode = RL_MODE_SHADOW;
                    return std::string("影子模式：LQR 控制，强化学习后台计算");
                }
                if (mode == "rearm") {
                    RlRearm();
                    return std::string("已清除强化学习的保护锁定");
                }
                return std::string("未知模式，请使用 rl / lqr / shadow / rearm");
            });

        mcp_server.AddTool("self.robot.pitch",
            "云台模式下调整机身俯仰，仅云台模式生效；小车模式调用则直接返回不动作。"
            "用户说抬头、头抬高：relative=1，angle 为正（不说度数默认 15）；"
            "说低头、低下：relative=1，angle 为负（不说度数默认 -15）；"
            "说设定到某角度、俯仰到某度：relative=0，angle 为绝对角度。"
            "0 为水平，正为抬头（内部会取反写入目标），负为低头，范围 ±45。",
            PropertyList({
                Property("angle", kPropertyTypeInteger, 15, -45, 45),
                Property("relative", kPropertyTypeInteger, 0, 0, 1),
            }), [](const PropertyList& properties) -> ReturnValue {
                if (control_mode != 1) {
                    return std::string("当前是小车模式，俯仰调整仅云台模式有效");
                }
                int angle = properties["angle"].value<int>();
                bool relative = properties["relative"].value<int>() != 0;
                SetGimbalPitch((float)angle, relative);
                return std::string("云台俯仰已设为 ") + std::to_string((int)gimbal_pitch_target) + "度";
            });

        mcp_server.AddTool("self.robot.motor11",
            "单独测试11号轮毂电机：speed为-100到100（100=最快正转，30=慢速正转，负值反转），"
            "seconds为持续时间0-10秒（0=立即停止）。仅用于短时间测试，机器人可能失去平衡。",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 30, -100, 100),
                Property("seconds", kPropertyTypeInteger, 2, 0, 10),
            }), [](const PropertyList& properties) -> ReturnValue {
                int speed = properties["speed"].value<int>();
                int seconds = properties["seconds"].value<int>();
                SetWheelMotorTest(11, speed, (uint32_t)seconds * 1000);
                return std::string("11号电机 ") + std::to_string(speed) + "%，"
                     + std::to_string(seconds) + "秒";
            });

        mcp_server.AddTool("self.robot.motor21",
            "单独测试21号轮毂电机：speed为-100到100（100=最快正转，30=慢速正转，负值反转），"
            "seconds为持续时间0-10秒（0=立即停止）。仅用于短时间测试，机器人可能失去平衡。",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 30, -100, 100),
                Property("seconds", kPropertyTypeInteger, 2, 0, 10),
            }), [](const PropertyList& properties) -> ReturnValue {
                int speed = properties["speed"].value<int>();
                int seconds = properties["seconds"].value<int>();
                SetWheelMotorTest(21, speed, (uint32_t)seconds * 1000);
                return std::string("21号电机 ") + std::to_string(speed) + "%，"
                     + std::to_string(seconds) + "秒";
            });

        mcp_server.AddTool("self.robot.motor_stop",
            "立即停止11号和21号电机的独立测试输出",
            PropertyList(std::vector<Property>{}), [](const PropertyList& properties) -> ReturnValue {
                SetWheelMotorTest(11, 0, 0);
                SetWheelMotorTest(21, 0, 0);
                return std::string("11/21号电机测试已停止");
            });

        mcp_server.AddTool("self.status.battery",
            "查询当前电池电量,返回剩余电量百分比",
            PropertyList(std::vector<Property>{}), [this](const PropertyList& properties) -> ReturnValue {
                int level = 0;
                bool charging = false, discharging = false;
                if (GetBatteryLevel(level, charging, discharging)) {
                    return std::string("当前电池电量 ") + std::to_string(level) + "%"
                        + (charging ? ", 正在充电" : "");
                }
                return std::string("暂未读取到电池电压");
            });

    }

    void UpdateTouchListening(Application& app) {
        const bool tapped = touch_listen_requested_.exchange(false);
        if (!tapped && !touch_gesture_playing_) {
            return;
        }
        if (app.GetDeviceState() != kDeviceStateIdle || calibration_mode_) {
            touch_gesture_playing_ = false;
            return;
        }

        constexpr int shake = static_cast<int>(hover_rl::GestureId::kNo);
        if (touch_gesture_playing_) {
            RobotGestureStatus gesture;
            RobotGetGestureStatus(&gesture);
            if ((gesture.active && gesture.id == shake) || touch_long_press_active_) {
                return;
            }
            touch_gesture_playing_ = false;
        } else {
            const int result = RobotGesturePlay(shake, static_cast<int>(hover_rl::GestureId::kShy));
            touch_gesture_playing_ = result == static_cast<int>(hover_rl::GestureResult::kOk);
            if (touch_gesture_playing_) {
                return;
            }
        }
        app.HandleChatTouch(kDeviceStateIdle);
    }

    void RunPromptTask() {
        auto& app = Application::GetInstance();
        auto& audio = app.GetAudioService();
        LowBatteryReminder battery_reminder;
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            const auto sample = GetBatteryVoltageSample();
            const int64_t now_us = esp_timer_get_time();
            {
                std::lock_guard<std::mutex> lock(calibration_ui_mutex_);
                if (launch_overlay_until_us_ && now_us >= launch_overlay_until_us_) {
                    display_->StopAnimDialog();
                    launch_overlay_until_us_ = 0;
                }
            }
            const bool warn_battery = battery_reminder.Update(sample, now_us);
            int face_id = 0;
            bool face_start = false;
            if (RobotTakeGestureFace(&face_id, &face_start) && !calibration_mode_ && !touch_long_press_active_) {
                const char* emotion = hover_rl::GestureInfoOf((hover_rl::GestureId)face_id).emotion;
                Application::GetInstance().Schedule([this, emotion, face_start]() {
                    if (face_start) {
                        display_->SetEmotion(emotion);
                    } else if (strcmp(display_->GetEmotion(), emotion) == 0) {
                        const bool listening = Application::GetInstance().GetDeviceState() == kDeviceStateListening;
                        display_->SetEmotion(listening ? "listen" : "neutral");
                    }
                });
            }
            UpdateTouchListening(app);
            if (!audio.IsRunning()) {
                continue;
            }

            if (RobotTakeStandEvent()) {
                app.PlaySound(STAND_OK_SOUND);
                ESP_LOGI(TAG, "Stand-up prompt");
            } else if (warn_battery && !calibration_mode_ && !touch_long_press_active_ &&
                       app.GetDeviceState() == kDeviceStateIdle && audio.IsIdle()) {
                battery_reminder.MarkPlayed();
                app.PlaySound(LOW_BATTERY_SOUND);
                ESP_LOGW(TAG, "Low battery reminder: %.1f V", sample.volts);
            }
        }
    }

public:
    HoverBoard() : boot_button_(BOOT_BUTTON_GPIO), touch_button_(TOUCH_BUTTON_GPIO) {
        startup_calibration_pending_ = !InitHeadZeroPos();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeTools();
        
        InitializeCamera();
        
        InitializeUart();
        InitializeController();

        // 绔嬪嵆璇诲彇涓€娆＄數姹犵數鍘嬶紝閬垮厤绛夊緟 60 绉掓墠鏈夌數閲忔暟鎹?
        ReadServoVoltage(3);

        InitializeBootButton();
        
        // IMU 鍒濆鍖?
        imu_init();

        // 鏈哄櫒浜烘帶鍒朵换鍔?
        xTaskCreatePinnedToCore([](void* arg) {
            (void)arg;
            while (true) {
                robot_control();
            }
            vTaskDelete(NULL);
        }, "robot_task", 6144, this, 12, &robot_task_handle_, 0);   // 4096 -> 6144: the RL step adds ~1 KB of stack (see RlStatus.stack_free)

        xTaskCreatePinnedToCore([](void* arg) {
            (void)arg;
            while (true) {
                motor_rx();
                vTaskDelay(pdMS_TO_TICKS(3));
            }
            vTaskDelete(NULL);
        }, "motor_rx_task", 4096, this, 12, &motor_rx_task_handle_, 1);
        ESP_LOGI(TAG, "Robot control tasks created");

        xTaskCreatePinnedToCore([](void* arg) {
            (void)arg;
            while (true) {
                imu_read_once();
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            vTaskDelete(NULL);
        }, "imu_task", 4096, this, 12, &imu_task_handle_, 1);
        ESP_LOGI(TAG, "IMU control tasks created");

    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual std::string GetDeviceStatusJson() override {
        std::string base_json = WifiBoard::GetDeviceStatusJson();
        cJSON* root = cJSON_Parse(base_json.c_str());
        if (!root) {
            return base_json;
        }

        imu_read_once();

        cJSON* imu = cJSON_CreateObject();
        cJSON_AddBoolToObject(imu, "initialized", imu_is_initialized());
        cJSON_AddNumberToObject(imu, "roll", roll);
        cJSON_AddNumberToObject(imu, "pitch", pitch);
        cJSON_AddNumberToObject(imu, "yaw", yaw);
        cJSON_AddItemToObject(root, "imu", imu);

        char* json_str = cJSON_PrintUnformatted(root);
        std::string result(json_str);
        cJSON_free(json_str);
        cJSON_Delete(root);

        return result;
    }

    virtual void OnStartup() override {
        xTaskCreate([](void* arg) {
            static_cast<HoverBoard*>(arg)->RunPromptTask();
        }, "hover_prompts", 3072, this, 3, &prompt_task_);
        if (startup_calibration_pending_) {
            display_->SetEmotion("calibration");
        } else {
            ShowLaunchOverlay(2000, "neutral");
        }
    }

    void CheckCalibration(Display*, AudioService&) override {
        if (!startup_calibration_pending_) {
            return;
        }
        EnterCalibrationMode();
        while (startup_calibration_pending_) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    virtual void OnAgentConfigChecked(bool agent_configured) override {
        // 鑱旂綉婵€娲诲悗鐢?Application 鍥炶皟锛氭湇鍔＄鏄惁宸茬粦瀹氭櫤鑳戒綋
        agent_configured_ = agent_configured && !activation_required_;
        ESP_LOGW(TAG, "Agent config checked: configured=%d activation_required=%d",
                 agent_configured_ ? 1 : 0, activation_required_ ? 1 : 0);
    }

    virtual void OnInitializationComplete() override {
        // gpio_set_level(LASER_GPIO, 0);  // 鍒濆鍖栧畬鎴愶紝鍏抽棴婵€鍏?
        ESP_LOGI(TAG, "Initialization complete, laser off");

        // 宸查厤缃櫤鑳戒綋鏃舵挱鏀惧紑鏈洪煶鏁堬紱鏈厤缃椂淇濇寔瀹夐潤锛岃璇煶鎻愮ず鏇存竻妤?
        if (!just_provisioned_ && agent_configured_) {
            Application::GetInstance().PlaySound(STARTUP_SOUND);
        }
    }

    virtual void OnWifiConfigStart() override {
        just_provisioned_ = false;
        ESP_LOGI(TAG, "WiFi config start, reset to stand then keep sit");
    }

    virtual void OnWifiConfigEnd() override {
        // 閰嶇綉缁撴潫锛氳繛鎺ユ垚鍔熷悗鐨勮闊虫彁绀虹粺涓€鐢?OnWifiConnected 鎾斁锛岄伩鍏嶉噸澶嶆彁绀?
#if CONFIG_USE_ESP_BLUFI_WIFI_PROVISIONING
        just_provisioned_ = true;
        ShowWifiConnectedAnimation();
#endif
    }

    virtual void OnWifiConnected() override {
        ShowWifiConnectedAnimation();
        std::string ip = WifiManager::GetInstance().GetIpAddress();
        // sdkconfig 榛樿鏃ュ織绾у埆涓?WARN锛屽繀椤荤敤 LOGW 鎵嶈兘鍦ㄤ覆鍙ｇ湅鍒?
        ESP_LOGW(TAG, "WiFi IP: %s", ip.c_str());

        // 棣栨鎴愬姛鑱旂綉鎾斁瀹屾暣鎻愮ず骞朵繚鎸佺珯濮?15 绉掞紝涔嬪悗鍙挱鏀俱€學iFi宸茶繛鎺ャ€?
        // 提示音按固件区域固定语言（中文固件中文、英文固件英文）
        NotifyWifiConnected();
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        const int percent = GetBatteryVoltageSample().LevelPercent();
        if (percent < 0) {
            return false;
        }
        level = percent;
        charging = false;
        discharging = true;
        return true;
    }

    // Hover 使用引擎声作为成功提示音；未配置智能体时不播放，避免盖过语音提示
    virtual std::string_view GetSuccessSound() override {
        const bool just_provisioned = just_provisioned_.exchange(false);
        return agent_configured_ && !just_provisioned ? THROTTLE_SOUND : std::string_view{};
    }

    virtual std::string GetBoardDescription() override {
        return "一个双轮平衡小车形态机器人，搭载圆形 240x240 LCD 屏幕、ESP32-S3 MCU、8MB PSRAM、"
               "2路智能舵机、IMU 姿态传感器、GC0308 摄像头，支持光剑控制。"
               "2S 锂电池供电(6.6V-8.4V)。";
    }
};

DECLARE_BOARD(HoverBoard);
