#pragma once

#include "display.h"
#include <memory>
#include <string>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_timer.h>
#include "expression_emote.h"
#include "gfx.h"

namespace emote {

class EmoteDisplay : public Display {
public:
    EmoteDisplay(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io, int width, int height);
    virtual ~EmoteDisplay();

    virtual void SetEmotion(const char* emotion) override;
    virtual const char* GetEmotion() override { return current_emotion_.c_str(); }
    virtual void SetEmotionOverride(const char* emotion, bool enable) override;
    virtual void SetStatus(const char* status) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void SetTheme(Theme* theme) override;
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override;
    virtual void UpdateStatusBar(bool update_all = false) override;
    virtual void SetPowerSaveMode(bool on) override;
    virtual void SetPreviewRgb565(const void* data, int width, int height, int stride) override;
    virtual void SetPreviewImage(const void* image);
    // 分段环形进度（长按触摸 10 秒进入标定时的进度圈），total<=0 隐藏
    virtual void ShowProgressRing(int step, int total) override;

    bool StopAnimDialog();
    bool InsertAnimDialog(const char* emoji_name, uint32_t duration_ms);

    void RefreshAll();

    // Get emote handle for internal use
    emote_handle_t GetEmoteHandle() const { return emote_handle_; }

private:
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    emote_handle_t emote_handle_ = nullptr;
    gfx_obj_t* preview_obj_ = nullptr;  // 预览图像对象
    gfx_image_dsc_t preview_img_dsc_ = {};  // 预览图像描述符
    uint8_t* preview_data_ = nullptr;  // 预览数据副本
    size_t preview_data_size_ = 0;  // 预览数据大小
    esp_timer_handle_t preview_timer_ = nullptr;  // 预览隐藏定时器

    uint16_t* progress_buf_[2] = {nullptr, nullptr};  // 环形进度条双缓冲（PSRAM）
    int progress_buf_size_ = 0;          // 单块画布像素数
    int progress_idx_ = 0;               // 当前正在显示的缓冲索引
    gfx_obj_t* progress_obj_ = nullptr;  // 进度圈专用图像对象（独立图层，避免刷新闪烁）
    gfx_image_dsc_t progress_dsc_ = {};  // 进度圈图像描述符
    bool progress_visible_ = false;      // 当前是否已显示
    std::string current_emotion_ = "neutral";  // 当前表情名
    std::string override_emotion_;       // 表情锁定目标（标定图片）
    bool override_active_ = false;       // 锁定期间忽略其他 SetEmotion

};

} // namespace emote
