#include "progress_ring.h"

#include <math.h>
#include <string.h>

#define PR_PI 3.14159265f

// 3x5 点阵数字，行优先，每行低 3 位有效（bit2 = 最左列）
static const uint8_t kDigits[10][5] = {
    {0x7, 0x5, 0x5, 0x5, 0x7},  // 0
    {0x2, 0x6, 0x2, 0x2, 0x7},  // 1
    {0x7, 0x1, 0x7, 0x4, 0x7},  // 2
    {0x7, 0x1, 0x7, 0x1, 0x7},  // 3
    {0x5, 0x5, 0x7, 0x1, 0x1},  // 4
    {0x7, 0x4, 0x7, 0x1, 0x7},  // 5
    {0x7, 0x4, 0x7, 0x5, 0x7},  // 6
    {0x7, 0x1, 0x1, 0x2, 0x2},  // 7
    {0x7, 0x5, 0x7, 0x5, 0x7},  // 8
    {0x7, 0x5, 0x7, 0x1, 0x7},  // 9
};

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3));
}

static inline uint16_t lerp565(uint16_t a, uint16_t b, float t)
{
    int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
    int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
    int r = ar + (int)((br - ar) * t);
    int g = ag + (int)((bg - ag) * t);
    int bl = ab + (int)((bb - ab) * t);
    return (uint16_t)((r << 11) | (g << 5) | bl);
}

static void draw_digit(uint16_t *buf, int size, int x0, int y0, int scale, int digit, uint16_t color)
{
    if (digit < 0 || digit > 9) {
        return;
    }
    for (int row = 0; row < 5; row++) {
        uint8_t bits = kDigits[digit][row];
        for (int col = 0; col < 3; col++) {
            if ((bits & (1 << (2 - col))) == 0) {
                continue;
            }
            for (int dy = 0; dy < scale; dy++) {
                int y = y0 + row * scale + dy;
                if (y < 0 || y >= size) {
                    continue;
                }
                uint16_t *line = buf + (size_t)y * size;
                for (int dx = 0; dx < scale; dx++) {
                    int x = x0 + col * scale + dx;
                    if (x < 0 || x >= size) {
                        continue;
                    }
                    line[x] = color;
                }
            }
        }
    }
}

void draw_progress_ring(uint16_t *buf, int size, int step, int total)
{
    if (buf == NULL || size <= 0 || total <= 0) {
        return;
    }
    if (step < 0) {
        step = 0;
    }
    if (step > total) {
        step = total;
    }

    const uint16_t c_track    = rgb565(10, 12, 16);     // 未点亮：接近黑，弱化成一圈暗影
    const uint16_t c_fill_a   = rgb565(0, 220, 130);   // 起始：青绿
    const uint16_t c_fill_b   = rgb565(255, 64, 48);   // 结束：红（提醒这是重置操作）
    const uint16_t c_text     = rgb565(255, 255, 255);
    const uint16_t c_text_hot = rgb565(255, 90, 70);

    memset(buf, 0, (size_t)size * (size_t)size * sizeof(uint16_t));

    const float cx = (float)(size - 1) * 0.5f;
    const float cy = (float)(size - 1) * 0.5f;
    const float r_out = (float)size * 0.44f;    // 240 -> 105.6
    const float r_in  = (float)size * 0.35f;    // 240 -> 84
    const float r_out2 = r_out * r_out;
    const float r_in2  = r_in * r_in;

    const float seg_deg = 360.0f / (float)total;
    const float gap_deg = seg_deg * 0.18f;      // 段与段之间留 18% 空隙

    for (int y = 0; y < size; y++) {
        float dy = (float)y - cy;
        uint16_t *line = buf + (size_t)y * size;
        for (int x = 0; x < size; x++) {
            float dx = (float)x - cx;
            float r2 = dx * dx + dy * dy;
            if (r2 < r_in2 || r2 > r_out2) {
                continue;   // 环带以外保持黑底，省掉 sqrt/atan
            }

            // 角度：顶部为 0，顺时针 0..360
            float ang = atan2f(dy, dx) * (180.0f / PR_PI) + 90.0f;
            if (ang < 0.0f) {
                ang += 360.0f;
            }
            int seg = (int)(ang / seg_deg);
            if (seg >= total) {
                seg = total - 1;
            }
            float in_seg = ang - (float)seg * seg_deg;
            if (in_seg < gap_deg * 0.5f || in_seg > seg_deg - gap_deg * 0.5f) {
                continue;   // 段间隙
            }

            uint16_t color;
            if (seg < step) {
                float t = (total > 1) ? (float)seg / (float)(total - 1) : 1.0f;
                color = lerp565(c_fill_a, c_fill_b, t);
            } else {
                color = c_track;
            }
            line[x] = color;
        }
    }

    // 中央倒计时数字：10 → 0
    int remain = total - step;
    int scale = size / 40;
    if (scale < 1) {
        scale = 1;
    }
    const int dw = 3 * scale;
    const int dh = 5 * scale;
    const int gap = scale;

    if (remain >= 10) {
        int total_w = dw * 2 + gap;
        int x0 = (size - total_w) / 2;
        int y0 = (size - dh) / 2;
        draw_digit(buf, size, x0, y0, scale, 1, c_text);
        draw_digit(buf, size, x0 + dw + gap, y0, scale, 0, c_text);
    } else {
        int x0 = (size - dw) / 2;
        int y0 = (size - dh) / 2;
        uint16_t c = (remain <= 3) ? c_text_hot : c_text;
        draw_digit(buf, size, x0, y0, scale, remain, c);
    }
}
