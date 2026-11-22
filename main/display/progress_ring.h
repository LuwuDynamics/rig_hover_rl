/**
 * 环形分段进度条渲染（纯软件绘制，RGB565）
 *
 * 给 RIG-Arm 的 240x240 圆屏用：把一圈分成 total 段，已完成的 step 段点亮，
 * 中间显示剩余秒数。用于「长按头部触摸 10 秒 → 进入标定模式」的进度提示。
 *
 * 缓冲区是本机字节序的 RGB565（uint16_t），上屏前由 EmoteDisplay 统一做字节交换。
 */
#ifndef PROGRESS_RING_H
#define PROGRESS_RING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param buf    目标画布（size × size 像素，本机字节序 RGB565）
 * @param size   画布边长（正方形，像素）
 * @param step   已完成段数，0..total
 * @param total  总段数（RIG-Arm 用 10 = 每秒一段）
 */
void draw_progress_ring(uint16_t *buf, int size, int step, int total);

#ifdef __cplusplus
}
#endif

#endif /* PROGRESS_RING_H */
