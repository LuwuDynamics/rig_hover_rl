// Build with c++ -std=c++17 -Imain/boards/hover from the repository root.
#include "gesture_player.h"
#include <cassert>
#include <initializer_list>

using namespace hover_rl;

static void check_carried_gesture(GestureId id, bool start_balancing) {
    GesturePlayer player;
    GestureCtx ctx;
    ctx.balancing = start_balancing;
    assert(player.request(id, ctx) == GestureResult::kOk);
    assert(player.step(ctx).started);

    ctx.balancing = false;
    ctx.lean_deg = 85.0f;
    ctx.v_wheel = 0.8f;
    bool moved_left = false;
    bool moved_right = false;
    for (ctx.now_us = 10000; ctx.now_us <= 5000000; ctx.now_us += 10000) {
        const auto out = player.step(ctx);
        assert(out.cancelled == GestureCancel::kNone);
        assert(out.bias_deg == 0 && out.n_events == 0 && out.head_on);
        moved_left |= out.head_deg > 5.0f;
        moved_right |= out.head_deg < -5.0f;
        if (out.finished) {
            assert(moved_left && moved_right && out.head_deg == 0);
            return;
        }
    }
    assert(false && "gesture did not finish");
}

int main() {
    for (GestureId id : {GestureId::kNo, GestureId::kLook}) {
        check_carried_gesture(id, false);
        check_carried_gesture(id, true);
        GesturePlayer player;
        GestureCtx ctx;
        ctx.battery_v = 7.0f;
        assert(player.request(id, ctx) == GestureResult::kLowBattery);
        ctx.battery_v = 7.5f;
        assert(player.request(id, ctx) == GestureResult::kOk);
        ctx.battery_v = 6.8f;
        assert(player.step(ctx).cancelled == GestureCancel::kBattery);
    }
    for (int i = 1; i < static_cast<int>(GestureId::kCount); ++i) {
        const auto id = static_cast<GestureId>(i);
        if (id == GestureId::kNo || id == GestureId::kLook) continue;
        GesturePlayer player;
        GestureCtx ctx;
        assert(player.request(id, ctx) == GestureResult::kNotBalancing);
        ctx.balancing = true;
        assert(player.request(id, ctx) == GestureResult::kNotQuiet);
        for (int j = 0; j < 125; ++j) {
            ctx.now_us += 10000;
            player.step(ctx);
        }
        assert(player.request(id, ctx) == GestureResult::kOk);
        ctx.balancing = false;
        assert(player.step(ctx).cancelled == GestureCancel::kNotBalancing);
    }
}
