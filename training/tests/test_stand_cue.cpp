// Build with c++ -std=c++17 -Imain/boards/hover from the repository root.
#include "rl_controller.h"
#include <cassert>

using namespace hover_rl;

static Output tick(RlController& controller, int64_t time_us, float pitch_rate = 0.0f) {
    Signals s{};
    s.now_us = s.wheel1_us = s.wheel2_us = time_us;
    s.imu_ok = true;
    s.dq_dps = pitch_rate;
    return controller.step(s);
}

int main() {
    RlController controller;
    RlController muted;
    muted.cfg.stand_ok_enable = false;

    // The cue and full torque handover coincide on a steady start.
    for (int ms = 5; ms <= 1000; ms += 5) {
        auto out = tick(controller, ms * 1000);
        auto silent = tick(muted, ms * 1000);
        assert(out.stood == (ms == 260));
        if (out.stood) assert(out.ramp == 1.0f);
        assert(!silent.stood && silent.tor1 == out.tor1 && silent.tor2 == out.tor2);
    }

    // A moving robot needs a new uninterrupted steady window after it settles.
    controller.reset();
    for (int ms = 5; ms <= 1200; ms += 5) {
        auto out = tick(controller, ms * 1000, ms <= 500 ? 30.0f : 0.0f);
        assert(out.stood == (ms == 755));
    }

    // Time without feedback must not count toward release.
    controller.reset();
    for (int ms = 5; ms <= 240; ms += 5) assert(!tick(controller, ms * 1000).stood);
    for (int ms = 300; ms <= 1000; ms += 5) {
        assert(tick(controller, ms * 1000).stood == (ms == 555));
    }
}
