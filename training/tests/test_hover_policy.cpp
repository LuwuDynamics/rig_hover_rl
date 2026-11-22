// Replays simulator ticks through hover_rl::Policy and compares with what the simulator + numpy actor produced.
// Build: clang++ -O2 -std=c++17 -I<artifact dir> -I<sim>/firmware tests/test_hover_policy.cpp -o test_hover_policy
// Run:   ./test_hover_policy <artifact dir>/test_vectors.txt
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "hover_policy.h"

static int run_file(const char* path, bool verbose) {
    FILE* f = std::fopen(path, "r");
    if (!f) { std::perror(path); return 2; }
    hover_rl::Policy p;
    p.reset();
    int n = 0, bad = 0;
    double max_act = 0, max_cmd = 0, max_int = 0, max_vcmd = 0;
    double dt, pitch, q, r, v, vt, rt, vcmd, rcmd, ev, er, ep, a0, a1, left, right;
    while (std::fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &dt, &pitch, &q, &r, &v, &vt, &rt,
                       &vcmd, &rcmd, &ev, &er, &ep, &a0, &a1, &left, &right) == 16) {
        hover_rl::Inputs in{(float)pitch, (float)q, (float)r, (float)v, (float)vt, (float)rt, (float)dt};
        hover_rl::Outputs o = p.step(in);
        double d_act = std::fmax(std::fabs(o.act_common - a0), std::fabs(o.act_diff - a1));
        double d_cmd = std::fmax(std::fabs(o.left - left), std::fabs(o.right - right));
        double d_int = std::fmax(std::fmax(std::fabs(p.speedErrorIntegral() - ev), std::fabs(p.yawErrorIntegral() - er)),
                                 std::fabs(p.positionErrorIntegral() - ep));
        double d_v = std::fmax(std::fabs(p.speedCommand() - vcmd), std::fabs(p.yawRateCommand() - rcmd));
        max_act = std::fmax(max_act, d_act); max_cmd = std::fmax(max_cmd, d_cmd);
        max_int = std::fmax(max_int, d_int); max_vcmd = std::fmax(max_vcmd, d_v);
        if (d_act > 2e-3 || d_int > 1e-4 || d_v > 1e-4) bad++;
        n++;
    }
    std::fclose(f);
    if (verbose)
        std::printf("%s: ticks %d  max|act diff| %.3e  max|cmd diff| %.3e units  max|integral diff| %.3e  max|command-shaping diff| %.3e  bad %d\n",
                    path, n, max_act, max_cmd, max_int, max_vcmd, bad);
    return (n > 0 && bad == 0) ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: %s test_vectors_1.txt [more files]\n", argv[0]); return 2; }
    int rc = 0;
    for (int i = 1; i < argc; i++) rc |= run_file(argv[i], true);
    return rc;
}
