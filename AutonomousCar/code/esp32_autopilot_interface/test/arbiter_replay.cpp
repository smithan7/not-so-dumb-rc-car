// Replays Arbiter inputs from stdin and prints its outputs, so tests can check that the Python
// port (failsafe.py, used by car_emulator.py) behaves exactly like failsafe.h.
//   in : now rc_seen rc_last ch3 rc_speed rc_steer j_seen j_last j_speed j_steer enc_seen enc_last
//   out: mode speed_us steer_us armed
#include <cstdio>

#include "../failsafe.h"

int main() {
  car::Config cfg;
  car::Arbiter arb(cfg);
  car::Inputs in;
  unsigned long now, rc_last, j_last, enc_last;
  int rc_seen, j_seen, enc_seen;
  while (std::scanf("%lu %d %lu %d %d %d %d %lu %d %d %d %lu", &now, &rc_seen, &rc_last, &in.rc_ch3_us,
                    &in.rc_speed_us, &in.rc_steer_us, &j_seen, &j_last, &in.jetson_speed_us, &in.jetson_steer_us,
                    &enc_seen, &enc_last) == 12) {
    in.now_ms = (uint32_t)now;
    in.rc_seen = rc_seen != 0;
    in.rc_last_ms = (uint32_t)rc_last;
    in.jetson_seen = j_seen != 0;
    in.jetson_last_ms = (uint32_t)j_last;
    in.enc_seen = enc_seen != 0;
    in.enc_last_edge_ms = (uint32_t)enc_last;
    car::Output o = arb.update(in);
    std::printf("%d %d %d %d\n", o.mode, o.speed_us, o.steer_us, o.armed ? 1 : 0);
  }
  return 0;
}
