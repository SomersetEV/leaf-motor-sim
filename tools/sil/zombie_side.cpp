// The Zombie half of the software-in-the-loop harness. Compiled against the
// Stm32-vcu tree; see zombie_side.h.
//
// What runs is the Zombie's own code: Throttle::SpeedLimitCommand and
// Throttle::RampThrottle (src/throttle.cpp), GovLog::Pack4A* (src/govlog.cpp)
// and the parameter store (libopeninv params.cpp). The glue around them below
// mirrors, line for line, the parts of the Zombie that call them in run mode:
//
//   Ms10Task               src/stm32_vcu.cpp  (run-mode branch, then the speed read)
//   utils::ProcessThrottle src/utils.cpp      (minus the pedal read and derates)
//   SendGovLog             src/stm32_vcu.cpp
//   LeafINV::SetTorque, LeafINV::DecodeCAN    src/leafinv.cpp
//   NissLeafMng::Task10Ms  src/NissLeafMng.cpp (the 0x1D4 frame only)
//   Param::Change          src/stm32_vcu.cpp  (the Throttle:: assignments only)
//
// Left out, and so absent from every harness run: the pedal map (the pedal
// is given in percent), UdcLimitCommand, IdcLimitCommand and both
// TemperatureDerate calls. On the bench they do nothing while voltage,
// current and temperatures sit inside their limits.
//
// One detail matters for the loop: Ms10Task calls the inverter's Task10Ms,
// which sends 0x1D4, before SetTorque stores this tick's request, so each
// 0x1D4 carries the previous tick's torque. Bench logs confirm it (every
// 0x1D4 matches the potnom of the 0x4A0 one tick earlier).

#include "zombie_side.h"

#include <math.h>
#include <string.h>

#include "govlog.h"
#include "my_math.h"
#include "params.h"
#include "throttle.h"

// Needed to link libopeninv, as in Stm32-vcu test/test_main.cpp.
const char *errorListString = "";

// Mirror of the Throttle:: assignments in Param::Change (stm32_vcu.cpp).
// The rest of that function configures hardware and chargers.
void Param::Change(Param::PARAM_NUM) {
  Throttle::potmin[0] = Param::GetInt(Param::potmin);
  Throttle::potmax[0] = Param::GetInt(Param::potmax);
  Throttle::potmin[1] = Param::GetInt(Param::pot2min);
  Throttle::potmax[1] = Param::GetInt(Param::pot2max);
  Throttle::regenRpm = Param::GetFloat(Param::regenrpm);
  Throttle::regenendRpm = Param::GetFloat(Param::regenendrpm);
  Throttle::ThrotRpmFilt = Param::GetFloat(Param::throtrpmfilt);
  if (Throttle::regenRpm < Throttle::regenendRpm) {
    Throttle::regenRpm = 1500;
    Throttle::regenendRpm = 100;
    Param::SetFloat(Param::regenrpm, 1500);
    Param::SetFloat(Param::regenendrpm, 100);
  }
  Throttle::regenmax = Param::GetFloat(Param::regenmax);
  Throttle::throtmax = Param::GetFloat(Param::throtmax);
  Throttle::throtmin = Param::GetFloat(Param::throtmin);
  Throttle::throtdead = Param::GetFloat(Param::throtdead);
  Throttle::speedLimit = Param::GetInt(Param::revlim);
  Throttle::govKp = Param::GetFloat(Param::govKp) * 0.01f;
  Throttle::govKi = Param::GetFloat(Param::govKi) * 0.001f;
  Throttle::govKd = Param::GetFloat(Param::govKd);
  Throttle::govImax = Param::GetFloat(Param::govImax);
  Throttle::govFlt = Param::GetInt(Param::govFlt);
  Throttle::regenRamp = Param::GetFloat(Param::regenramp);
  Throttle::throttleRamp = Param::GetFloat(Param::throtramp);
  Throttle::throtmaxRev = Param::GetFloat(Param::throtmaxRev);
  Throttle::regenBrake = Param::GetFloat(Param::regenBrake);
}

namespace zside {

static int16_t s_inv_speed = 0;           // LeafINV::speed
static int16_t s_final_torque_request = 0; // LeafINV::final_torque_request
static uint8_t s_mprun10 = 0;             // NissLeafMng::mprun10
static uint8_t s_gov_counter = 0;         // SendGovLog statics
static int s_gov_slow_div = 0;

// The tractor's governor setup (Ben, 3 Nov 2026, CLAUDE.md), on top of the
// Zombie's defaults. regenramp is not logged anywhere: in the M4 bench log
// it never limited a step, the largest being 13.6 %/10 ms, so the bench
// Zombie (which carries the tractor's saved parameters) has at least that,
// not the default 1. Ben will set the tractor to at least 5 (11 Nov 2026); in
// the harness every value from 3 up behaves the same, so 100, its maximum. Not listed, so at their defaults: throtramprpm 20000, throtmax 100,
// throtmin -100.
static const struct {
  Param::PARAM_NUM p;
  float v;
} TRACTOR[] = {
    {Param::Inverter, 1}, // Leaf_Gen1
    {Param::revlim, 2200},   {Param::govKp, 11},     {Param::govKi, 0},
    {Param::govKd, 0},       {Param::govImax, 100},  {Param::govFlt, 4},
    {Param::regenmax, -30},  {Param::throtramp, 10}, {Param::regenramp, 100},
};

void init() {
  Param::LoadDefaults();
  for (const auto &t : TRACTOR)
    Param::SetFloat(t.p, t.v);
  Param::SetInt(Param::opmode, MOD_RUN);
  Param::SetInt(Param::dir, 1);
  Param::SetInt(Param::speed, 0);
  Param::Change(Param::revlim);
}

const char *set_raw(const char *name, int32_t raw) {
  Param::PARAM_NUM p = Param::NumFromString(name);
  if (p == Param::PARAM_INVALID)
    return "no such Zombie parameter";
  if (Param::Set(p, raw) != 0) // range-checked; calls Param::Change
    return "Zombie refused (abort 0x06090030)";
  return nullptr;
}

bool get(const char *name, float &value) {
  Param::PARAM_NUM p = Param::NumFromString(name);
  if (p == Param::PARAM_INVALID)
    return false;
  value = Param::GetFloat(p);
  return true;
}

void receive_1da(const uint8_t d[8]) {
  // LeafINV::DecodeCAN, id 0x1DA (the speed; voltage and error are not used)
  const uint8_t *bytes = d;
  int16_t parsed_speed = (bytes[4] << 7) | bytes[5] >> 1;
  if (parsed_speed > 0x3fff)
    parsed_speed -= 0x7fff; // 15 bit signed conversion
  s_inv_speed = parsed_speed;
}

// NissLeafMng::nissan_crc
static void nissan_crc(uint8_t *data, uint8_t polynomial) {
  data[7] = 0;
  uint8_t crc = 0;
  for (int b = 0; b < 8; b++) {
    for (int i = 7; i >= 0; i--) {
      uint8_t bit = ((data[b] & (1 << i)) > 0) ? 1 : 0;
      if (crc >= 0x80)
        crc = (uint8_t)(((crc << 1) + bit) ^ polynomial);
      else
        crc = (uint8_t)((crc << 1) + bit);
    }
  }
  data[7] = crc;
}

// NissLeafMng::Task10Ms(final_torque_request), the 0x1D4 frame
static void task10ms_1d4(int16_t final_torque_request, uint8_t bytes[8]) {
  int opmode = Param::GetInt(Param::opmode);
  s_mprun10 = (s_mprun10 + 1) % 4;
  bytes[0] = 0xF7;
  bytes[1] = 0x07;
  if (opmode != MOD_RUN) {
    final_torque_request = 0;
  }
  if (final_torque_request >= -2048 && final_torque_request <= 2047) {
    bytes[2] = ((final_torque_request < 0) ? 0x80 : 0) |
               ((final_torque_request >> 4) & 0x7f);
    bytes[3] = (final_torque_request << 4) & 0xf0;
  } else {
    bytes[2] = 0x00;
    bytes[3] = 0x00;
  }
  bytes[4] = 0x07; // HV on: run mode
  bytes[4] = bytes[4] | (s_mprun10 << 6);
  bytes[5] = 0x44;
  bytes[6] = 0x30;
  nissan_crc(bytes, 0x85);
}

// utils::ProcessThrottle with the pedal given in percent
static float process_throttle(int speed, float pedal_pct) {
  float finalSpnt;

  if (speed < Param::GetInt(Param::throtramprpm)) {
    Throttle::throttleRamp = Param::GetFloat(Param::throtramp);
  } else {
    Throttle::throttleRamp = Param::GetAttrib(Param::throtramp)->max;
  }

  finalSpnt = pedal_pct; // utils::GetUserThrottleCommand()

  Throttle::potUser = finalSpnt;
  Param::SetFloat(Param::potuser, finalSpnt);

  Throttle::SpeedLimitCommand(finalSpnt, ABS(speed));
  // UdcLimitCommand, IdcLimitCommand and TemperatureDerate left out

  finalSpnt = Throttle::RampThrottle(finalSpnt);

  if (finalSpnt < -100.0f)
    finalSpnt = -100.0f;
  else if (finalSpnt > 100.0f)
    finalSpnt = 100.0f;

  Param::SetFloat(Param::potnom, finalSpnt);

  return finalSpnt;
}

Ms10Out ms10(float pedal_pct) {
  Ms10Out o;
  memset(&o, 0, sizeof(o));
  int16_t previousSpeed = Param::GetInt(Param::speed);
  int requestedDirection = Param::GetInt(Param::dir);
  int rollingDirection = 0;

  float torquePercent = process_throttle(ABS(previousSpeed), pedal_pct);
  o.potnom = torquePercent;

  if (Param::GetInt(Param::GovLog)) { // SendGovLog(torquePercent)
    o.govlog = true;
    GovLog::Pack4A0(o.f4a0, torquePercent);
    GovLog::Pack4A1(o.f4a1, s_gov_counter++);
    if (++s_gov_slow_div >= GovLog::SLOW_DIVIDER) {
      s_gov_slow_div = 0;
      o.slow = true;
      GovLog::Pack4A2(o.f4a2);
      GovLog::Pack4A3(o.f4a3);
    }
  }

  if (torquePercent < 0 && Param::GetInt(Param::Inverter) != InvModes::OpenI) {
    if (Param::GetInt(Param::reversemotor) == 0) {
      rollingDirection = previousSpeed >= 0 ? 1 : -1;
    } else {
      rollingDirection = previousSpeed >= 0 ? -1 : 1;
    }
    if (rollingDirection != requestedDirection) {
      torquePercent = -torquePercent;
    }
  }

  torquePercent *= requestedDirection;

  // selectedInverter->Task10Ms(): sends the request stored last tick
  task10ms_1d4(s_final_torque_request, o.f1d4);

  // selectedInverter->SetTorque(torquePercent)
  s_final_torque_request = (torquePercent * 2047) / 100.0f;
  if (Param::GetInt(Param::reversemotor) == 1) {
    s_final_torque_request *= -1;
  }
  Param::SetInt(Param::torque, s_final_torque_request);

  // speed = selectedInverter->GetMotorSpeed()
  int16_t speed = (float)s_inv_speed;
  Param::SetInt(Param::speed, speed);
  return o;
}

// ---- Golden test ------------------------------------------------------------
// GoldenInput and Triangle are copied verbatim from Stm32-vcu
// test/test_throttle.cpp (a05e984); the recorded outputs are included from
// that tree's test/golden_governor.h.

static const int GOLDEN_TICKS = 500;
#include "golden_governor.h"

// Integer triangle wave, amplitude amp, period 4 * amp / step ticks.
static int Triangle(int t, int amp, int step) {
  int period = 4 * amp / step;
  int p = t % period;
  int q = period / 4;
  if (p < q)
    return p * step;
  if (p < 3 * q)
    return (2 * q - p) * step;
  return (p - 4 * q) * step;
}

static void GoldenInput(int t, int &speed, float &throttle) {
  if (t == 0) {
    Throttle::speedLimit = 2000;
    Throttle::govKp = 30 * 0.01f;
    Throttle::govKi = 2 * 0.001f;
    Throttle::govKd = 2;
    Throttle::govImax = 100;
    Throttle::regenmax = -10;
  }
  if (t == 300) {
    Throttle::govKi = 50 * 0.001f;
    Throttle::govImax = 20;
  }

  if (t < 100) { // full throttle acceleration through the limit
    speed = t * 24;
    throttle = 100;
  } else if (t < 200) { // ripple around the limit, baler-like
    speed = 2000 + Triangle(t, 60, 6);
    throttle = 100;
  } else if (t < 250) { // pedal is the binding limit
    speed = 1950 + t % 5;
    throttle = 10;
  } else if (t < 300) { // drop below half the limit: integral reset
    speed = MAX(0, 1950 - (t - 250) * 40);
    throttle = 100;
  } else if (t < 400) { // just under the limit: integral clamps at govImax
    speed = 1990;
    throttle = 100;
  } else if (t < 450) { // over the limit: regen, negative clamp
    speed = 2050;
    throttle = 100;
  } else { // driver regen request below the limit
    speed = 1500;
    throttle = -5;
  }
}

int golden_mismatches(int *first) {
  int mismatches = 0;
  *first = -1;
  for (int t = 0; t < GOLDEN_TICKS; t++) {
    int speed;
    float finalSpnt;
    GoldenInput(t, speed, finalSpnt);
    Throttle::SpeedLimitCommand(finalSpnt, speed);
    uint32_t bits;
    memcpy(&bits, &finalSpnt, sizeof(bits));
    int32_t govInt = Param::Get(Param::govInt);
    if (bits != goldenSpntBits[t] || govInt != goldenGovIntRaw[t]) {
      if (*first < 0)
        *first = t;
      mismatches++;
    }
  }
  return mismatches;
}

} // namespace zside
