#include "params_store.h"

#include <Preferences.h>

#include "param_table.h"

// Bump when SimParams changes layout, so an old blob is ignored.
static constexpr uint32_t PARAMS_LAYOUT = 1;

bool params_load(SimParams &p) {
    Preferences prefs;
    if (!prefs.begin("leafsim", true)) return false;
    SimParams tmp;
    bool ok = prefs.getUInt("layout", 0) == PARAMS_LAYOUT &&
              prefs.getBytes("params", &tmp, sizeof(tmp)) == sizeof(tmp) &&
              simcore::params_valid(tmp);
    prefs.end();
    if (ok) p = tmp;
    return ok;
}

bool params_save(const SimParams &p) {
    Preferences prefs;
    if (!prefs.begin("leafsim", false)) return false;
    bool ok = prefs.putBytes("params", &p, sizeof(p)) == sizeof(p) &&
              prefs.putUInt("layout", PARAMS_LAYOUT) == sizeof(uint32_t);
    prefs.end();
    return ok;
}
