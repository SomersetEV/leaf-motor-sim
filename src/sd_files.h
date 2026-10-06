// microSD access: load profiles and presets into RAM.
// See docs/SIMULATOR_PLAN.md section 6, "Presets and profiles".
// Called from the console task only; the 10 ms tick never touches the card.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "profile.h"

bool sd_begin();
bool sd_ready();

// Reads /profiles/<file> into p (".csv" is added if there is no extension).
// Returns nullptr on success, otherwise an error message; line is set to the
// offending line number for parse errors.
const char *sd_load_profile(const char *file, loads::Profile &p, uint32_t &line);

// Reads /presets/<name>.json into buf. Returns nullptr on success; "missing"
// if there is no such file.
const char *sd_read_preset(const char *name, char *buf, size_t len);

// Lists files in a directory on the serial console, one "ok"-less line each.
void sd_list(const char *dir);
