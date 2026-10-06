#include "sd_files.h"

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>

#include "config.h"
#include "pins.h"

static SPIClass s_spi(HSPI);
static bool s_ready = false;

bool sd_begin() {
    s_spi.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    s_ready = SD.begin(SD_CS_PIN, s_spi);
    return s_ready;
}

bool sd_ready() { return s_ready; }

// Names are plain file names: no directories, no "..".
static bool safe_name(const char *name) {
    return name[0] && !strchr(name, '/') && !strchr(name, '\\') && !strstr(name, "..");
}

const char *sd_load_profile(const char *file, loads::Profile &p, uint32_t &line) {
    line = 0;
    if (!s_ready) return "no SD card";
    if (!safe_name(file)) return "bad file name";
    char path[64];
    snprintf(path, sizeof(path), "%s%s%s", SD_PROFILE_DIR, file, strchr(file, '.') ? "" : ".csv");
    File f = SD.open(path, FILE_READ);
    if (!f) return "file not found";

    loads::ProfileCsvParser csv(p);
    char buf[96];
    const char *err = nullptr;
    while (f.available()) {
        size_t n = f.readBytesUntil('\n', buf, sizeof(buf) - 1);
        buf[n] = '\0';
        if (n == sizeof(buf) - 1 && f.available() && f.peek() != '\n') {
            err = "line too long";
            line = csv.line_number() + 1;
            break;
        }
        if (!csv.line(buf)) {
            err = csv.error();
            line = csv.line_number();
            break;
        }
    }
    f.close();
    if (!err && !csv.finish()) err = csv.error();
    if (err) p.clear();
    return err;
}

const char *sd_read_preset(const char *name, char *buf, size_t len) {
    if (!s_ready) return "missing";
    if (!safe_name(name)) return "bad preset name";
    char path[64];
    snprintf(path, sizeof(path), "%s%s.json", SD_PRESET_DIR, name);
    if (!SD.exists(path)) return "missing";
    File f = SD.open(path, FILE_READ);
    if (!f) return "missing";
    size_t n = f.read((uint8_t *)buf, len - 1);
    bool too_big = f.available() > 0;
    f.close();
    if (too_big) return "preset file too large";
    buf[n] = '\0';
    return nullptr;
}

void sd_list(const char *dir) {
    if (!s_ready) return;
    File d = SD.open(dir);
    if (!d || !d.isDirectory()) return;
    for (File e = d.openNextFile(); e; e = d.openNextFile()) {
        if (!e.isDirectory()) Serial.printf("  %s %lu\n", e.name(), (unsigned long)e.size());
        e.close();
    }
    d.close();
}
