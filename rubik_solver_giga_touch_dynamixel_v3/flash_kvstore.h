// ==== flash_kvstore.h ====
#pragma once
#include <cstring>
#include "mbed.h"              // brings in core mbed definitions
#include "drivers/FlashIAP.h"  // defines class FlashIAP

using namespace mbed;  // optional, to avoid writing mbed::FlashIAP

// Reserve one 4-kB sector near the end of flash
#define FLASH_KV_SECTOR_SIZE (4096)
#define FLASH_KV_START_ADDR \
  (flash.get_flash_start() + flash.get_flash_size() - FLASH_KV_SECTOR_SIZE)

class FlashKVStore {
public:
  void begin() {
    flash.init();
  }
  void end() {
    flash.deinit();
  }

  // write one 32-bit value under a short key (<= 15 chars)
  void putInt(const char *key, int value) {
    char buf[FLASH_KV_SECTOR_SIZE];
    memset(buf, 0xFF, sizeof(buf));

    // read existing sector
    flash.read(buf, FLASH_KV_START_ADDR, sizeof(buf));

    // store key=value as "key=value\n"
    char line[32];
    snprintf(line, sizeof(line), "%s=%d\n", key, value);
    strcat(buf, line);

    flash.erase(FLASH_KV_START_ADDR, FLASH_KV_SECTOR_SIZE);
    flash.program(buf, FLASH_KV_START_ADDR, sizeof(buf));
  }

  // read key; return def if not found
  int getInt(const char *key, int def = 0) {
    char buf[FLASH_KV_SECTOR_SIZE];
    flash.read(buf, FLASH_KV_START_ADDR, sizeof(buf));

    char *line = strtok(buf, "\n");
    while (line) {
      char k[16];
      int v;
      if (sscanf(line, "%15[^=]=%d", k, &v) == 2) {
        if (strcmp(k, key) == 0) return v;
      }
      line = strtok(nullptr, "\n");
    }
    return def;
  }

private:
  FlashIAP flash;
};
