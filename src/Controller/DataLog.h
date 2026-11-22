#pragma once
#include <SD.h>

// CSV logger on the Teensy 4.1 built-in SD. Serial-monitor scroll is not a
// store -- this is. Writes are no-ops if the card is missing so the loop
// still runs.

class DataLog {
public:
    bool begin();
    bool isReady() const { return _ready; }

    // prefix e.g. "ATT" -> first free ATT001.CSV
    bool open(const char* prefix);
    void write(const char* line);
    void close();
    bool isOpen() const { return _open; }
    const char* fileName() const { return _name; }

private:
    bool _ready = false;
    bool _open  = false;
    char _name[13] = {};
    File _file;
};
