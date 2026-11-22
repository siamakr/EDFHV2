#include "DataLog.h"
#include "consts.h"
#include <stdio.h>

bool DataLog::begin()
{
    if (!cfg::log::SD_ENABLE) {
        _ready = false;
        return false;
    }
    _ready = SD.begin(BUILTIN_SDCARD);
    return _ready;
}

bool DataLog::open(const char* prefix)
{
    close();
    if (!_ready) return false;

    char pre[4] = { 'L', 'O', 'G', 0 };
    if (prefix && prefix[0]) {
        pre[0] = prefix[0];
        pre[1] = prefix[1] ? prefix[1] : 'X';
        pre[2] = prefix[2] ? prefix[2] : 'X';
        pre[3] = 0;
    }
    for (int i = 1; i <= 999; ++i) {
        snprintf(_name, sizeof(_name), "%s%03d.CSV", pre, i);
        if (!SD.exists(_name)) break;
        if (i == 999) return false;
    }

    _file = SD.open(_name, FILE_WRITE);
    _open = (bool)_file;
    return _open;
}

void DataLog::write(const char* line)
{
    if (!_open || !line) return;
    _file.print(line);
    _file.flush();
}

void DataLog::close()
{
    if (_open) {
        _file.flush();
        _file.close();
    }
    _open = false;
}
