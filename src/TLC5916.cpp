#include "TLC5916.hpp"

#include <Arduino.h>

// bitbanging
TLC5916::TLC5916(uint8_t sdiPin, uint8_t sckPin, uint8_t lePin, uint8_t oePin) {
    _sdiPin = sdiPin;
    _sckPin = sckPin;
    _lePin = lePin;
    _oePin = oePin;
    _spiWire = nullptr;

    pinMode(_sdiPin, OUTPUT);
    digitalWrite(_sckPin, LOW);
    pinMode(_sckPin, OUTPUT);

    _init();
}

// hw spi
TLC5916::TLC5916(SPIClass* spiWire, uint8_t sckPin, uint8_t lePin, uint8_t oePin) {
    _sdiPin = NO_PIN;
    _sckPin = sckPin;
    _lePin = lePin;
    _oePin = oePin;
    _spiWire = spiWire;

    _init();
}

void setBit(unsigned int bitIdx, bool value) {
    uint8_t& byte = _data[sizeof(_data) - (bitIdx / 8) - 1];
    uint8_t bitMask = 1 << (bitIdx % 8);
    if (value) {
        byte |= bitMask;
    } else {
        byte &= bitMask;
    }
}

void sendData() { _writeData(); }

void enableSink() {
    if (_oePin == NO_PIN)
        return;
    digitalWrite(_oePin, LOW);
}

void disableSink() {
    if (_oePin == NO_PIN)
        return;
    digitalWrite(_oePin, HIGH);
}

void setConfig(uint8_t tlcIdx, bool currentMultiplier, bool currentLimiter, uint8_t currentValue) {
    setConfig(tlcIdx, (currentMultiplier ? 0b10000000 : 0) | (currentLimiter ? 0b01000000 : 0) |
                          (currentValue & 0b00111111));
}

void setConfig(uint8_t tlcIdx, uint8_t config) {
    uint8_t& byte = _config[sizeof(_config) - tlcIdx - 1];
    byte = 0;
    for (unsigned int i = 0; i < 8; ++i) {
        if (config & (1 << (8 - i - 1)))
            byte |= 1 << i;
    }
}

void applyConfigs() {
    if (_oePin == NO_PIN)
        return;

    disableSink();
    _modeSwitch(true);
    _writeData();
    _modeSwitch(false);
    enableSink();
}

void TLC5916::_init() {
    memset(_data, sizeof(_data), 0);
    memset(_config, sizeof(_data), 0xff);
    if (_oePin != NO_PIN) {
        digitalWrite(_oePin, HIGH); // disable sink
        pinMode(_oePin, OUTPUT);
    }
    digitalWrite(_lePin, LOW);
    pinMode(_lePin, OUTPUT);
}

void TLC5916::_writeData() {
    if (_spiWire) {
        // max is 30MHz
        _spiWire->beginTransaction(SPISettings(30000000, MSBFIRST, SPI_MODE0));
        _spiWire->transfer(_data, sizeof(_data));
        _spiWire->endTransaction();
    } else {
        for (unsigned int i = 0; i < sizeof(_data); ++i) {
            auto const byte = _data[i];
            for (unsigned int j = 0; j < 8; ++j) {
                // data is already stored reversed -> write MSB first
                uint8_t const bitMask = 1 << (8 - j - 1);
                digitalWrite(_sdiPin, byte & bitMask);
                _sckFlash();
            }
        }
    }
    _leFlash();
}

void TLC5916::_leFlash() {
    digitalWrite(_lePin, HIGH);
    digitalWrite(_lePin, LOW);
}

void TLC5916::_sckFlash() {
    digitalWrite(_sckPin, HIGH);
    digitalWrite(_sckPin, LOW);
}

void TLC5916::_modeSwitch(bool special) {
    if (_eoPin == NO_PIN)
        return;

    _sckFlash();
    digitalWrite(_oePin, LOW);
    _sckFlash();
    digitalWrite(_oePin, HIGH);
    _sckFlash();
    if (special)
        digitalWrite(_lePin, HIGH);
    _sckFlash();
    if (special)
        digitalWrite(_lePin, LOW);
    _sckFlash();
}
