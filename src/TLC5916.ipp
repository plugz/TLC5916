#include "TLC5916.hpp"

#include <Arduino.h>

template<uint8_t TCount>
TLC5916<TCount>::TLC5916(uint8_t sdiPin, uint8_t sckPin, uint8_t lePin, uint8_t oePin) {
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

template<uint8_t TCount>
void TLC5916<TCount>::setBit(unsigned int bitIdx, bool value) {
    uint8_t& byte = _data[count - (bitIdx / 8) - 1];
    uint8_t bitMask = 1 << (bitIdx % 8);
    if (value) {
        byte |= bitMask;
    } else {
        byte &= ~bitMask;
    }
}

template<uint8_t TCount>
void TLC5916<TCount>::sendData() { _writeData(_data); }

template<uint8_t TCount>
void TLC5916<TCount>::enableSink() {
    if (_oePin == NO_PIN)
        return;
    digitalWrite(_oePin, LOW);
}

template<uint8_t TCount>
void TLC5916<TCount>::disableSink() {
    if (_oePin == NO_PIN)
        return;
    digitalWrite(_oePin, HIGH);
}

template<uint8_t TCount>
void TLC5916<TCount>::setConfig(uint8_t tlcIdx, bool currentMultiplier, bool currentLimiter, uint8_t currentValue) {
    setConfig(tlcIdx, (currentMultiplier ? 0b10000000 : 0) | (currentLimiter ? 0b01000000 : 0) |
                          (currentValue & 0b00111111));
}

template<uint8_t TCount>
void TLC5916<TCount>::setConfig(uint8_t tlcIdx, uint8_t config) {
    uint8_t& byte = _config[count - tlcIdx - 1];
    byte = 0;
    for (unsigned int i = 0; i < 8; ++i) {
        if (config & (1 << (8 - i - 1)))
            byte |= 1 << i;
    }
}

template<uint8_t TCount>
void TLC5916<TCount>::applyConfigs() {
    if (_oePin == NO_PIN)
        return;

    disableSink();
    _modeSwitch(true);
    _writeData(_config);
    _modeSwitch(false);
    enableSink();
}

template<uint8_t TCount>
void TLC5916<TCount>::setSpiWire(SPIClass* spiWire, uint32_t spiSpeed) {
    _spiWire = spiWire;
    _spiSpeed = spiSpeed;
}

template<uint8_t TCount>
void TLC5916<TCount>::_init() {
    memset(_data, 0, count);
    memset(_config, 0xff, count);
    if (_oePin != NO_PIN) {
        digitalWrite(_oePin, HIGH); // disable sink
        pinMode(_oePin, OUTPUT);
    }
    digitalWrite(_lePin, LOW);
    pinMode(_lePin, OUTPUT);
}

template<uint8_t TCount>
void TLC5916<TCount>::_writeData(uint8_t* data) {
    static uint8_t recvBuffer[count];
    if (_spiWire) {
        _spiWire->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
        _spiWire->transfer(data, recvBuffer, count);
        _spiWire->endTransaction();
    } else {
        // bitbanging
        for (unsigned int i = 0; i < count; ++i) {
            auto const byte = data[i];
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

template<uint8_t TCount>
void TLC5916<TCount>::_leFlash() {
    digitalWrite(_lePin, HIGH);
    digitalWrite(_lePin, LOW);
}

template<uint8_t TCount>
void TLC5916<TCount>::_sckFlash() {
    digitalWrite(_sckPin, HIGH);
    digitalWrite(_sckPin, LOW);
}

template<uint8_t TCount>
void TLC5916<TCount>::_modeSwitch(bool special) {
    if (_oePin == NO_PIN)
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
