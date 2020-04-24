#ifndef __TLC5916_HPP__
#define __TLC5916_HPP__

#include <SPI.h>

template <uint8_t TCount> class TLC5916 {
public:
    static constexpr uint8_t count = TCount;

    enum : uint8_t { NO_PIN = 0xff };

public:
    // bitbanging
    TLC5916(uint8_t sdiPin, uint8_t sckPin, uint8_t lePin, uint8_t oePin = NO_PIN);
    // hw spi
    TLC5916(SPIClass* spiWire, uint8_t sckPin, uint8_t lePin, uint8_t oePin = NO_PIN);

    void setBit(unsigned int bitIdx, bool value);
    void sendData();

    void enableSink();
    void disableSink();

    void setConfig(uint8_t tlcIdx, bool, bool, uint8_t config);
    void setConfig(uint8_t tlcIdx, uint8_t config);
    void applyConfigs();

private:
    void _init();
    void _writeData();
    void _leFlash();
    void _sckFlash();
    void _modeSwitch(bool special);

    uint8_t _sdiPin;
    uint8_t _sckPin;
    uint8_t _lePin;
    uint8_t _eoPin;
    SPIClass* _spiWire;

    uint8_t _data[count];
    uint8_t _config[count];
};

#endif
