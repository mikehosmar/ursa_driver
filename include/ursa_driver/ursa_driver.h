/**
 The MIT License (MIT)

 \file      ursa_driver.h
 \authors   Mike Hosmar <mikehosmar@gmail.com>
 \copyright Copyright (c) 2015, Michael Hosmar, All rights reserved.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#ifndef URSA_DRIVER_H_
#define URSA_DRIVER_H_
#undef DEBUG_
#define ADMIN_

#include <boost/lexical_cast.hpp>
#include <boost/array.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/thread/lock_guard.hpp>

#include <stdint.h>
#include <queue>
#include <iostream>
#include <sstream>

#include <serial/serial.h>

namespace serial
{
  class Serial;
}

namespace ursa
{

  class Interface
  {

  private:
    const char *port_;
    int baud_;
    bool connected_;
    bool responsive_;
    bool acquiring_;
    bool gmMode_;
    serial::Serial *serial_;
    std::stringstream tx_buffer_;
    std::deque<uint8_t> rx_buffer_;

    float battV;

    int ramp_;
    int voltage_;

    boost::mutex array_mutex_;
    boost::array<uint32_t, 4096> pulses_;

    bool checkComms();
    void transmit();
    void processData();
    void processBatt(uint16_t input);

  public:
    Interface(const char *port, int baud);
    ~Interface();

    void read();
    void getSpectra(boost::array<uint32_t, 4096>* array);
    void clearSpectra();

    void connect();
    bool connected() {
      if (connected_)
      {
        if (responsive_)
          return (true);
        else
          return (checkComms());
      }
      else
        return (false);
    }
    bool acquiring() {
      return (acquiring_);
    }

    void startAcquire();
    void stopAcquire();

    void startGM();
    void stopGM();
    uint32_t requestCounts(); //only in GM mode

    void stopVoltage();
    void requestBatt(); //can be active

    void startASCII();
    void stopASCII();
    int requestSerialNumber();
    void requestMaxHV();
#ifdef ADMIN_
    void setSerialNumber(int serial);  //Factory only
    void setSmudgeFactor(int smudge);  //Factory only
    void setMaxHV(int HV);
#endif
    enum inputs
    {
      INPUT1NEG = 0, INPUT1POS, INPUT2NEG, INPUT2POS, INPUTXPOS //INPUTXPOS either input positive
                                                                //pre-shaped pulse
    };

    enum shaping_time
    {
      TIME0_25uS = 0,
      TIME0_5uS,
      TIME1uS,
      TIME2uS,
      TIME4uS,
      TIME6uS,
      TIME8uS,
      TIME10uS
    };

    void loadPrevSettings();
    void setNoSave();  //dont save HV to EEPROM
    void setVoltage(int voltage);  //EEPROM
    void setGain(double gain); //EEPROM
    void setInput(inputs input); //EEPROM
    void setShapingTime(shaping_time time);  //EEPROM
    void setThresholdOffset(int mVolts);  //EEPROM

    void setBitMode(int bits);
    void setRamp(int seconds);
    void noRamp();

    void setAlarm0(bool enable);
    void setAlarm1(bool enable);
  };

}

#endif /* INCLUDE_URSA_DRIVER_URSA_H_ */
