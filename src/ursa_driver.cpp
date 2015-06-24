/**
 The MIT License (MIT)

 \file      ursa_driver.cpp
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

#include <boost/algorithm/string/trim.hpp>
#include <boost/thread/lock_guard.hpp>
#include <serial/serial.h>
#include <ursa_driver/ursa_driver.h>
#include <iostream>
#include <sstream>

namespace ursa
{
  const std::string eol("\n");
  const size_t max_line_length(128);

  Interface::Interface(const char *port, int baud) :
      port_(port), baud_(baud), connected_(false), serial_(NULL), acquiring_(
          false), responsive_(false), gmMode_(false), battV(0) {
    pulses_.fill(0);
  }

  Interface::~Interface() {
  }

  void Interface::connect() {
    if (!serial_)
      serial_ = new serial::Serial();
    serial::Timeout timeout(serial::Timeout::simpleTimeout(1000));
    serial_->setTimeout(timeout);
    serial_->setPort(port_);
    serial_->setBaudrate(baud_);

    for (int i = 0; i < 5; i++)
    {
      if (!connected_)
      {
        try
        {
          serial_->open();
          stopAcquire();
        }
        catch (serial::IOException & err)
        {
        }

        if (serial_->isOpen())
        {

          connected_ = true;
          break;
        }
        else
        {
          connected_ = false;
          std::cout << "Unable to connect to serial port:" << port_
              << std::endl;
        }
      }
    }
    for (int j = 0; j < 5; j++)
    {
      if (checkComms())
      {
        responsive_ = true;
        return;
      }
      else
      {
        responsive_ = false;
        std::cout << "URSA not responding." << std::endl;
      }
    }
    std::cout << "Unable to communicate with URSA" << std::endl;
  }

  void Interface::transmit() {
#ifdef DEBUG_
    std::cout << "Transmitting:" << tx_buffer_.str() << std::endl;
#endif
    tx_buffer_ << eol;
    ssize_t bytes_written = serial_->write(tx_buffer_.str());
    if (bytes_written < tx_buffer_.tellp())
    {
      std::cout << "Serial write timeout, " << bytes_written
          << " bytes written of " << tx_buffer_.tellp() << "." << std::endl;
    }
    tx_buffer_.str("");
  }

  void Interface::read() {
    while (serial_->available())
    {
      uint8_t temp[max_line_length];
      long length = serial_->read(temp, max_line_length);
      rx_buffer_.insert(rx_buffer_.end(), temp, temp + length);
      //std::cout<< "RX: " << reinterpret_cast<const char*>(temp) << std::endl;
    }
#ifdef DEBUG_
    std::cout << "Buffer Size: " << rx_buffer_.size() << std::endl;
#endif
    processData();
  }
  void Interface::processData() {
    while (rx_buffer_.size() >= 3)
    {
      if (rx_buffer_.front() == 0xff)
      {
        rx_buffer_.pop_front();        //drop the first byte it is only for sync
        uint8_t char1, char2, count;
        uint16_t energy;

        char1 = rx_buffer_.front();
        rx_buffer_.pop_front();
        char2 = rx_buffer_.front();
        rx_buffer_.pop_front();

        count = char1 >> 2;
        energy = (char1 & 0x03) << 8 | char2;

        if (count == 0)
          processBatt(energy);
        else
        {
          boost::lock_guard<boost::mutex> lock(array_mutex_);
          pulses_[energy] += count >> 2; //I think the docs were wrong and this is the correct method
        }       //this will increment by the highest 4 bits of the original 16
      }
      else      //the stream should start with a 0xFF for each read
      {
        std::cout << "Read error, dropping chars:" << (int) rx_buffer_.front();
        rx_buffer_.pop_front();
        while (rx_buffer_.front() != 0xff && rx_buffer_.size() > 0)
        {
          std::cout << ", " << (int) rx_buffer_.front();
          rx_buffer_.pop_front();
        }
        std::cout << std::endl;
      }
    }
  }

  void Interface::getPulses(boost::array<unsigned int, 4096>* array) {
    boost::lock_guard<boost::mutex> lock(array_mutex_);
    *array = pulses_;
  }

  void Interface::processBatt(uint16_t input) {
    battV = input * 12 / 1024;
  }

  bool Interface::checkComms() {
    stopAcquire();
    serial_->flush();
    tx_buffer_ << "U";
    transmit();
    serial_->waitReadable();
    std::string msg = serial_->readline(max_line_length, eol);
    boost::trim(msg);
    if (msg == "URSA2")
      return (true);
    else
      return (false);
  }

  void Interface::stopAcquire() {
    do
    {
      std::string ignored = serial_->read(max_line_length);
      tx_buffer_ << "R";
      transmit();
      usleep(500);
    }
    while (serial_->available());
    acquiring_ = false;
  }

  void Interface::startAcquire() {
    if (!acquiring_)
    {
      tx_buffer_ << "G";
      transmit();
      acquiring_ = true;
    }
  }

  void Interface::startGM() {
    tx_buffer_ << "J";
    transmit();
    gmMode_ = true;
  }

  void Interface::stopGM() {
    tx_buffer_ << "j";
    transmit();
    gmMode_ = false;
  }

  void Interface::requestCounts() {
    if (gmMode_)
    {
      tx_buffer_ << "c";
      transmit();
    }
  }

  void Interface::stopVoltage() {
    tx_buffer_ << "v";
    transmit();
  }

  void Interface::requestBatt() {
    tx_buffer_ << "B";
    transmit();
  }

  void Interface::startASCII() {
    if (!acquiring_)
    {
      tx_buffer_ << "A";
      transmit();
    }
  }

  void Interface::stopASCII() {
    if (!acquiring_)
    {
      tx_buffer_ << "N";
      transmit();
    }
  }

  void Interface::requestSerialNumber() {
    if (!acquiring_)
    {
      tx_buffer_ << "@";
      transmit();
    }
  }

  void Interface::setSerialNumber(int serial) {
    if (!acquiring_ && serial >= 20000 && serial <= 29999)
    {
      tx_buffer_ << "#" << boost::lexical_cast<std::string>(serial);
      transmit();
    }
    else
      std::cout << "ERROR: Serial must be between 20000 and 29999" << std::endl;
  }

  void Interface::setSmudgeFactor(int smudge) {
    if (!acquiring_ && smudge >= 0 && smudge)
    {
      tx_buffer_ << "X" << boost::lexical_cast<std::string>(smudge);
      transmit();
    }
  }

  void Interface::loadPrevSettings() {
    if (!acquiring_)
    {
      tx_buffer_ << "r";
      transmit();
    }
  }

  void Interface::setNoSave() {
    if (!acquiring_)
    {
      tx_buffer_ << "d";
      transmit();
    }
  }

  void Interface::setVoltage(int voltage) {
    if (!acquiring_ && voltage >= 0 && voltage <= 2000)
    {
      tx_buffer_ << "V";
      transmit();
    }
  }

  void Interface::setGain(int gain) {
    if (!acquiring_)
    {
      tx_buffer_ << "C" << "F";
      transmit();
    }
  }

  void Interface::setInput(inputs input) {
    if (!acquiring_)
    {
      tx_buffer_ << "I" << boost::lexical_cast<std::string>(input);
      transmit();
    }
  }

  void Interface::setShapingTime(shaping_time time) {
    if (!acquiring_)
    {
      tx_buffer_ << "S" << boost::lexical_cast<std::string>(time);
      transmit();
    }
  }

  void Interface::setThresholdOffset(int mVolts) {
    if (!acquiring_)
    {

    }
  }

  void Interface::setBitMode(int bits) {
    if (!acquiring_ && bits >= 8 && bits <= 12)
    {
      tx_buffer_ << "S" << boost::lexical_cast<std::string>(13 - bits);
      transmit();
    }
    else
      std::cout << "ERROR: bits must be between 12 and 8 bits" << std::endl;
  }

  void Interface::setRamp(int seconds) {
    if (!acquiring_ && seconds >= 6 && seconds <= 219)
    {
      int ramp = round((seconds * 303.45) - 1197);
      if (ramp > 16383)
        ramp = 16838;
      tx_buffer_ << "P" << boost::lexical_cast<std::string>(ramp);
      transmit();
    }
    else
      std::cout << "ERROR: Ramp must be between 6 and 219 seconds" << std::endl;
  }

  void Interface::noRamp() {
    if (!acquiring_)
    {
      tx_buffer_ << "p";
      transmit();
    }
  }

  void Interface::setAlarm0(bool enable) {
    tx_buffer_ << (enable ? "Z" : "z");
    transmit();
  }

  void Interface::setAlarm1(bool enable) {
    tx_buffer_ << (enable ? "W" : "w");
    transmit();
  }
}
