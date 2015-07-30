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
    tx_buffer_ << "R" << "v";
    transmit();
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
          std::cout << "WARN: Unable to connect to serial port:" << port_
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
        std::cout << "WARN: URSA not responding." << std::endl;
      }
    }
    std::cout << "ERROR: Unable to communicate with URSA" << std::endl;
  }

  void Interface::transmit() {
#ifdef DEBUG_
    std::cout << "DEBUG: Transmitting:" << tx_buffer_.str() << std::endl;
#endif
    ssize_t bytes_written = serial_->write(tx_buffer_.str());
    if (bytes_written < tx_buffer_.tellp())
    {
      std::cout << "ERROR: Serial write timeout, " << bytes_written
          << " bytes written of " << tx_buffer_.tellp() << "." << std::endl;
    }
    tx_buffer_.str("");
    usleep(100000);  //for stability
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
    std::cout << "DEBUG: Receive buffer size: " << rx_buffer_.size() << std::endl;
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
#ifdef DEBUG_
          std::cout << "DEBUG: Incrementing Bin: "
              << boost::lexical_cast<std::string>(energy) << " By amount: "
              << boost::lexical_cast<std::string>(count >> 2) << std::endl;
#endif
          pulses_[energy] += (count >> 2); //I think the docs were wrong and this is the correct method
        }       //this will increment by the highest 4 bits of the original 16
      }
      else      //the stream should start with a 0xFF for each read
      {
        std::cout << "ERROR: Read error, dropping chars:" << (int) rx_buffer_.front();
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
    if (!acquiring_)
    {
      tx_buffer_ << "J";
      transmit();
      gmMode_ = true;
      startAcquire();
    }
  }

  void Interface::stopGM() {
    if (acquiring_)
      stopAcquire();
    tx_buffer_ << "j";
    transmit();
    gmMode_ = false;
  }

  void Interface::requestCounts() {
    if (gmMode_ && acquiring_)
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

  int Interface::requestSerialNumber() {
    if (!acquiring_)
    {
      tx_buffer_ << "@";
      transmit();
      usleep(50000);
      std::string msg = serial_->readline(max_line_length, eol);
      boost::trim(msg);
      std::cout << "INFO: The serial number is: " << msg << std::endl;
      return (boost::lexical_cast<int>(msg.c_str()));
    }
    else
      return (-1);
  }

  void Interface::setSerialNumber(int serial) {
    if (!acquiring_ && serial >= 200000 && serial <= 299999)
    {
      tx_buffer_ << "#";
      transmit();
      tx_buffer_ << boost::lexical_cast<std::string>(serial);
      transmit();
      sleep(3);
    }
    else
      std::cout << "ERROR: Serial must be between 200000 and 299999" << std::endl;
  }

  void Interface::setSmudgeFactor(int smudge) {
    if (!acquiring_ && smudge >= 0 && smudge <= 4)
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
      //This sets HV so we need to wait for ramp
      sleep(5);
      while (!serial_->waitReadable())
      {
        tx_buffer_ << "B";
        transmit();
      }
      std::string msg = serial_->readline(max_line_length, eol);
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
      if (voltage == 0)
        setNoSave();
      uint16_t outVolts = 0;
      outVolts = round(double(voltage) / 2000 * 65532);
      tx_buffer_ << "V" << (uint8_t) (outVolts >> 8)
          << (uint8_t) (outVolts & 0xff);
      transmit();
      // blocking call to serial to wait for responsiveness
      sleep(5);
      while (!serial_->waitReadable())
      {
        tx_buffer_ << "B";
        transmit();
      }
      std::string msg = serial_->readline(max_line_length, eol);
    }
    else
      std::cout << "ERROR: Voltage must be between 0 and 2000 volts"
          << std::endl;
  }

  void Interface::setGain(double gain) {
    if (!acquiring_)
    {
      char coarse;

      uint8_t fine;
      if (gain < 2)
      {
        coarse = '0';
        fine = round((gain / 2) * 256 - 1);
      }
      else if (gain < 4)
      {
        coarse = '1';
        fine = round((gain / 4) * 256 - 1);
      }
      else if (gain < 15)
      {
        coarse = '2';
        fine = round((gain / 15) * 256 - 1);
      }
      else if (gain < 35)
      {
        coarse = '3';
        fine = round((gain / 35) * 256 - 1);
      }
      else if (gain < 125)
      {
        coarse = '4';
        fine = round((gain / 125) * 256 - 1);
      }
      else if (gain < 250)
      {
        coarse = '5';
        fine = round((gain / 250) * 256 - 1);
      }
      else
      {
        std::cout << "ERROR: Gain must be bellow 250x" << std::endl;
        return;
      }

      double confirmGain = ((double(fine) + 1) / 256);
      std::cout << "INFO: Setting fine gain to: "
          << boost::lexical_cast<std::string>(confirmGain) << std::endl;
      tx_buffer_ << "C" << coarse << "F" << fine;
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
      tx_buffer_ << "M" << boost::lexical_cast<std::string>(13 - bits);
      transmit();
    }
    else
      std::cout << "ERROR: bits must be between 12 and 8 bits" << std::endl;
  }

  void Interface::setRamp(int seconds) {
    if (!acquiring_ && seconds >= 6 && seconds <= 219)
    {
      ramp_ = seconds;
      uint16_t ramp = round((seconds * 303.45) - 1197);
      if (ramp > 16383)
        ramp = 16838;
      tx_buffer_ << "P" << (uint8_t) (ramp >> 8) << (uint8_t) (ramp & 0xFF);
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
