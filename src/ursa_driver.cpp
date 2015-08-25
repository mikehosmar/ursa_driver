/** Implementation of the ursa::Interface class.
 \file      ursa_driver.cpp
 \authors   Mike Hosmar <mikehosmar@gmail.com>
 \copyright Copyright (c) 2015, Michael Hosmar, All rights reserved.

 The MIT License (MIT)

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

#include <ursa_driver/ursa_driver.h>

namespace ursa
{
  const size_t max_line_length(64);

  //! All private variables are initialized to zero or there initial values. The pulses_ array is filled with zeros.
  Interface::Interface(const char *port, int baud) :
      port_(port), baud_(baud), connected_(false), serial_(NULL), acquiring_(
          false), responsive_(false), gmMode_(false), battV_(0), ramp_(6), voltage_(
          0) {
    pulses_.fill(0);
  }

  /** Stops acquire mode and immediately disables voltage if still enabled.
   * @todo In practice this doesn't work. The serial port is probably destroyed first.
   */
  Interface::~Interface() {
    tx_buffer_ << "R" << "v";
    transmit();
  }
  /**
   * This function first creates a new serial object. Then sets the timeout, port and baud rate.
   * The timeout is a global attribute for the serial port. Changing it will change the behavior of waitReadable
   * and therefore the function of many of the Interfaces functions.
   *
   * The function then tries 5 times to open the port and if successful it sends a stop acquire command down the line
   * then checks to see that the port is still open. If this is successful Interface::connected_ is set to true
   *
   * It then tries calling checkComms 5 times. If this is successful Interface::responsive_ is set to true.
   *
   * If either fail an error is writen to cout.
   */
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

  /**
   * This function writes the whole output buffer to the serial port as a string.
   * It will write an error to cout if a serial timeout occurs.
   * With DEBUG_ enabled it will write the buffer to std::cout.
   */
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

  /**
   * This uses a while loop to read a buffer of length max_line_length from the serial buffer.
   * This is copied into the rx_buffer buffer.
   *
   * If DEBUG_ enable prints out the length of the rx_buffer after filling it.
   *
   * Commented out line will print the input buffer straight to cout.
   */
  void Interface::read() {
    while (serial_->available())
    {
      uint8_t temp[max_line_length];
      long length = serial_->read(temp, max_line_length);
      rx_buffer_.insert(rx_buffer_.end(), temp, temp + length);
      //std::cout<< "RX: " << reinterpret_cast<const char*>(temp) << std::endl;
    }
#ifdef DEBUG_
    std::cout << "DEBUG: Receive buffer size: " << rx_buffer_.size()
    << std::endl;
#endif
    processData();
  }

  /**
   * This function processes incoming data in acquire mode.
   * The spectra data comes in as 3 bytes starting with 0xFF then a 4 bit count and then 12 bits of energy.
   * The 12 bit energy is used as an index for the Interface::pulses_ array which is incremented by the 4 bit count.
   *
   * If the top 6 bits of the second byte is 0 the data is 10bit battery data and can be treated as such.
   * This function loops until there is less than 3 bytes in the receive buffer.
   *
   * If the first byte is not 0xFF then bytes are dropped until there is a 0xFF on the front of the buffer.
   *
   * If DEBUG_ is enabled then each increment of the pulses_ array is reported to std::cout.
   */
  void Interface::processData() {
    while (rx_buffer_.size() >= 3)
    {
      if (rx_buffer_.front() == 0xff)
      {
        rx_buffer_.pop_front();      //drop the first byte it is only for sync
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
          pulses_[energy] += (count >> 2);
        }
      }
      else
      {
        std::cout << "ERROR: Read error, dropping chars:"
            << (int) rx_buffer_.front();
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

  /**
   * The function uses a boost::lock_gaurd before copying to protect against multiple access errors.
   * This should help in the future if multithreading is implemented.
   */
  void Interface::getSpectra(boost::array<unsigned int, 4096>* array) {
    boost::lock_guard<boost::mutex> lock(array_mutex_);
    *array = pulses_;
  }

  /**
   * This function sets all elements in the pulses_ array to zero using a boost:lock_gaurd to prevent multiple access errors.
   */
  void Interface::clearSpectra() {
    boost::lock_guard<boost::mutex> lock(array_mutex_);
    pulses_.fill(0);
  }

  /** Called from processData().  The reading is multiplied by 12/1024 to get volts.
   *
   * @param input The 10 bit battery voltage data
   */
  void Interface::processBatt(uint16_t input) {
    battV_ = (float) input * 12 / 1024;

#ifdef DEBUG_
    std::cout << "DEBUG: Battery voltage processed: "
    << boost::lexical_cast<std::string>(battV_) << std::endl;
#endif
  }

  /**
   * This function requires that the serial port be already opened.
   * It sends a "U" and the correct response from the Ursa is URSA2.
   * If this is what is received the function responds true otherwise it returns false.
   */
  bool Interface::checkComms() {
    stopAcquire();
    serial_->flush();
    tx_buffer_ << "U";
    transmit();
    std::string msg = serial_->read(max_line_length);
    boost::trim(msg);
    if (msg == "URSA2")
      return (true);
    else
      return (false);
  }

  void Interface::stopAcquire() {
    do
    {
      std::string ignored = serial_->read(128);
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
    else
      std::cout << "WARNING: Already acquiring" << std::endl;
  }

  void Interface::startGM() {
    if (!acquiring_)
    {
      tx_buffer_ << "J";
      transmit();
      gmMode_ = true;
      startAcquire();
    }
    else
      std::cout << "WARNING: Already acquiring" << std::endl;
  }

  void Interface::stopGM() {
    if (acquiring_)
      stopAcquire();
    tx_buffer_ << "j";
    transmit();
    gmMode_ = false;
  }

  /**
   * This function will only work in GM mode and while acquiring. The number of counts counted since the last time
   * this function was called is returned via serial as 4 bytes.  This function then combines the 4 bytes into one 32 but number and returns the number.
   *
   * If the number of bytes in the serial buffer is not 4 an error is written to cout.
   *
   */
  uint32_t Interface::requestCounts() {
    if (gmMode_ && acquiring_)
    {
      uint8_t temp_buffer[10];
      uint8_t count = 0;
      tx_buffer_ << "c";
      transmit();
      serial_->waitReadable();
      if (serial_->available() <= 4)
      {
        count = serial_->read(temp_buffer, 4);
        if (count == 4)
        {
          return (((uint32_t) temp_buffer[0] << 24)
              | ((uint32_t) temp_buffer[1] << 16)
              | ((uint32_t) temp_buffer[2] << 8) | ((uint32_t) temp_buffer[3]));
        }
      }
      std::cout << "ERROR: Did not receive correct number of bytes"
          << std::endl;
      return (0);
    }
    else
    {
      std::cout << "ERROR: Either not acquiring or not in GM mode."
          << std::endl;
      return (0);
    }
  }

  void Interface::stopVoltage() {
    tx_buffer_ << "v";
    transmit();
  }

  void Interface::requestBatt() {
    tx_buffer_ << "B";
    transmit();
    if (!acquiring_ || gmMode_)
    {
      uint8_t gm = (gmMode_ ? 1 : 0);
      serial_->waitReadable();
      if (serial_->available() <= (2 + gm))
      {
        uint8_t temp_buffer[3];
        uint8_t count = serial_->read(temp_buffer, 2 + gm);
        if (count == (2 + gm))
        {
          processBatt(
              ((uint16_t) temp_buffer[0 + gm] << 8)
                  | ((uint16_t) temp_buffer[1 + gm]));
          return;
        }
      }
      else
      {
        std::cout << "ERROR: Failed to process Batt. voltage." << std::endl;
      }
    }
  }

  float Interface::getBatt() {
    return (battV_);
  }

  void Interface::startASCII() {
    if (!acquiring_)
    {
      tx_buffer_ << "A";
      transmit();
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to switch ASCII mode."
          << std::endl;
  }

  void Interface::stopASCII() {
    if (!acquiring_)
    {
      tx_buffer_ << "N";
      transmit();
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to switch ASCII mode."
          << std::endl;
  }

  int Interface::requestSerialNumber() {
    if (!acquiring_)
    {
      tx_buffer_ << "@";
      transmit();
      usleep(50000);
      std::string msg = serial_->read(max_line_length);
      boost::trim(msg);
      std::cout << "INFO: The serial number is: " << msg << std::endl;
      return (boost::lexical_cast<int>(msg.c_str()));
    }
    else
    {
      std::cout << "ERROR: Acquiring. Stop acquiring to switch ASCII mode."
          << std::endl;
      return (-1);
    }
  }

  /**
   * This function is mainly used by the original software to
   * determine the ratio for setting the high voltage.
   * The max HV is almost always 2000.
   */
  void Interface::requestMaxHV() {
    if (!acquiring_)
    {
      tx_buffer_ << "2";
      transmit();
      usleep(50000);
      std::string msg = serial_->read(max_line_length);
      boost::trim(msg);
      std::cout << "INFO: The max HV is: " << msg << std::endl;
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to request Max HV."
          << std::endl;
  }

#ifdef ADMIN_
  void Interface::setSerialNumber(int serial)
  {
    if (!acquiring_ && serial >= 200000 && serial <= 299999)
    {
      tx_buffer_ << "#";
      transmit();
      tx_buffer_ << boost::lexical_cast<std::string>(serial);
      transmit();
      sleep(3);
    }
    else
    std::cout
    << "ERROR: Serial must be between 200000 and 299999 and the system must not be acquiring."
    << std::endl;
  }

  void Interface::setSmudgeFactor(int smudge)
  {
    if (!acquiring_ && smudge >= 0 && smudge <= 4)
    {
      tx_buffer_ << "X" << boost::lexical_cast<std::string>(smudge);
      transmit();
    }
    else
    std::cout
    << "ERROR: Smudge factor must be between 0 and 4 and the system must not be acquiring."
    << std::endl;
  }

  //This function must NOT be used
//  void Interface::setMaxHV(int HV) {
//    if (!acquiring_ && HV >= 0 && HV <= 10000)
//    {
//      tx_buffer_ << "???";  //character not actually known.
//      transmit();
//      tx_buffer_ << char(HV >> 8) << char(HV & 0xFF);
//      transmit();
//      sleep(3);
//    }
//    else
//      std::cout << "ERROR: Acquiring. Stop acquiring to change max HV."
//          << std::endl;
//  }
#endif

  void Interface::loadPrevSettings() {
    if (!acquiring_)
    {
      tx_buffer_ << "r";
      transmit();
      int seconds = 1;
      //This sets HV so we need to wait for ramp
      std::string msg = serial_->read(max_line_length);
      while (!serial_->waitReadable())
      {
        tx_buffer_ << "B";
        transmit();
        std::cout << "INFO: Ramping HV.  Approx. seconds elapsed: " << seconds
            << std::endl;
        seconds++;
      }
      msg = serial_->read(max_line_length);
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to load settings."
          << std::endl;
  }

  void Interface::setNoSave() {
    if (!acquiring_)
    {
      tx_buffer_ << "d";
      transmit();
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to disable EEPROM saving."
          << std::endl;
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
      //calculate seconds for ramp then adjust for the loop taking 1.1 seconds
      int seconds = ((ramp_ * abs(voltage - voltage_) / 100) / 1.1) - 1;
      // blocking call to serial to wait for responsiveness
      std::string msg = serial_->read(max_line_length);
      while (!serial_->waitReadable())
      {
        std::cout << "INFO: Ramping HV to: " << voltage
            << " Approx. Seconds Remaining: " << seconds << std::endl;
        seconds--;
        tx_buffer_ << "B";
        transmit();
      }
      msg = serial_->read(max_line_length);
      voltage_ = voltage;
    }
    else
      std::cout
          << "ERROR: Voltage must be between 0 and 2000 volts and the system must not be acquiring"
          << std::endl;
  }

  void Interface::setGain(double gain) {
    if (!acquiring_)
    {
      char coarse;
      uint8_t fine;
      std::string coarse_str;
      if (gain < 2)
      {
        coarse = '0';
        fine = round((gain / 2) * 256 - 1);
        coarse_str = "2";
      }
      else if (gain < 4)
      {
        coarse = '1';
        fine = round((gain / 4) * 256 - 1);
        coarse_str = "4";
      }
      else if (gain < 15)
      {
        coarse = '2';
        fine = round((gain / 15) * 256 - 1);
        coarse_str = "15";
      }
      else if (gain < 35)
      {
        coarse = '3';
        fine = round((gain / 35) * 256 - 1);
        coarse_str = "35";
      }
      else if (gain < 125)
      {
        coarse = '4';
        fine = round((gain / 125) * 256 - 1);
        coarse_str = "125";
      }
      else if (gain < 250)
      {
        coarse = '5';
        fine = round((gain / 250) * 256 - 1);
        coarse_str = "250";
      }
      else
      {
        std::cout << "ERROR: Gain must be bellow 250x" << std::endl;
        return;
      }

      std::cout << "INFO: Setting coarse gain to: " << coarse_str << std::endl;

      double confirmGain = ((double(fine) + 1) / 256);
      std::cout << "INFO: Setting fine gain to: "
          << boost::lexical_cast<std::string>(confirmGain) << std::endl;
      tx_buffer_ << "C" << coarse << "F" << fine;
      transmit();
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to change gain."
          << std::endl;
  }

  void Interface::setInput(inputs input) {
    if (!acquiring_)
    {
      setVoltage(0);
      tx_buffer_ << "I" << boost::lexical_cast<std::string>(input);
      transmit();
    }
    else
      std::cout
          << "ERROR: Acquiring. Stop acquiring to switch inputs or polarity."
          << std::endl;
  }

  void Interface::setShapingTime(shaping_time time) {
    if (!acquiring_)
    {
      tx_buffer_ << "S" << boost::lexical_cast<std::string>(time);
      transmit();
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to change shaping time."
          << std::endl;
  }

  void Interface::setThresholdOffset(int mVolts) {
    if (!acquiring_ && mVolts >= 25 && mVolts <= 1023)
    {
      uint16_t min_offset = 50;
      uint16_t thresh = mVolts * 2;
      uint16_t offset = 0;
      if (mVolts > min_offset * 2)
      {
        offset = mVolts;  //offset = mV/2 *2
      }
      else
        offset = min_offset * 2;

      tx_buffer_ << "T" << (unsigned char) ((thresh >> 4) & 0xFF)
          << (unsigned char) (((thresh & 0x0F) << 4) | ((offset >> 8) & 0x0F))
          << (unsigned char) (offset & 0xFF);
      transmit();
    }
    else
      std::cout
          << "ERROR: Threshold must be between 25 and 1024 mV and the system must not be acquiring"
          << std::endl;
  }

  void Interface::setBitMode(int bits) {
    if (!acquiring_ && bits >= 8 && bits <= 12)
    {
      tx_buffer_ << "M" << boost::lexical_cast<std::string>(13 - bits);
      transmit();
    }
    else
      std::cout
          << "ERROR: Bits must be between 12 and 8 bits and the system must not be acquiring"
          << std::endl;
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
      std::cout
          << "ERROR: Ramp must be between 6 and 219 seconds and the system must not be acquiring"
          << std::endl;
  }

  void Interface::noRamp() {
    if (!acquiring_)
    {
      tx_buffer_ << "p";
      transmit();
    }
    else
      std::cout << "ERROR: Acquiring. Stop acquiring to disable ramping of HV."
          << std::endl;
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
