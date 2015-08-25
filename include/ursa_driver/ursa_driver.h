/** The header file for the ursa::Interface class.
 \file      ursa_driver.h
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

#ifndef URSA_DRIVER_H_
#define URSA_DRIVER_H_
//#define DEBUG_
//#define ADMIN_

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

//! The ursa namespace.
namespace ursa
{
  /** \brief An enum for the different input configurations.
   *
   *  Used in Interface::setInput().
   *  ursa::INPUTXPOS is either input positive pre-shaped pulse. */
  enum inputs
  {
    INPUT1NEG = 0, /**< Input 1 negative polarity.*/
    INPUT1POS, /**< Input 1 positive polarity.*/
    INPUT2NEG, /**< Input 2 negative polarity.*/
    INPUT2POS, /**< Input 2 positive polarity.*/
    INPUTXPOS /**< Either input positive.*/
  };

  /** An Enum for each possible shaping time.
   *
   * Used in Interface::setShapingTime().
   */
  enum shaping_time
  {
    TIME0_25uS = 0, //!< 0.25 μS
    TIME0_5uS,     //!< 0.5 μS
    TIME1uS,       //!< 1 μS
    TIME2uS,       //!< 2 μS
    TIME4uS,       //!< 4 μS
    TIME6uS,       //!< 6 μS
    TIME8uS,       //!< 8 μS
    TIME10uS       //!< 10 μS
  };

  //! The interface class implements a link to the ursa hardware.
  class Interface
  {

  private:
    const char *port_;  //!< The Serial port that the Ursa is connected to.
    int baud_; //!< The baud rate that Ursa communicates at. This is always 115200.
    bool connected_; //!< A boolean signaling if the Interface successfully connected.
    bool responsive_; //!< A boolean signaling if the Ursa has responded to the Interface.
    bool acquiring_;    //!< A boolean which marks when the ursa is acquiring.
    bool gmMode_;       //!< A boolean which reports if the ursa is in GM mode.
    serial::Serial *serial_; //!< A serial object which controls comunication to the serial port.
    std::stringstream tx_buffer_;   //!< A String buffer for output commands.
    std::deque<uint8_t> rx_buffer_; //!< A Character buffer for incoming data.

    float battV_; //!< The current Battery voltage. This is NOT the 12v input voltage.

    int ramp_;      //!< The ramp time in seconds per 100 volts.
    int voltage_;   //!< The currently set high voltage.

    boost::mutex array_mutex_; //!< The locking mechanism for the spectrum array.
    boost::array<uint32_t, 4096> pulses_; //!< An array of the pulses received in each bin. This consists of 4096 32 bit unsigned integers.

    /**
     * \brief Private function which checks to see if Ursa will respond to communication.
     * @return True: communication verified. False: failed to receive correct response.
     */
    bool checkComms();

    void transmit(); //!< \brief Private utility function for flushing the transmit buffer down the line.
    void processData(); //!< \brief Private utility function which processes incoming data.
    void processBatt(uint16_t input); //!< \brief Private utility function which processes a battery voltage message if in acquire mode.

  public:
    /**
     * \brief Interface constructor.
     * @param port The port that the Ursa hardware is connected to.
     * @param baud Baud rate to communicate to the Ursa with.
     */
    Interface(const char *port, int baud);
    ~Interface(); //!< \brief Interface destructor.

    void read(); //!< \brief A utility function to flush the input buffer and process the data.
    /** \brief Access function which returns by reference a copy of the spectra data.
     * @param array The array to fill with spectra data.
     */
    void getSpectra(boost::array<uint32_t, 4096>* array);
    void clearSpectra(); //!< \brief A utility function to clear the internal Interface::pulses_ array.

    void connect(); //!< \brief Opens the serial port and attempts to confirm communication to the Ursa.
    /** \brief A utility function to check the status of the connection to the Ursa.
     * @return Returns true if Interface::connected_ and Interface::responsive_ are true.
     *
     * The function will attempt to check responsiveness if Interface::responsive_ is false using
     * the Interface::checkComms() function.
     */
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
    //! \brief A utility function to check if the driver is acquiring.
    bool acquiring() {
      return (acquiring_);
    }

    void startAcquire(); //!< \brief Instructs the Ursa to start acquiring.
    void stopAcquire(); //!< \brief Instructs the Ursa to stop acquiring.

    void startGM(); //!< \brief Instructs the Ursa to start acquiring in Gieger Muller mode.
    void stopGM(); //!< \brief Instructs the Ursa to stop acquiring and to switch out of Geiger Muller Mode.
    /** \brief In GM mode; Returns the number of counts since the last call to requestCounts().
     *
     * @return The number of counts as a unsigned 32 bit integer.
     */
    uint32_t requestCounts();

    void stopVoltage(); //!< \brief Immediately sets High Voltage to zero.  This is not stored to EEPROM.

    void requestBatt(); //!< \brief Sends a request to the Ursa to report the batteries voltage.
    /** \brief An access function which returns the last successful battery voltage reading.
     * @return The battery voltage in volts as a float.
     */
    float getBatt();

    /** \brief Instructs Ursa to run in ASCII mode.
     *
     * This driver can't process data in ASCII mode so this function is only included for completeness.
     */
    void startASCII();
    void stopASCII(); //!< \brief Switches Ursa out of ASCII mode.
    /** \brief Requests the serial number of the connected Ursa.
     *
     * @return The serial number as a int.
     */
    int requestSerialNumber();

    void requestMaxHV(); //!< \brief This is a utility function to request the maximum HV capable by the Ursa.
#ifdef ADMIN_ // These functions should normally only be executed by factory personal.
    /** \brief Sets the serial number of the connected Ursa.
     * @param serial The new serial number as an int.
     */
    void setSerialNumber(int serial);
    /** \brief Sets the smudge factor of the A/D converter.
     * @param smudge The smudge factor as an int.
     */
    void setSmudgeFactor(int smudge);
    /** \brief A function to set the Ursas internal register for max HV.
     * This function should NOT be used. The actual instruction to accomplish this is still unknown.
     * It is only assumed to be possible.
     */
    void setMaxHV(int HV);
#endif

    void loadPrevSettings(); //!< \brief A function to load previously set settings from EEPROM.
    void setNoSave(); //!< \brief This function instructs the Ursa to not save the next instructed HV to EEPROM.
    void setVoltage(int voltage);
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
