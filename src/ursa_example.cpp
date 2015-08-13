/**
 The MIT License (MIT)

 \file      ursa_example.cpp
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

#include "ursa_driver/ursa_driver.h"
#include <iostream>
#include <fstream>

int main(int argc, char **argv) {
  std::string port = "/dev/pts/4";
  int32_t baud = 115200;

  boost::array<uint32_t, 4096> array;
  std::ofstream myfile;
  myfile.open("sample_output.csv");

  bool GMmode = true;

  ursa::Interface * ursa = new ursa::Interface(port.c_str(), baud);

  ursa->connect();

  if (ursa->connected())
    std::cout << "Connected" << std::endl;
  else
    return (-1);

  ursa->requestSerialNumber();
  //ursa->loadPrevSettings();
  ursa->setGain(70);
  ursa->setThresholdOffset(100);
  ursa->setShapingTime(ursa::Interface::TIME1uS);
  ursa->setInput(ursa::Interface::INPUT1NEG);
  ursa->setRamp(6);

  ursa->setVoltage(900); //use appropriate settings*/

  if (GMmode)
  {
    ursa->startGM();
    for (int i = 0; i < 30; i++)
    {
      std::cout << "Approx CPS: " << ursa->requestCounts() << std::endl; //number of counts since last read
      sleep(1); //Terrible way to get 1Hz (actual rate on my pc ~.8 Hz)
    }
    ursa->stopGM();
  }

  else
  {
    ursa->startAcquire();
    for (int i = 0; i < 90; i++) //~90 seconds
    {
      ursa->read();  //process incoming data
      sleep(1);
    }
    ursa->stopAcquire(); //stop acquiring
    ursa->read();      //one last read
    ursa->getSpectra(&array); //retrieve the spectra

    for (boost::array<uint32_t, 4096>::iterator i = array.begin();
        i != array.end(); i++)
    {
      myfile << boost::lexical_cast<std::string>(*i) << "," << std::endl; //fill the csv file
    }
    myfile.close();
  }
  ursa->setVoltage(0); //disable HV

}

