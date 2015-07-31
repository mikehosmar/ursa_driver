/**
 The MIT License (MIT)

 \file      main.cpp
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
  std::string port = "/dev/pts/7";
  int32_t baud = 115200;
  boost::array<unsigned int, 4096> array;
  std::ofstream myfile;
  myfile.open("outputwith90s.csv");

  ursa::Interface * ursa = new ursa::Interface(port.c_str(), baud);

  ursa->connect();

  if (ursa->connected())
    std::cout << "YAY" << std::endl;
  else
    return (-1);

  //ursa->setMaxHV(2000);
  //sleep(3);
  //ursa->requestMaxHV();
  //sleep(20);
  //ursa.setSerialNumber(200119);
  ursa->requestSerialNumber();
  //ursa->loadPrevSettings();
  ursa->setGain(25.293);
  ursa->setThresholdOffset(100);
  ursa->setInput(ursa::Interface::INPUT1NEG);

  ursa->setRamp(6);
  ursa->setVoltage(900);
  //sleep(20);
//  sleep(5);
//  ursa.setVoltage(0);
//  sleep(60);
  ursa->startAcquire();

  for (int i = 0; i < 90; i++)
  {
    ursa->read();
    sleep(1);
  }
  ursa->stopAcquire();
  ursa->read();
  ursa->getSpectra(&array);
//std::cout << boost::lexical_cast<std::string>(array[0]) << std::endl;
  for (boost::array<unsigned int, 4096>::iterator i = array.begin();
      i != array.end(); i++)
  {
    myfile << boost::lexical_cast<std::string>(*i) << "," << std::endl;
    //usleep(200000);
  }
  myfile.close();
  sleep(10);
  ursa->setVoltage(0);

}

