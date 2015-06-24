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


int main (int argc, char **argv) {
  std::string port = "/dev/ttyACM0";
  int32_t baud = 115200;
  boost::array<unsigned int, 4096>  array;

  ursa::Interface ursa(port.c_str(), baud);

  ursa.connect();

  if(ursa.connected()) std::cout << "YAY" << std::endl;
  else
    return (-1);
  ursa.startAcquire();

  while(1){
ursa.read();
ursa.getPulses(&array);
std::cout << boost::lexical_cast<std::string>(array[0]) << std::endl;
sleep(1);
  }
}


