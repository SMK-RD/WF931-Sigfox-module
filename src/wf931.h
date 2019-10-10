/*
 *  wf931.h - WF931(Sigfox Module) Library Header
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#ifndef wf931_h
#define wf931_h

#include <sys/types.h>

class wf931Class
{
public:
  void begin();
  void end(){}

  void wakeup();
  void sleep();

  void send(int8_t,int8_t,int8_t,int8_t,int8_t,int8_t,int8_t,int8_t,int8_t,int8_t,int8_t,int8_t);
  void send(int16_t,int16_t,int16_t,int16_t,int16_t,int16_t);
  void send(float,float,float);

  bool result();

};

extern wf931Class wf931;

#endif /* wf931_h */

