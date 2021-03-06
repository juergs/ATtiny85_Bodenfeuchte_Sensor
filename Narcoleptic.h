/**
 * Narcoleptic - A sleep library for Arduino
 * Copyright (C) 2010 Peter Knight (Cathedrow)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef Narcoleptic_h
#define Narcoleptic_h

#include <inttypes.h>
#include <stdbool.h>
#include <Arduino.h>

class NarcolepticClass
{
  public:
    void delay_millis(unsigned int milliseconds);
	  void delay_minutes(int minutes);
  private:
    void sleep(uint8_t);
};
extern NarcolepticClass Narcoleptic;

#endif
