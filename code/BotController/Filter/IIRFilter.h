/***
 * IIR Filter Library - Header
 *
 * Copyright (C) 2016  Martin Vincent Bloedorn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/** \TODO Rename low-pass filter coefficients to a more standard a0..aN, b0..bN notation.
 *  \TODO Recompute low-pass formulations with Bilinear transforms.
 *  \TODO Put all required coefficients in vector form (a[0] instead of a0), ...
 *  \TODO Rewrite all difference equations to be in a single form. (eliminate High/Low distinction),
 *        and allow everything to be shoved nicely into a single for loop (for i=0; ... i<od)
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <Arduino.h>

namespace IIR {
  const uint8_t MAX_ORDER = 5;
  enum class ORDER  : uint8_t {OD1 = 0, OD2, OD3, OD4};//, OD5};
  enum class TYPE   : uint8_t {LOWPASS = 0, HIGHPASS = 1};

  const float SQRT2 = sqrt(2.0);
  const float SQRT3 = sqrt(3.0);
  const float SQRT5 = sqrt(5.0);

class Filter {
public:

  /** \brief Filter class.
   *
   */
  Filter(float hz_, float ts_, ORDER od_, TYPE ty_ = TYPE::LOWPASS);
  void init(float hz_, float ts_, ORDER od_, TYPE ty_ = TYPE::LOWPASS) {
	  ts = ts_;
	  hz = hz_;
	  od = od_;
	  ty = ty_;
	  init(true);
  }

  Filter();

  ~Filter();

  float update(float input);
  float get();

  void flush();
  void init(bool doFlush=true);


  bool isInErrorState() { return f_err;  }
  bool isInWarnState()  { return f_warn; }
  void dumpParams();

private:
  float ts;	// sample rate in [s]
  float hz;	// cutoff frequency [Hz]
  ORDER od;	// order
  TYPE  ty;	// Low pass or high pass

  // Difference equation terms
  float k0, k1, k2, k3, k4, k5;
  float j0, j1, j2;

  // Filter buffer
  float y[MAX_ORDER], u[MAX_ORDER];

  bool f_err, f_warn; ///< Numerical error or warning; only relevant for 8-bit micros

  inline float computeLowPass(float input);
  inline float computeHighPass(float input);

  /** \brief Computes the discrete coefficients for a Butterworth low-pass filter via pole-zero matching.
   *  Up to order 4.
   */
  inline void  initLowPass();

  /** \brief Computes the discrete coefficients for a Butterworth high-pass filter via Bilinear-Transformation.
   *  Up to order 2 (I was lazy).
   */
  inline void  initHighPass();
};
}



#endif /* FILTER_H_ */
