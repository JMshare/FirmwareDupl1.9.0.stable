/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "LowPassFilter3pVector3f.hpp"

#include <px4_defines.h>

#include <cmath>

namespace math
{

void LowPassFilter3pVector3f::set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
	_cutoff_freq = cutoff_freq;

	// reset delay elements on filter change
	_delay_element_1.zero();
	_delay_element_2.zero();
	_delay_element_1p.zero();
	_delay_element_2p.zero();

	if (_cutoff_freq <= 0.0f) {
		// no filtering
		_b0 = 1.0f;
		_b1 = 0.0f;
		_b2 = 0.0f;

		_a1 = 0.0f;
		_a2 = 0.0f;

		_b0p = 1.0f;
		_b1p = 0.0f;
		_b2p = 0.0f;

		_a1p = 0.0f;
		_a2p = 0.0f;

		return;
	}

	const float fr = sample_freq / _cutoff_freq;
	const float ohm = tanf(M_PI_F / fr);
	const float ohm2 = ohm*ohm;

	const float r = sinf(M_PI_F/8.0f);
	const float c = 1.0f + 2.0f*r*ohm + ohm2;

	_b0 = ohm2 / c;
	_b1 = 2.0f * _b0;
	_b2 = _b0;

	_a1 = 2.0f * (ohm2 - 1.0f) / c;
	_a2 = (1.0f - 2.0f*r*ohm + ohm2) / c;

	const float rp = sinf(M_PI_F*3.0f/8.0f);
	const float cp = 1.0f + 2.0f*rp*ohm + ohm2;
	_b0p = ohm2 / cp;
	_b1p = 2.0f * _b0p;
	_b2p = _b0p;

	_a1p = 2.0f * (ohm2 - 1.0f) / cp;
	_a2p = (1.0f - 2.0f*rp*ohm + ohm2) / cp;
}

matrix::Vector3f LowPassFilter3pVector3f::reset(const matrix::Vector3f &sample)
{
	const matrix::Vector3f dval = sample / (_b0 + _b1 + _b2);

	if (PX4_ISFINITE(dval(0)) && PX4_ISFINITE(dval(1)) && PX4_ISFINITE(dval(2))) {
		_delay_element_1 = dval;
		_delay_element_2 = dval;

	} else {
		_delay_element_1 = sample;
		_delay_element_2 = sample;
	}

	const matrix::Vector3f dvalp = sample / (_b0p + _b1p + _b2p);

	if (PX4_ISFINITE(dvalp(0)) && PX4_ISFINITE(dvalp(1)) && PX4_ISFINITE(dvalp(2))) {
		_delay_element_1p = dvalp;
		_delay_element_2p = dvalp;

	} else {
		_delay_element_1p = sample;
		_delay_element_2p = sample;
	}

	return apply(sample);
}

} // namespace math
