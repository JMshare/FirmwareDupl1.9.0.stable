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

/// @file	Detect_oscillations_Vector3f.hpp
/// @brief	A class to implement oscillations detection on a Vector3f

#pragma once

#include <matrix/math.hpp>

namespace math
{
class Detect_oscillations_Vector3f
{
public:

	Detect_oscillations_Vector3f(float pksz, float dtlim)
	{
		reset(pksz, dtlim);
	}

	inline matrix::Matrix<int,3,1> apply(const matrix::Matrix<float,3,1> &sample, const uint64_t time)
	{
		for(int i=0; i<3; i++){
			if(sample(i,0) > _pksz){ // above the upper bound
				if(_flip(i,0) != 1){ // just crossed
					_flip(i,0) = 1;
					_dtflip_up(i,0) = time - _tflip_dw(i,0); // how long it took to flip up from the last flip down
					_tflip_up(i,0) = time; // update the flip up time
				}
			}
			if(sample(i,0) < -_pksz){ // bellow the lower bound
				if(_flip(i,0) != -1){ // just crossed
					_flip(i,0) = -1;
					_dtflip_dw(i,0) = time - _tflip_up(i,0); // how long it took to flip down from the last flip up
					_tflip_dw(i,0) = time; // update the flip down time
				}
			}
			if((_dtflip_dw(i,0)/1000000.0f < _dtlim) && (_dtflip_up(i,0)/1000000.0f < _dtlim) && (_dtflip_dw(i,0)!=0) && (_dtflip_up(i,0)!=0)){ // if it fipped up-and-down or down-and-up both in less that dtlim, oscillating 
				_oscillating(i,0) = 1;
				if(_tflip_up(i,0) < _tflip_dw(i,0)){ // need to reset one of the dtflips now
					_dtflip_up(i,0) = 0;
				}
				else{
					_dtflip_dw(i,0) = 0;
				}

			}
			else{
				_oscillating(i,0) = 0;
			}
			
		}

		return _oscillating;
	}

	int reset(float pksz, float dtlim){
		_pksz = pksz;
		_dtlim = dtlim;
		_tflip_up.setAll(0);
		_tflip_dw.setAll(0);
		_dtflip_dw.setAll(0);
		_dtflip_up.setAll(0);
		_flip.setAll(0);
		_oscillating.setAll(0);
		return PX4_OK;
	}

private:
	float _pksz{0.1f};
	float _dtlim{0.2f};
	matrix::Matrix<int,3,1> _flip;
	matrix::Matrix<int,3,1> _oscillating;	
	matrix::Matrix<uint64_t,3,1> _tflip_up;
	matrix::Matrix<uint64_t,3,1> _tflip_dw;
	matrix::Matrix<uint64_t,3,1> _dtflip_up;
	matrix::Matrix<uint64_t,3,1> _dtflip_dw;
};

} // namespace math
