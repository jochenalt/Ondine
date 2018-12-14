/*
 * FIR filter class, by Mike Perkins
 * 
 * a simple C++ class for linear phase FIR filtering
 *
 * For background, see the post http://www.cardinalpeak.com/blog?p=1841
 *
 * Copyright (c) 2013, Cardinal Peak, LLC.  http://www.cardinalpeak.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1) Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2) Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 
 * 3) Neither the name of Cardinal Peak nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * CARDINAL PEAK, LLC BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


#include <Filter/FIRFilter.h>

using namespace FIR;

#define ECODE(x) {m_error_flag = x; return;}

// Handles LPF and HPF case
void Filter::init(filterType filt_t, float allowedRipple, float supression, float SamplingFrequency, float FilterFrequency) {
	// computation according to
	float numberOfTaps = 2.0/3.0 * log10(1.0/(10.0*allowedRipple*supression)*SamplingFrequency/FilterFrequency);
	init(filt_t, numberOfTaps, SamplingFrequency, FilterFrequency);
}

void Filter::init(filterType filt_t, int num_taps, float SamplingFrequency, float FilterFrequency)
{
	m_error_flag = 0;
	m_filt_t = filt_t;
	m_num_taps = num_taps;
	m_Fs = SamplingFrequency;
	m_Fx = FilterFrequency;
	m_lambda = M_PI * FilterFrequency / (SamplingFrequency/2);

	if( SamplingFrequency <= 0 ) ECODE(-1);
	if( FilterFrequency <= 0 || FilterFrequency >= SamplingFrequency/2 ) ECODE(-2);
	if( m_num_taps <= 0 || m_num_taps > MAX_NUM_FILTER_TAPS ) ECODE(-3);

	m_taps = m_sr = NULL;
	m_taps = (float*)malloc( m_num_taps * sizeof(float) );
	m_sr = (float*)malloc( m_num_taps * sizeof(float) );
	if( m_taps == NULL || m_sr == NULL ) ECODE(-4);
	
	init();

	if( m_filt_t == LOWPASS ) designLowPass();
	else if( m_filt_t == HIGHPASS ) designHighPass();
	else ECODE(-5);

	return;
}

// Handles BPF case
void Filter::init(filterType filt_t, int num_taps, float SamplingFrequency, float lowerFilterFrequency,
               float higherFilterFrequency)
{
	m_error_flag = 0;
	m_filt_t = filt_t;
	m_num_taps = num_taps;
	m_Fs = SamplingFrequency;
	m_Fx = lowerFilterFrequency;
	m_Fu = higherFilterFrequency;
	m_lambda = M_PI * lowerFilterFrequency / (SamplingFrequency/2);
	m_phi = M_PI * higherFilterFrequency / (SamplingFrequency/2);

	if( SamplingFrequency <= 0 ) ECODE(-10);
	if( lowerFilterFrequency >= higherFilterFrequency ) ECODE(-11);
	if( lowerFilterFrequency <= 0 || lowerFilterFrequency >= SamplingFrequency/2 ) ECODE(-12);
	if( higherFilterFrequency <= 0 || higherFilterFrequency >= SamplingFrequency/2 ) ECODE(-13);
	if( m_num_taps <= 0 || m_num_taps > MAX_NUM_FILTER_TAPS ) ECODE(-14);

	m_taps = m_sr = NULL;
	m_taps = (float*)malloc( m_num_taps * sizeof(float) );
	m_sr = (float*)malloc( m_num_taps * sizeof(float) );
	if( m_taps == NULL || m_sr == NULL ) ECODE(-15);
	
	init();

	if( m_filt_t == BANDPASS ) designBandPass();
	else ECODE(-16);

	return;
}

Filter::~Filter()
{
	if( m_taps != NULL ) free( m_taps );
	if( m_sr != NULL ) free( m_sr );
}

void 
Filter::designLowPass()
{
	int n;
	float mm;

	for(n = 0; n < m_num_taps; n++){
		mm = n - (m_num_taps - 1.0) / 2.0;
		if( mm == 0.0 ) m_taps[n] = m_lambda / M_PI;
		else m_taps[n] = sin( mm * m_lambda ) / (mm * M_PI);
	}

	return;
}

void 
Filter::designHighPass()
{
	int n;
	float mm;

	for(n = 0; n < m_num_taps; n++){
		mm = n - (m_num_taps - 1.0) / 2.0;
		if( mm == 0.0 ) m_taps[n] = 1.0 - m_lambda / M_PI;
		else m_taps[n] = -sin( mm * m_lambda ) / (mm * M_PI);
	}

	return;
}

void 
Filter::designBandPass()
{
	int n;
	float mm;

	for(n = 0; n < m_num_taps; n++){
		mm = n - (m_num_taps - 1.0) / 2.0;
		if( mm == 0.0 ) m_taps[n] = (m_phi - m_lambda) / M_PI;
		else m_taps[n] = (   sin( mm * m_phi ) -
		                     sin( mm * m_lambda )   ) / (mm * M_PI);
	}

	return;
}

void 
Filter::get_taps( float *taps )
{
	int i;

	if( m_error_flag != 0 ) return;

	for(i = 0; i < m_num_taps; i++) taps[i] = m_taps[i];

  return;		
}

int Filter::get_no_of_taps() {
	return m_num_taps;
}


void 
Filter::init()
{
	int i;

	if( m_error_flag != 0 ) return;

	for(i = 0; i < m_num_taps; i++) m_sr[i] = 0;

	return;
}

float
Filter::update(float data_sample)
{
	int i;
	float result;

	if( m_error_flag != 0 ) return(0);

	for(i = m_num_taps - 1; i >= 1; i--){
		m_sr[i] = m_sr[i-1];
	}	
	m_sr[0] = data_sample;

	result = 0;
	for(i = 0; i < m_num_taps; i++) result += m_sr[i] * m_taps[i];

	return result;
}
