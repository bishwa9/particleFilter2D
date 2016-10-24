#include "bmm.h"

beamMeasurementModel::beamMeasurementModel():
_maxRange(1000),
_minRange(0),
_pShortLambda(0.005),
_startMax(990),
_uHit(500),
_sigmaHit(30)
{}

bool beamMeasurementModel::get_param(param param_type, float *val) const
{
	switch(param_type)
	{
		case MAX_RANGE:
		{
			*val = static_cast<float> (_maxRange);
			return true;
		}
		case MIN_RANGE:
		{
			*val = static_cast<float> (_maxRange);
			return true;
		}
		case P_SHORT_LAMBDA:
		{
			*val = _pShortLambda;
			return true;
		}
		case P_MAX_START:
		{
			*val = static_cast<float> (_startMax);
			return true;
		}
		case P_HIT_U:
		{
			*val = _uHit;
			return true;
		}
		case P_HIT_SIGMA:
		{
			*val = _sigmaHit;
			return true;
		}
		default: 
		{
			return false;
		}
	}
}

bool beamMeasurementModel::set_param(param param_type, float val)
{
	switch(param_type)
	{
		case MAX_RANGE:
		{
			_maxRange = static_cast<int> (val);
			return true;
		}
		case MIN_RANGE:
		{
			_minRange = static_cast<int> (val);
			return true;
		}
		case P_SHORT_LAMBDA:
		{
			_pShortLambda = val;
			return true;
		}
		case P_MAX_START:
		{
			_startMax = static_cast<int> (val);
			return true;
		}
		case P_HIT_U:
		{
			_uHit = val;
			return true;
		}
		case P_HIT_SIGMA:
		{
			_sigmaHit = val;
			return true;
		}
		default: 
		{
			return false;
		}
	}
}

float beamMeasurementModel::eval_pRand(float x_t) const
{
	// Eq. 6.12 Pg. 128
	if( x_t >= _minRange && x_t <= _maxRange )
	{
		return 1.0 / (_maxRange - _minRange);
	}
	else
	{
		return 0.0;
	}
}
float beamMeasurementModel::eval_pMax(float x_t) const
{
	// Eq. 6.11 Pg. 127
	if( x_t >= _startMax && x_t <= _maxRange )
	{
		return 1.0 / (_maxRange - _startMax);
	}
	else
	{
		return 0.0;
	}
}
float beamMeasurementModel::eval_pShort(float x_t) const
{
	// Eqs. 6.7 - 6.10 Pg. 127
	if(x_t >= _minRange && x_t <= _uHit)
	{
		float exp_term = -1.0 * _pShortLambda * x_t;
		//un normalized value
		float unN_val = _pShortLambda * exp(exp_term);
		//normalize
		exp_term = -1.0 * _pShortLambda * _uHit;
		float eta = 1 / ( 1 - exp(exp_term) );
		float val = eta * unN_val;

		return val;
	}
	else
	{
		return 0.0;
	}
}
float beamMeasurementModel::eval_pHit(float x_t) const
{
	// Eqs. 6.4 - 6.6 Pg. 126
	if(x_t >= _minRange && x_t <= _maxRange)
	{
		float varHit = pow(_sigmaHit, 2);
		//coefficient
		float coeff = 1 / sqrt( 2*M_PI*varHit );

		//exponent term
		float diff = x_t - _uHit;
		float exp_term = pow(diff, 2) / (varHit);
		exp_term = -0.5 * exp_term;

		//un-normalized val
		float unN_val = coeff * exp(exp_term);

		//normalize
		float eta = 1;
		float val = eta * unN_val;

		return val;
	}
	else
	{
		return 0.0;
	}
}

float beamMeasurementModel::getP(float x_t) const
{
	// individual p components
	float pHit = eval_pHit(x_t);
	float pShort = eval_pShort(x_t);
	float pMax = eval_pMax(x_t);
	float pRand = eval_pRand(x_t);

	// weighted average
	float hitW = 1;
	float shortW = 1;
	float maxW = 0.2;
	float randW = 1;

	float p = hitW * pHit + shortW * pShort + maxW * pMax + randW * pRand;

	return p;
}