#ifndef _BMM_H
#define _BMM_H

#include <math.h>

enum param {MAX_RANGE=0, MIN_RANGE, P_SHORT_LAMBDA, P_MAX_START, P_HIT_U, P_HIT_SIGMA};

class beamMeasurementModel
{
private:
	int _maxRange;
	int _minRange;
	
	/* exponential parameters = p_short */
	float _pShortLambda;

	/* uniform noise = p_rand */

	/* max range uniform spike parameters = p_max */
	int _startMax;

	/* expected gaussian parameters = p_hit */
	float _uHit;
	float _sigmaHit;

	float eval_pRand(float) const;
	float eval_pMax(float) const;
	float eval_pShort(float) const;
	float eval_pHit(float) const;
public:
	beamMeasurementModel();
	bool set_param(param, float);
	bool get_param(param, float*) const;

	/* Return P(Z_t | x_t) */
	float getP(float x_t) const;
};

#endif