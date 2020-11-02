#include "../include/Lmath.h"

float LimitRadianRange(float radian)
{
	if (radian <= -PI)
	{
		return radian + 2*PI;
	}
	else if(radian > PI)
	{
		return radian - 2*PI;
	}
    else
    {
        return radian;
    }
}