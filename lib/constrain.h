/**************************************************************
 * Copyright (C) 2020 Vtol Develop Team - All Rights Reserved
 *************************************************************/

/**
 * @file the constrain lib function
 * @author zhongzhw zhongzhwin@163.com
 */

#ifndef CONSTRAIN_H
#define CONSTRAIN_H

#include <cmath>

/**
 * @function map the degree to [-pi, pi)
 * @param rad
 * @return rad
 */
template<class T>
T wrap_pi(T x) {
    if(!std::isfinite(x)) {
        return x;
    }

    int c = 0;
    while(x >= static_cast<T>(M_PI)) {
        x -= static_cast<T>(2 * M_PI);
        if(c++ > 100) {
            return INFINITY;
        }
    }

    c = 0;
    while(x < static_cast<T>(-M_PI)) {
        x += static_cast<T>(2 * M_PI);
        if(c++ > 100) {
            return INFINITY;
        }
    }

    return x;
}

/**
 * @function map the degree to [0, 2pi)
 * @param rad
 * @return rad
 */
template<class T>
T wrap_2pi(T x) {
    if(!std::isfinite(x)) {
        return x;
    }

    int c = 0;
    while(x >= static_cast<T>(2 * M_PI)) {
        x -= static_cast<T>(2 * M_PI);
        if(c++ > 100) {
            return INFINITY;
        }
    }

    c = 0;
    while(x < static_cast<T>(0)) {
        x += static_cast<T>(2 * M_PI);
        if(c++ > 100) {
            return INFINITY;
        }
    }

    return x;
}

/**
 * @fucnton: constrain the value between min and max
 * @param val input
 * @param min
 * @param max
 */
template<typename _Tp>
constexpr _Tp constrain(_Tp val, _Tp min_val, _Tp max_val)
{
	return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/**
 * @function constrain the value between min and max
 * @param input data
 * @param min
 * @param max
 */
template<class T>
void constrain_min_max(T& value, float min, float max) {
    if(value < min)
        value = min;
    else if(value > max)
        value = max;
}

/**
 * @function constrain the value abs
 * @param input data
 * @param abs_max
 */
template<class T>
void constrain_max_abs(T& value, float abs_max) {
    if(fabs(value) > fabs(abs_max)){
        if(value > 0)
            value = fabs(abs_max);
        else
            value = -fabs(abs_max);
    }
}

#endif