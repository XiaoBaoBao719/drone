#include <math.h>
#include <stdint.h>

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

template <int order>   // Low pass filter of order 1 or 2
class LowPassFilter
{
    private:
        float a[order];
        float b[order+1];
        float omega0;
        float dt;
        bool adapt;
        float tn1 = 0;
        float x[order+1]; // Raw values
        float y[order+1]; // Filtered values

    public:
        LowPassFilter(float cutoff_freq, float sample_freq, bool adaptive = false);

        void setCoeffs();
        void filter(float input, float &output);
        float filter(float input);

};

#endif