#include "LowPassFilter.h"

template <int order>
LowPassFilter<order>::LowPassFilter(float cutoff_freq, float sample_freq, bool adaptive)
{
    // cutoff_freq : cutoff frequency (Hz)
    // sample_freq : sample frequency (Hz)
    // adaptive: boolean flag, if set to 1, the code will automatically set
    // the sample frequency based on the time history.

    omega0 = 6.28318530718 * cutoff_freq;
    dt = 1.0 / sample_freq;
    adapt = adaptive;
    tn1 = -dt;
    for (int k = 0; k < order + 1; k++)
    {
        x[k] = 0;
        y[k] = 0;
    }
    setCoeffs();
}

template <int order>
void LowPassFilter<order>::setCoeffs()
{
    if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(this->order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(this->order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
}

template <int order>
void LowPassFilter<order>::filter(float input, float &output)
{
    /* Take the raw value 'input' and return the filtered value in 'output' */
    if (adapt) { setCoeffs(); }

    x[0] = input;
    y[0] = 0;

    // Compute the filtered values
    for (int k = 0; k < order; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
    }

    /* The filtered value is calculated by multiplying each coefficient by it's respective order */
    y[0] += b[order] * x[order];

    /* Lastly, save the filtered values for the next update */
    for (int k = order; k > 0; k--) {
        x[k] = x[k - 1];
        y[k] = y[k - 1];
    }

    // Return the filtered value
    output = y[0];
}