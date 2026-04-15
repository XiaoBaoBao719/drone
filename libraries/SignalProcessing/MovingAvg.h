#include <math.h>
#include <stdint.h>
#include <Arduino.h>

#ifndef MOVINGAVG_H
#define MOVINGAVG_H

template <int order>
class MovingAvg
{
private:
    float filteredData[order+1];
    float filteredHist[order+1];
    
    float _filteredInput;
    float filteredOutput;
    uint32_t filterIndex;

public:
    MovingAvg();

    void filter(float input, float &output);
    float filter(float input);
    // float getMovingAvgData(float *data);
};

template <int order>
MovingAvg<order>::MovingAvg()
{
    filteredHist[order] = {0.0};
    filterIndex = 0;
}

template <int order>
void MovingAvg<order>::filter(float input, float &output)
{
    _filteredInput = input;
    for (uint32_t i = 0; i < order; i++) 
    {
        _filteredInput += filteredHist[i];
    }

    filteredOutput = _filteredInput / (order + 1);
    filteredHist[filterIndex] = filteredOutput;

    filterIndex++;
    if (filterIndex == order) {
        filterIndex = 0;
    }

    output = filteredOutput;
}

template <int order>
float MovingAvg<order>::filter(float input)
{
    _filteredInput = input;
    for (uint32_t i = 0; i < order; i++) 
    {
        _filteredInput += filteredHist[i];
    }

    filteredOutput = _filteredInput / (order + 1);
    filteredHist[filterIndex] = filteredOutput;

    filterIndex++;
    if (filterIndex == order) {
        filterIndex = 0;
    }

    return filteredOutput;
}

#endif