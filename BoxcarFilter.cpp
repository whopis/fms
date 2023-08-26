#include "BoxcarFilter.h"

#include <iostream>

using namespace std;


BoxcarFilter::BoxcarFilter()
{
    //ctor
    length = 1;
    for (int i =0; i < 100; i++)
    {
        history[i] = 0;
    }
    total = 0;
    currentLength = 0;
    currentPos = 0;
}

BoxcarFilter::~BoxcarFilter()
{
    //dtor
}


void BoxcarFilter::setLength(int _length)
{
    length = _length;
    currentLength = 0;
    currentPos = 0;
}



float BoxcarFilter::addSample(float sample)
{
    total = total - history[currentPos];
    history[currentPos] = sample;
    total = total + history[currentPos];
    currentPos++;
    if (currentPos == length)
    {
        currentPos = 0;
    }
    if (currentLength < length)
    {
        currentLength++;
    }

    return total / currentLength;
}
