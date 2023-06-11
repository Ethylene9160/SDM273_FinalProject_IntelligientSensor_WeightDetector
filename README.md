# SDM273_FinalProject_IntelligientSensor_WeightDetector
This is a weight detector(as a final project for SDM273 in SUSTC), after debugging.

## Kalman Filter

in `Kalman.h` and `Kalman.cpp`. This is an outer lib.

## MyAverageFilter

This is an average filter originally, but has been alterned to a data-stablizer for the experimaters. 

Constructors:

`MyAverageFilter(AverageInteface* averageInteface);`

`MyAverageFilter(AverageInteface* averageInteface, mydata length)`

`MyAverageFilter(AverageInteface* averageInteface, mydata length, mydata absoluteError, demical relativeError)`

The default length is defined in the header file. Default `absoluteError` is 10 and `relativeError` is 5%.

Main function(s):

`void pudata(mydata data)`: put a new data into it, it will judge the stability of the system, and output through interface `AverageInteface->show(processedData)`.

## AverageInterface

A proxy interface with a pure virtual function:

`virtual void show(mydata data)`: After processing the data, the filter will call this function aotumatically.

## LongAverageFilter

a integrating filter, was built into the `MyAverageFilter`.
