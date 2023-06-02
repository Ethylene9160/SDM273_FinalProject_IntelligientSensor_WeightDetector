#ifndef MY_AVERAGE_INTERFACE
#define MY_AVERAGE_INTERFACE
#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif
class AverageInteface {
public:
    virtual void show(mydata data) = 0;
};
#endif
