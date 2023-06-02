#ifndef _MY_AVERAGE_FILTER_
#define _MY_AVERAGE_FILTER_ 3
#include <Math.h>
#include "MyAverageFilter.h"
//#include<queue>
#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif



#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif
//using namespace std;
#include "LongAverageFilter.h"

extern int cp_num;


struct Link {
  int val;
  Link* next;
  Link();
  Link(int val);
  Link(int val, Link* next);
};



class MyAverageFilter:public AverageInteface {
  private:
    Link* startLink;
    mydata length;
    mydata average;
    mydata summary;
    mydata absoluteError;
    int calculator;
    demical relativeError;
    AverageInteface* averageInteface;
    LongAverageFilter* longFilter;
    mydata* my_queue;
    bool isFull;

    void calculateAverage();

    bool checkBalence();

    /*
    check whether new data is under the error of the average.
    */
    bool checkSingleData(mydata data);

    void addElement(mydata data);

    void reset();
  public:
    bool isBalence;
    MyAverageFilter(AverageInteface* averageInteface);

    MyAverageFilter(AverageInteface* averageInteface, mydata length);

    MyAverageFilter(AverageInteface* averageInteface, mydata length, mydata absoluteError, demical relativeError);

    ~MyAverageFilter();

    /*
    update data
    */
    void update(mydata data);

    mydata getAverage();

    void reset(mydata absoluteError, demical relativeError); 

    virtual void show(mydata data) override;
};


#endif
