#ifndef LONG_AVERAGE_FILTER_H
#define LONG_AVERAGE_FILTER_H
#include"MyAverageInterface.h"
#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif

#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif
extern int cp_num;

struct ListNode {
    int val;
    ListNode* next;

    ListNode() : ListNode(0) {}
    ListNode(int val) : ListNode(val, nullptr) {}
    ListNode(int val, ListNode* next) {
        this->val = val;
        this->next = next;
    }
};


class LongAverageFilter:public AverageInteface {
private:


    ListNode* startLink;
    //AverageInteface* inteface;
    mydata length;
    mydata average;
    int period, lastPeriod;
    demical summary;
    int calculator;
    bool state;
    const bool MAX = 1, MIN = 0;
    ListNode* stateNode;
    //std::vector<mydata> dataBuffer;
    //std::vector<mydata> autocorrelation;



    void deleteAll(ListNode* link);

    void push(mydata data);

    void pop();

    void calculateAverageOfLinkedList();

    



public:
    LongAverageFilter(mydata length);

    void reset();
    /*
    put data here.
    */
    void update(mydata data);

    /*
    get the average of this filter.
    */
    mydata getAverage();

    virtual void show(mydata data) override;

    ~LongAverageFilter();
};

#endif  // LONG_AVERAGE_FILTER_H
