#include "MyAverageFilter.h"
#include<arduino.h>

void MyAverageFilter::calculateAverage()
{

  average = 0;
  for (int i = 0; i < length; ++i) {
    average += *(my_queue + i);
  }
  average = average / length;

  //calculateAverageOfLinkedList();
}

void MyAverageFilter::reset(mydata absoluteError, demical relativeError){
  this->absoluteError = absoluteError;
  this->relativeError = relativeError;
}

bool MyAverageFilter::checkBalence()
{
  if (isFull) {
    calculateAverage();

    for (int i = 0; i < length; ++i) {
      if (!checkSingleData(*(my_queue + i))) return false;
    }
    /*
      Link* node = startLink;
      while (node != nullptr) {
    	if (!checkSingleData(node->val)) return false;
      }
    */
  }
  return true;
}

bool MyAverageFilter::checkSingleData(mydata data)
{
  mydata absoluteError = abs(data - average);
  if (absoluteError > this->absoluteError) return false;
  if ((demical)absoluteError / (demical)average > relativeError) return false;
  return true;
}

void MyAverageFilter::addElement(mydata data)
{
  *(my_queue + calculator) = data;
  calculator = (++calculator) % length;
  //push(data);
  //pop();
}

void MyAverageFilter::reset()
{
  isFull = false;
  isBalence = false;
  average = 0;
  summary = 0;
  calculator = 0;
  for (int i = 0; i < length; ++i) {
    *(my_queue + i) = 0;
  }


  /*
  	Link* node = startLink;
  	while (node != nullptr) {
  		node->val = 0;
  		node = node->next;
  	}
  */
}

MyAverageFilter::MyAverageFilter(AverageInteface* averageInteface) : MyAverageFilter(averageInteface, _MY_AVERAGE_FILTER_)
{

}

MyAverageFilter::MyAverageFilter(AverageInteface* averageInteface, mydata length, mydata absoluteError, demical relativeError)
{
  //todo
  Serial.print("init MyAverageFilter... ");
  my_queue = new  mydata[length];
  /*
    Link* node = new Link(0);
    startLink = new Link(0,node);
    cp_num += 2;
    for (int i = 2; i < length; ++i) {
  	node->next = new Link(0);
  	node = node->next;
  	cp_num++;
    }
  */
  this->averageInteface = averageInteface;
  this->longFilter = new LongAverageFilter(120);//50 sampling points remains
  //this->longFilter2 = new LongAverageFilter(1);//50 sampling points remains
  cp_num+=2;

  this->length = length;
  this->absoluteError = absoluteError;
  this->relativeError = relativeError;



  //this->averageInteface->show(111);
  reset();
  Serial.println("Successfully! ");
}

MyAverageFilter::~MyAverageFilter()
{
  delete[] my_queue;
  cp_num-=2;
  delete this->longFilter;
  //delete this->longFilter2;
  //deleteAll(startLink);
}

void MyAverageFilter::update(mydata data)
{
  //remenber to delete
  //push(data);
  //pop();
  //remenber to delete

    //Serial.print("data in af:");
    
    longFilter->update(data);
    //Serial.println(longFilter->getAverage());
    //longFilter2->update(longFilter->getAverage());

    show(longFilter->getAverage());
    //show(data);

}

void MyAverageFilter::show(mydata data)
{    
    //averageInteface->show(data);
    //return;
    if (isBalence) {
        if (checkSingleData(data)) {
            averageInteface->show(average);
            
            return;
        }
        else {
        }
        reset();
    }
    else {

        if (isFull) {
            if (checkBalence()) {
                isBalence = true;
            }
        }
        else {
            if (calculator + 1 == this->length) {
                isFull = true;
            }
        }
    }

    this->addElement(data);
    averageInteface->show(data);

}

mydata MyAverageFilter::getAverage(){
  return this->average;
}

MyAverageFilter::MyAverageFilter(AverageInteface* averageInteface, mydata length) : MyAverageFilter(averageInteface, length, 10, 0.05)
{

}

//mydata MyAverageFilter::abs(mydata data){
//  return data > 0? data:-data;
//}

Link::Link(): Link(0)
{
}

Link::Link(int val): Link(val, nullptr)
{
}

Link::Link(int val, Link* next)
{
  this->val = val;
  this->next = next;
}
