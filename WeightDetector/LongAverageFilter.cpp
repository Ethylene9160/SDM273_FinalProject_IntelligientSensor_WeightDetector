#include "LongAverageFilter.h"
#include<arduino.h>
//int cp_num = 0;





LongAverageFilter::LongAverageFilter(mydata length) {
  average = 0;
  state = MAX;
  //this->inteface = inteface;
  
  if (length < 2) length = 2;
  this->length = length;
  this->period = this->length;
  this->lastPeriod = this->period;

  ListNode* node = new ListNode(0);
  startLink = new ListNode(0, node);
  cp_num += 2;
  for (int i = 2; i < length; ++i) {
    node->next = new ListNode(0);
    node = node->next;
    cp_num++;
  }
}

void LongAverageFilter::deleteAll(ListNode* link) {
  while (link != nullptr) {
    ListNode* l = link;
    link = l->next;
    delete l;
    cp_num--;
  }
}

void LongAverageFilter::push(mydata data) {
  ListNode* node = startLink;
  while (node->next != nullptr) {
    node = node->next;
  }
  node->next = new ListNode(data);
  cp_num++;
  summary = summary + (demical)data / (demical)period;
}

void LongAverageFilter::pop() {
  ListNode* node = startLink;
  startLink = startLink->next;
  summary = summary - (demical)node->val / (demical)period;
  delete node;
  cp_num--;
}

void LongAverageFilter::reset() {
  ListNode* node = startLink;
  while (node != nullptr) {
    node->val = 0;
    node = node->next;
  }
  average = 0;
  summary = 0;
}

void LongAverageFilter::calculateAverageOfLinkedList() {



  mydata sum = 0;
  int len = 0;
  int max = startLink->val;
  int min = startLink->val;
  ListNode* node = startLink;
  ListNode* maxNode = startLink;
  ListNode* minNode = startLink;
  int startIndex = 0, endIndex = 0, tpIndex = 0;
  while (node->next != nullptr && node->next->next != nullptr) {
    //if (state == MAX) {
    if (node->next->val > node->val && node->next->val > node->next->next->val) {
      if (maxNode->val < node->next->val) {
        maxNode = node->next;
        startIndex = tpIndex;
        //stateNode = node->next;
      }
    }
    //}
    //else {
    else if (node->next->val <  node->val && node->next->val < node->next->next->val) {
      if (node->next->val < minNode->val) {
        minNode = node->next;
        //stateNode = node->next;
        endIndex = tpIndex;
      }
    }
    //}
    node = node->next;
    ++tpIndex;
  }
  //state = !state;
  //if (average == INT_MIN) average = max;
  int t = abs(startIndex-endIndex);
  if(t>0) period = t;
  //if(this->period)
  //average = (maxNode->val + minNode->val) / 2;
  //this->summary = average;

  
 
 
  //Serial.print("average: ");
  //Serial.println(average);
  //this->inteface->show(average);


  /*
    while (node != nullptr) {
      sum += node->val;
      len++;
      node = node->next;
    }
  */
  //average = sum / len;
}

void LongAverageFilter::update(mydata data) {
  // Add new data point to the buffer
  //dataBuffer.push_back(data);

  //if (dataBuffer.size() >= length) {
  // Calculate autocorrelation function
  //calculateAutocorrelation();

  // Check the estimated period
  //checkPeriod();
  //}

  // Update the filter by adding a new data point

  //inteface->show(data);
  //return;

  pop();
  push(data);
  //Serial.print("avg: ");
  //Serial.println((mydata)summary);
  //inteface->show((mydata)summary);

  ListNode* node = startLink;
  average = 0;
  for(int i = 0; i < period; ++i, node = node->next){
    average+=node->val;
  }
  average = average/period;
  //Serial.print("period: ");
  //Serial.println(period);
  
  //if (++calculator % length) return;
  //calculateAverageOfLinkedList();
  //calculator = 0;
}

void LongAverageFilter::show(mydata data) {
  //this->inteface->show(data);
}

mydata LongAverageFilter::getAverage() {
//  return average;
return summary;
}

LongAverageFilter::~LongAverageFilter() {
  //cp_num--;
  deleteAll(startLink);
  //delete inteface;
  //delete this->basicFilter;
  //cp_num--;
}
