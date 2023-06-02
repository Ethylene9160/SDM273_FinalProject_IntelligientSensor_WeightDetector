#ifndef TO_WEIGHT_DATA
#define TO_WEIGHT_DATA 6

#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif

#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif

class ToWeightData{

public:
  static mydata analog2digit(demical deltaAngualr, demical deltax);

  static mydata analog2error(demical data);

  //deprecated
  static mydata data2weight(mydata data);

  static mydata digit2analog(demical data);

  //transport the angle to weight, when the object to be measured is light
  static mydata smallTransport(mydata data);

  //transport the angle to weight, when the object to be measured is heavy
  static mydata largeTransport(mydata data);
};

//mydata analog2digit(demical deltaAngular, demical deltax);

#endif
