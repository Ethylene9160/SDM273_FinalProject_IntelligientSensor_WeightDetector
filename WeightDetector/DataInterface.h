#ifndef SELF_DATA_INTERFACE
#define SELF_DATA_INTERFACE
#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif
class AngleDetector {
public:
	virtual demical getAngle() = 0;
};

class Closable{
public:
	virtual void close() = 0;  
};

class Startable {
	virtual void begin() = 0;
};

#endif
