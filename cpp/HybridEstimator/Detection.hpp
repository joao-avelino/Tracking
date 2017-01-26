#ifndef DETECTION_HPP
#define DETECTION_HPP
#include <memory>
#include <string>

//Forward declaration. BaseBayesianTracker exists
template <class Obj> class BaseBayesianTracker;

template <class Obj> class Detection
{

public:
	Detection<Obj>(std::shared_ptr<Obj> objectPTR, std::string sourceSensor) : objectPTR(objectPTR), sourceSensor(sourceSensor) {};


	std::string getSourceSensor() { return sourceSensor };
	void setSourceSensor(std::string newSourceSensor) { sourceSensor = newSourceSensor };

	std::shared_ptr<Obj> getObjPTR() { return objectPTR; };


	//Each object has it's own comparison methods

	double compareWith(Obj &otherObject, int mode, int metric) { return objectPTR->compareWith(otherObject, mode, metric);  };
	double compareWith(Detection<Obj> &detection, int mode, const int metric) { return detection.compareWith(objectPTR, mode, metric); };
	double compareWith(BaseBayesianTracker<Obj> &tracker, int mode, const int metric) { return tracker.compareWith(objectPTR, mode, metric);  };

private:
	std::string sourceSensor;
	std::shared_ptr<Obj> objectPTR;

};


#endif // DETECTION_HPP
