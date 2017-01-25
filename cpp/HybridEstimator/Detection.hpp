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

	std::shared_ptr<Obj> getObjPTR() {return objectPTR};


	//Each object has it's own comparison methods

	double compareWith(const Obj &otherObject, const int mode, const int metric) { return objectPTR->compareWith(otherObject, mode, metric)};
	double compareWith(const Detection<Obj> &detection, const int mode, const int metric) { return detection.compareWith(objectPTR, mode, metric); };
	double compareWith(const BaseBayesianTracker<Obj> &tracker, const int mode, const int metric) { return tracker.compareWith(objectPTR, mode, metric);  };

private:
	std::string sourceSensor;
	std::shared_ptr<Obj> objectPTR;

};


#endif // DETECTION_HPP
