#include "BaseTracker.hpp"


template<class Obj>
double BaseTracker<Obj>::compareWith(const Obj &otherObject, const int mode, const int metric)
{
	return objectPTR->compareWith(otherObject, mode, metric);
}

template<class Obj>
double BaseTracker<Obj>::compareWith(const Detection<Obj>& detection, const int mode, const int metric)
{
	return detection.compareWith(objectPTR, mode, metric);
}

template<class Obj>
double BaseTracker<Obj>::compareWith(const BaseTracker<Obj>& tracker, const int mode, const int metric)
{
	return tracker.compareWith(objectPTR, mode, metric);
}

template<class Obj>
std::shared_ptr<Obj> BaseTracker<Obj>::getObjPTR()
{

	return this->objectPTR;
}
