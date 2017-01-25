#include "BaseBayesianTracker.hpp"
#include "Detection.hpp"


template<class Obj>
BaseBayesianTracker<Obj>::BaseBayesianTracker(std::shared_ptr<Obj> objectPTR)
{
	this->objectPTR = objectPTR;
	this->trackerType = objectPTR->getObjectType();
}
