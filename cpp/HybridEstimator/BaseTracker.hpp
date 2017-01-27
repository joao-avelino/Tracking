#ifndef BASETRACKER_HPP
#define BASETRACKER_HPP

#include <eigen3/Eigen/Core>
#include <memory>

using namespace Eigen;

//Forward declaration. Detection exists.
template <class Obj> class Detection;



template <class Obj> class BaseTracker
{

public:

	std::shared_ptr<Obj> getObjPTR()
	{
		return this->objectPTR;
	};

	int getObjectType()
	{

        return this->objectPTR->getObjectType();
	};

	double compareWith(Obj &otherObject, int mode, int metric)
	{
		return objectPTR->compareWith(otherObject, mode, metric);
	};

	double compareWith(Detection<Obj> &detection, int mode, int metric)
	{
		return detection.compareWith(*objectPTR, mode, metric);
	};


	double compareWith(BaseTracker<Obj> &tracker, int mode, int metric)
	{
		return tracker.compareWith(*objectPTR, mode, metric);
	};

protected:
	//Protected constructor no allowing the class to be instantiated
	BaseTracker() {};
	int trackerType;
	int trackerId;

	std::shared_ptr<Obj> objectPTR;

};
#endif // BASETRACKER_HPP
