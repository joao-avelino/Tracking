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

	std::shared_ptr<Obj> getObjPTR();

	double compareWith(const Obj &otherObject, const int mode, const int metric);
	double compareWith(const Detection<Obj> &detection, const int mode, const int metric);
	double compareWith(const BaseTracker<Obj> &tracker, const int mode, const int metric);

protected:
	//Protected constructor no allowing the class to be instantiated
	BaseTracker() {};
	int trackerType;
	int trackerId;

	std::shared_ptr<Obj> objectPTR;

};
#endif // BASETRACKER_HPP
