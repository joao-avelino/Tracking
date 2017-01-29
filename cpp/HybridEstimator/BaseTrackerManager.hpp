#ifndef BASETRACKERMANAGER_HPP
#define BASETRACKERMANAGER_HPP

#include <memory>
#include <vector>
#include "Association.hpp"

using namespace std;

template <class Obj, class Trk, class MoT> class BaseTrackerManager
{

public:
	BaseTrackerManager() {};
	~BaseTrackerManager() {};

	virtual void manageTracks(AssociationList<Obj, Trk> &assocList) = 0;

protected:
	MoT& multiObjectTracker;

};



#endif // TRACKERMANAGER_HPP
