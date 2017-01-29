#ifndef BASETRACKERMANAGER_HPP
#define BASETRACKERMANAGER_HPP

#include <memory>
#include <vector>
#include "Association.hpp"

using namespace std;

template <class Obj, class Trk> class BaseTrackerManager
{

public:
	BaseTrackerManager() {};
	~BaseTrackerManager() {};

	virtual void manageTracks(AssociationList<Obj, Trk> &assocList) = 0;

protected:
	vector <Trk> &trackerList;

};



#endif // TRACKERMANAGER_HPP
