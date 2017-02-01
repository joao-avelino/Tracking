#ifndef BASETRACKERMANAGER_HPP
#define BASETRACKERMANAGER_HPP

#include <memory>
#include <vector>
#include "Association.hpp"
#include "BaseMoT.hpp"

using namespace std;

template <class Obj, class Trk> class BaseTrackerManager
{

public:
	BaseTrackerManager() {};
	~BaseTrackerManager() {};

	virtual void manageTracks(AssociationList<Obj, Trk> &assocList) = 0;
	void assignMoTPTR(BaseMot<Obj, Trk> *trackerPTR)
	{
		this->multiObjectTrackerPTR = trackerPTR;
	};

protected:
	BaseMot<Obj, Trk> *multiObjectTrackerPTR;

};



#endif // TRACKERMANAGER_HPP
