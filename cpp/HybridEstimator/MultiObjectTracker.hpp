#ifndef MULTIOBJECTTRACKER_HPP
#define MULTIOBJECTTRACKER_HPP
#include "Detection.hpp"
#include "BaseMoT.hpp"
#include <vector>



using namespace std;


template <class Trk, class TrkMgr, class Assoc, class Obj>
class MultiObjectTracker : public BaseMot<Obj, Trk>
{
public:

	MultiObjectTracker(TrkMgr &trackerManager, Assoc &associator, Trk *trackerToBeClonedPTR) : trackerToBeClonedPTR(trackerToBeClonedPTR),
		associator(associator), trackerManager(trackerManager)
	{
	
		//Assign the tracker pointer so that the tracker manager can add and delete tracks
		//trackerManager.assignMoTPTR(this);
	
	};


	/*These are the necessary methods for the Tracker Manager to work with*/

	void createTracker(Detection<Obj> detection)
	{
		
		shared_ptr<Trk> newTracker = static_pointer_cast<Trk>(trackerToBeClonedPTR->clone());
		newTracker->setObjPTR(detection.getObjPTR());
		trackersVector.push_back(std::move(newTracker));

	}
	

	void deleteTracker(int pos)
	{
		trackersVector.erase(trackersVector.begin()+pos);
	}


	vector<std::shared_ptr<Trk>> getTrackersVector()
	{
		return trackersVector;
	}

protected:
	TrkMgr &trackerManager;
	Assoc &associator;
	unique_ptr<Trk> trackerToBeClonedPTR; //

};

#endif // !MULTIOBJECTTRACKER_HPP
