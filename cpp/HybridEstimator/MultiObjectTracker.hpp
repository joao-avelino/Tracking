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
		trackerManager.assignMoTPTR(this);
	
	};


	/*These are the necessary methods for the Tracker Manager to work with*/

	void createTracker(Detection<Obj> detection, int id)
	{
		
		shared_ptr<Trk> newTracker = static_pointer_cast<Trk>(trackerToBeClonedPTR->clone());
		newTracker->setObjPTR(detection.getObjPTR());
		newTracker->setTrackerId(id);
		this->trackersVector.push_back(std::move(newTracker));

	}
	

	void deleteTracker(int pos)
	{
        this->trackersVector.erase(this->trackersVector.begin()+pos);
	}


	vector<std::shared_ptr<Trk>> getTrackersVector()
	{
        return this->trackersVector;
	}


	//Process detections - This is the "main" routine
	void processDetections(vector<shared_ptr<Detection<Obj>>> &detList)
	{
	
		//Predict
		for (auto& trk : this->trackersVector)
		{
			VectorXd empty = VectorXd();
			trk->predict(empty);
		}

		//Associate
		AssociationList<Obj, Trk> assList;
		assList = associator.associateData(this->trackersVector, detList);


		//Apply the validation Gate



		//Update
		std::vector<Association<Obj, Trk>> assVect = assList->getSuccessfulAssociations();

		for (auto& ass : assVect)
		{
			ass.getTrackerPTR()->
		}

		//Manage

	
	}


protected:
	TrkMgr &trackerManager;
    unique_ptr<Trk> trackerToBeClonedPTR;
	Assoc &associator;

};

#endif // !MULTIOBJECTTRACKER_HPP
