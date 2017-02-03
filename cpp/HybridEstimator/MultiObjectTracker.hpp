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
		
		shared_ptr<Trk> newTracker = static_pointer_cast<Trk>(trackerToBeClonedPTR->clone(detection.getObjPTR()));

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


	//Process detections - This is the "main" rotine
	void processDetections(vector<shared_ptr<Detection<Obj>>> &detList)
	{
		cout << "--- Before predict ---" << endl;

		for (auto& trk : this->trackersVector)
		{
			cout << "Obj: " << trk->getObjPTR()->getObservableStates() << endl;
			cout << "Estimator pred: " << trk->positionEstimator->getStatePred() << endl;
			cout << "Estimator post: " << trk->positionEstimator->getStatePost() << endl;
		}



	
		//Predict
		for (auto& trk : this->trackersVector)
		{
			VectorXd empty = VectorXd();
			trk->predict(empty);
		}

		cout << "--- After predict ---" << endl;

		for (auto& trk : this->trackersVector)
		{
			cout << "Tracker: " << trk->getObjPTR()->getObservableStates() << endl;
			cout << "Estimator pred: " << trk->positionEstimator->getStatePred() << endl;
			cout << "Estimator post: " << trk->positionEstimator->getStatePost() << endl;
		}


		//Associate
		AssociationList<Obj, Trk> assList;
		assList = associator.associateData(this->trackersVector, detList);

		/*Debug - print AssociateList*/

		cout << "Associations" << endl;
		for (auto& ass : assList.getSuccessfulAssociations())
		{
			cout << ass.getTrackerPTR()->getObjPTR()->getObservableStates() << endl;
			cout << "with" << endl;
			cout << ass.getDetectionPTR()->getObjPTR()->getObservableStates() << endl;

		}


		//Apply the validation Gate
		/*TODO TODO TODO TODO TODO TODO TODO TODO 
		* TODO TODO TODO TODO TODO TODO TODO TODO 
		* TODO TODO TODO TODO TODO TODO TODO TODO
		*/


		//Update
		std::vector<Association<Obj, Trk>> assVect = assList.getSuccessfulAssociations();

		for (auto& ass : assVect)
		{
			ass.getTrackerPTR()->update(*ass.getDetectionPTR());
		}

		//Manage trackers
		trackerManager.manageTracks(assList);

		cout << "Created Trackers" << endl;
		for (auto& trk : this->trackersVector)
		{
			cout << "Obj: " << trk->getObjPTR()->getObservableStates() << endl;
			cout << "Estimator pred: " << trk->positionEstimator->getStatePred() << endl;
			cout << "Estimator post: " << trk->positionEstimator->getStatePost() << endl;
		}

	
	}


protected:

    unique_ptr<Trk> trackerToBeClonedPTR;
	Assoc &associator;
    TrkMgr &trackerManager;
};

#endif // !MULTIOBJECTTRACKER_HPP
