#ifndef MULTIOBJECTTRACKER_HPP
#define MULTIOBJECTTRACKER_HPP
#include "Detection.hpp"
#include "BaseMoT.hpp"
#include <vector>
#include <Comparator.hpp>



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


		//Predict
		for (auto& trk : this->trackersVector)
		{
			VectorXd empty = VectorXd();
			trk->predict(empty);
		}


		//Associate
		AssociationList<Obj, Trk> assList;
		assList = associator.associateData(this->trackersVector, detList);

		/*Debug - print AssociateList*/


		//Apply the validation Gate
		/*TODO TODO TODO TODO TODO TODO TODO TODO 
		* It should be a different class, but I have no time for this...
		* TODO TODO TODO TODO TODO TODO TODO TODO
		*/

		vector<Association<Obj, Trk>> succAssVect = assList.getSuccessfulAssociations();
		vector<Association<Obj, Trk>> afterGateAssVect;
		

		for (auto& ass : succAssVect)
		{
			VectorXd detState = ass.getDetectionPTR()->getObjPTR()->getObservableStates();

			MatrixXd trkCov = ass.getTrackerPTR()->getObjPTR()->getObervableCovariance();
			VectorXd trkState = ass.getTrackerPTR()->getObjPTR()->getObservableStates();

			double mahal = Comparator::mahalanobis(detState, trkState, trkCov);

			if(mahal < 9.2103)
			{
				VectorXd colorDet = ass.getDetectionPTR()->getObjPTR()->getBvtHist();
				VectorXd colorTrk = ass.getTrackerPTR()->getObjPTR()->getBvtHist();

				double colorDist = Comparator::euclidean(colorDet, colorTrk);
				if(colorDist < 0.6)
					afterGateAssVect.push_back(ass);

			}


		}


		//Update
		//std::vector<Association<Obj, Trk>> assVect = assList.getSuccessfulAssociations();

		for (auto& ass : afterGateAssVect)
		{
			ass.getTrackerPTR()->update(*ass.getDetectionPTR());
		}

		//Manage trackers
		trackerManager.manageTracks(assList);
	
	}


protected:

    unique_ptr<Trk> trackerToBeClonedPTR;
	Assoc &associator;
    TrkMgr &trackerManager;
};

#endif // !MULTIOBJECTTRACKER_HPP
