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

	MultiObjectTracker(TrkMgr &trackerManager, Assoc &associator, Trk *trackerToBeClonedPTR, double rec_thr) : trackerToBeClonedPTR(trackerToBeClonedPTR),
		associator(associator), trackerManager(trackerManager), rec_thr(rec_thr)
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


		/*Get the normalized distances*/

		//Get maximum distance
		double normalizingFactor = 0.001;
		for (auto& trk : this->trackersVector)
		{
			for (auto& det : detList)
			{
				double dist = trk->compareWith((*det), Person3dBVT::COMP_POSITION, Comparator::METRIC_EUCLIDEAN);
				if (dist > normalizingFactor)
					normalizingFactor = dist;
			}
		}

		//Set the normalized observable state - THIS HAS TO CHANGE!
		for (auto& trk : this->trackersVector)
		{
			VectorXd pos = trk->getObjPTR()->getObservableStates();
			trk->getObjPTR()->setNormalizedPosition(pos.head(2) / normalizingFactor);
		}

		for (auto& det : detList)
		{
			VectorXd pos = det->getObjPTR()->getObservableStates();
			det->getObjPTR()->setNormalizedPosition(pos.head(2) / normalizingFactor);
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

				double colorDist = Comparator::hellinger(colorDet, colorTrk);
				if(colorDist < rec_thr)
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
	double rec_thr;
};

#endif // !MULTIOBJECTTRACKER_HPP
