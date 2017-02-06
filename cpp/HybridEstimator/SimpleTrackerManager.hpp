#ifndef SIMPLETRACKERMANAGER_HPP
#define SIMPLETRACKERMANAGER_HPP

#include "BaseTrackerManager.hpp"

using namespace std;

//Create trackers as they appear and delete after a set number of non associations

template <class Obj, class Trk> class SimpleTrackerManager : public BaseTrackerManager <Obj, Trk>
{
public:
	SimpleTrackerManager(int maxNotAssoc)
	{
		this->maxNotAssoc = maxNotAssoc;
		numOfIds = 0;
	};

	void manageTracks(AssociationList<Obj, Trk> &assocList)
	{
		assert(this->multiObjectTrackerPTR != nullptr && "The tracker manager needs to have a pointer to the Multiple Object Tracker! Please assign it in the constructor");

		for (int& as : notAssocVect)
		{
			as += 1;
		}

		//Reset the counter of associations
		vector<Association<Obj, Trk>> successAssoc = assocList.getSuccessfulAssociations();
        vector<shared_ptr<Trk>> trackers = this->multiObjectTrackerPTR->getTrackersVector();

		/*THIS NEEDS TO BE OPTIMIZED! BUT NOW I NEED RESULTS FOR THE PAPER*/

		for (Association<Obj, Trk>& assoc : successAssoc)
		{

			shared_ptr<Trk> trkPTR = assoc.getTrackerPTR();
			ptrdiff_t pos = find(trackers.begin(), trackers.end(), trkPTR) - trackers.begin();
			assert(pos < trackers.size() && "That tracker does no longer exist... or isnt on the notAssocVect");
			notAssocVect.at(pos) = 0;

		}


		//Delete the trackers that need to be deleted
		int vectSize = notAssocVect.size();
		for (int i = 0; i < vectSize; i++)
		{
			if (notAssocVect.at(i) >= maxNotAssoc)
			{
                this->multiObjectTrackerPTR->deleteTracker(i);
				notAssocVect.erase(notAssocVect.begin()+i);
				i--;
				vectSize = notAssocVect.size();
			}
			
		}

		//Create new tracks for each unassociated detection
		vector<shared_ptr<Detection<Obj>>> unassocDetsList = assocList.getUnassociatedDetections();

		for (auto& unassoc : unassocDetsList)
		{
			
            this->multiObjectTrackerPTR->createTracker(*unassoc, numOfIds);
			notAssocVect.push_back(0);
			numOfIds++;
		}

	}

protected:
	vector<int> notAssocVect;

	int maxNotAssoc;
	int numOfIds;

};

#endif // SIMPLETRACKERMANAGER_HPP
