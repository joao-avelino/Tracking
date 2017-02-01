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
			std::cout << "Pos[ " << pos << "]" << std::endl;
			notAssocVect.at(pos) = 0;

		}


		cout << "---------check if delete------------" << endl;
		//Delete the trackers that need to be deleted
		int vectSize = notAssocVect.size();
		for (int i = 0; i < vectSize; i++)
		{
			cout << "i: " << i << endl;
			if (notAssocVect.at(i) >= maxNotAssoc)
			{
                cout << "deleting: " << this->multiObjectTrackerPTR->getTrackersVector().at(i)->getObjPTR()->getPosition();
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
            this->multiObjectTrackerPTR->createTracker(*unassoc);
			notAssocVect.push_back(0);
		}

		/*DEBUG*/

		for (auto& lol : notAssocVect)
		{

			cout << "notAssocVect: " << lol << endl;
		}

		cout << "----------------" << endl;

	}

protected:
	vector<int> notAssocVect;

	int maxNotAssoc;

};

#endif // SIMPLETRACKERMANAGER_HPP
