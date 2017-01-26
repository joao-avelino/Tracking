#ifndef ASSOCIATION_HPP
#define ASSOCIATION_HPP

#include <memory>
#include <vector>
#include "BaseTracker.hpp"
#include "Detection.hpp"

using namespace std;

template <class Obj> class Association
{
public:
	Association(shared_ptr<BaseTracker<Obj> > trackPTR, shared_ptr<Detection<Obj> > detecPTR)
	{
		this->trackerPTR = trackPTR;
		this->detectionPTR = detecPTR;
	};

	~Association() {};

	shared_ptr<BaseTracker<Obj> > getTrackerPTR()
	{
		return trackerPTR;
	};

	shared_ptr<Detection<Obj>> getDetectionPTR()
	{
		return detectionPTR;
	};


private:
	shared_ptr<BaseTracker<Obj> > trackerPTR;
	shared_ptr<Detection<Obj> > detectionPTR;

};

template <class Obj> class AssociationList
{
public:

	AssociationList() {};

	vector<Association<Obj>> getSuccessfulAssociations()
	{
		return associationList;
	}


	vector<shared_ptr<BaseTracker<Obj>>> getUnassociatedTrackers()
	{
		return unassociatedDetections;
	}


	vector<shared_ptr<Detection<Obj>>> getUnassociatedDetections()
	{

		return unassociatedDetections;
	}

	void addSuccessfulAssociation(Association<Obj> assoc) 
	{
		this->associationList.push_back(assoc);
	};



	void addUnassociatedDetection(shared_ptr<Detection<Obj>> det) 
	{
		this->unassociatedDetections.push_back(det);
	};

protected:

	vector<Association<Obj>> associationList;
	vector<shared_ptr<Detection<Obj>>> unassociatedDetections;


};
#endif // ASSOCIATION_HPP

