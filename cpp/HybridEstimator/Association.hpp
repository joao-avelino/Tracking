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

		if (detecPTR == nullptr)
			is_associated = false;
		else
			is_associated = true;


	}

	~Association() {};


private:
	shared_ptr<BaseTracker<Obj> > trackerPTR;
	shared_ptr<Detection<Obj> > detectionPTR;

};

template <class Obj> class AssociationList : Association<Obj>
{
public:

	vector<Association<Obj>> getSuccessfulAssociations();
	vector<shared_ptr<BaseTracker<Obj>>> getUnassociatedTrackers();
	vector<shared_ptr<Detection<Obj>>> getUnassociatedDetections();

	void addSuccessfulAssociation(Association<Obj> assoc);
	void addUnassociatedDetection(shared_ptr<Detection<Obj>> det);

protected:

	vector<Association<Obj>> associationList;
	vector<shared_ptr<Detection<Obj>>> unassociatedDetections;


};
#endif // ASSOCIATION_HPP