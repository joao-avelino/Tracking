#ifndef PEOPLETRACKERWITHMMAE_H
#define PEOPLETRACKERWITHMMAE_H


#include "BaseBayesianTracker.hpp"

template <class Obj> class PeopleTrackerWithMMAE :
	public BaseBayesianTracker<Obj>
{
public:
	PeopleTrackerWithMMAE(std::shared_ptr<Obj> objectPTR) 
	{
		this->objectPTR = objectPTR;
		this->trackerType = objectPTR->getObjectType();
	};
	~PeopleTrackerWithMMAE() {};

	void preProcessingComputations() {};
	void predict(const MatrixXd &controlVect) {};
	void postPredictComputations() {};
	void update(const Detection<Obj> &det) {};
	void postUpdateComputations() {};
	
	
};



#endif // !PEOPLETRACKERWITHMMAE_H
