#ifndef PEOPLETRACKERWITHMMAE_H
#define PEOPLETRACKERWITHMMAE_H


#include "BaseBayesianTracker.hpp"

template <class Obj> class PeopleTrackerWithMMAE :
	public BaseBayesianTracker<Obj>
{
public:
	PeopleTrackerWithMMAE(std::shared_ptr<Obj> objectPTR) : BaseBayesianTracker<Obj> (objectPTR) {};
	~PeopleTrackerWithMMAE() {};

	void preProcessingComputations() {};
	void predict(const MatrixXd &controlVect) {};
	void postPredictComputations() {};
	void update(const Detection<Obj> &det) {};
	virtual void postUpdateComputations() {};
	

};



#endif // !PEOPLETRACKERWITHMMAE_H
