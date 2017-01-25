#ifndef BASEBAYESIANTRACKER_HPP
#define BASEBAYESIANTRACKER_HPP

#include "BaseTracker.hpp"

template <class Obj> class BaseBayesianTracker : BaseTracker<Obj>
{

public:
	BaseBayesianTracker<Obj> (std::shared_ptr<Obj> objectPTR);

	virtual void preProcessingComputations() = 0;
	virtual void predict(const MatrixXd &controlVect) = 0;
	virtual void postPredictComputations() = 0;
	virtual void update(const Detection<Obj> &det) = 0;
	virtual void postUpdateComputations() = 0;

};

#endif // BAYESIANTRACKER_HPP


