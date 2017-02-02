#ifndef TRACKERWITHBVT_H
#define TRACKERWITHBVT_H


#include "BaseBayesianTracker.hpp"

template <class Obj, class PosEstim> class TrackerWithBVT :
	public BaseBayesianTracker<Obj>
{
public:

	std::shared_ptr<PosEstim> positionEstimator;

	TrackerWithBVT(std::shared_ptr<Obj> objectPTR, std::shared_ptr<PosEstim> positionEstimator,
		double colorLeaningRate) : positionEstimator(positionEstimator), colorLearningRate(colorLeaningRate)
	{
		this->objectPTR = objectPTR;
		obsSize = objectPTR->getPosition().size();
		this->trackerType = objectPTR->getObjectType();
	};

	~TrackerWithBVT() {}

	void preProcessingComputations()
	{
	};

	void predict(VectorXd &controlVect)
	{
		positionEstimator->predict(controlVect);
		VectorXd state = positionEstimator->getStatePred();
        this->objectPTR->setPosition(state.head(obsSize));
		MatrixXd stateCov = positionEstimator->getCovPred();
        this->objectPTR->setPositionErrorCovariance(stateCov.block(0, 0, obsSize, obsSize));

		//Predict the colors somehow?

	};
	void postPredictComputations() {};

	void update(Detection<Obj> &det)
	{
		VectorXd detPosition = det.getObjPTR()->getPosition();
		positionEstimator->update(detPosition);
		VectorXd state = positionEstimator->getStatePost();
        this->objectPTR->setPosition(state.head(obsSize));
		MatrixXd stateCov = positionEstimator->getCovPost();
        this->objectPTR->setPositionErrorCovariance(stateCov.block(0, 0, obsSize, obsSize));
        this->objectPTR->setBvtHist(this->objectPTR->getBvtHist()*(1.0 - colorLearningRate) + colorLearningRate*det.getObjPTR()->getBvtHist());

	};

	void postUpdateComputations() {};

	shared_ptr<BaseTracker<Obj>> clone()
	{
		shared_ptr<PosEstim> estimClone = static_pointer_cast<PosEstim>(this->positionEstimator->clone());
		shared_ptr<Obj> objClone = static_pointer_cast<Obj>(this->objectPTR->clone());

		
		return shared_ptr<BaseTracker<Obj>>(new TrackerWithBVT<Obj, PosEstim>(objClone, estimClone,
			this->colorLearningRate));

	}
	
private:
	int obsSize;
	double colorLearningRate;
};



#endif // !TRACKERWITHBVT_H
