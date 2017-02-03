#ifndef TRACKERWITHBVT_H
#define TRACKERWITHBVT_H


#include "BaseBayesianTracker.hpp"

template <class Obj, class PosEstim> class TrackerWithBVT :
	public BaseBayesianTracker<Obj>
{
public:

	std::shared_ptr<PosEstim> positionEstimator;


	/****************************************Fail**************************************************************/
	TrackerWithBVT(std::shared_ptr<Obj> objectPTR, std::shared_ptr<PosEstim> positionEstimator,
		double colorLeaningRate) : colorLearningRate(colorLeaningRate)
	{

		cout << "Creating a new tracker" << endl;

		this->positionEstimator = positionEstimator;
		cout << "Estimator pred: " << this->positionEstimator->getStatePred() << endl;
		cout << "Estimator post: " << this->positionEstimator->getStatePost() << endl;

		this->objectPTR = objectPTR;
		cout << "Object: " << objectPTR->getObservableStates() << endl;


		/*FAIL*/

		obsSize = objectPTR->getObservableStates().size();
		this->trackerType = objectPTR->getObjectType();

		cout << "-- Tracker created -- " << endl;
	};

	/************************************************************************************************************/

	~TrackerWithBVT() {}

	void preProcessingComputations()
	{
	};

	void predict(VectorXd &controlVect)
	{
		positionEstimator->predict(controlVect);
		VectorXd state = positionEstimator->getStatePred();
		VectorXd obsStates = state.head(obsSize);
        this->objectPTR->setObservableStates(obsStates);
		MatrixXd stateCov = positionEstimator->getCovPred();
        this->objectPTR->setObservableCovariance(stateCov.block(0, 0, obsSize, obsSize));

		//Predict the colors somehow?

	};
	void postPredictComputations() {};

	void update(Detection<Obj> &det)
	{
		VectorXd detPosition = det.getObjPTR()->getObservableStates();
		MatrixXd detCov = det.getObjPTR()->getObervableCovariance();

		if(detCov.size() > 0)
			positionEstimator->update(detPosition, detCov);
		else
			positionEstimator->update(detPosition);
		
		VectorXd state = positionEstimator->getStatePost();
        this->objectPTR->setObservableStates(state.head(obsSize));
		MatrixXd stateCov = positionEstimator->getCovPost();
        this->objectPTR->setObservableCovariance(stateCov.block(0, 0, obsSize, obsSize));
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

	shared_ptr<BaseTracker<Obj>> clone(shared_ptr<Obj> objecto)
	{

		assert(objecto->getObservableStates().size() > 1 && "Object provided has zero size (during cloning of tracker)");
		assert(objecto->getObservableStates().size() == positionEstimator->getObsDim() && "The measurement dimensions are different from the observable states dimensions");
		
			

		int stateDim = this->positionEstimator->getStatePost().size();
		VectorXd initState = VectorXd::Zero(stateDim);
		int obsDim = objecto->getObservableStates().size();
		initState.head(obsDim) = objecto->getObservableStates();


		shared_ptr<PosEstim> estimClone;

		if (objecto->getObervableCovariance().size() < 1)
		{
			estimClone = static_pointer_cast<PosEstim>(this->positionEstimator->clone(initState));
		}
		else {
			estimClone = static_pointer_cast<PosEstim>(this->positionEstimator->clone(initState, objecto->getObervableCovariance()));
		}


		cout << "New estimator" << endl;
		cout << "Estimator pred: " << estimClone->getStatePred() << endl;
		cout << "Estimator post: " << estimClone->getStatePost() << endl;


		return shared_ptr<BaseTracker<Obj>>(new TrackerWithBVT<Obj, PosEstim>(objecto, estimClone,
			this->colorLearningRate));

	}
	
private:
	int obsSize;
	double colorLearningRate;
};



#endif // !TRACKERWITHBVT_H
