#ifndef MMAEItem_HPP
#define MMAEItem_HPP

#include "MMBankItem.hpp"
#include <eigen3/Eigen/Core>


class MMAEItem :
	public MMBankItem
{
public:
	MMAEItem(std::shared_ptr<BaseBayesianFilter> model, std::string modelName);
	~MMAEItem();

    void predict(VectorXd &control);
    void update(VectorXd &measurement);
	void update(VectorXd &measurement, MatrixXd &measurementCov);

    void predict()
    {
        predict(EMPTYVEC);
    }

    void update()
    {
        update(EMPTYVEC);
    }

    void computeProbabilityDensity(VectorXd &measurement);

    void computeProbabilityDensity()
    {
        computeProbabilityDensity(EMPTYVEC);
    }

	double getProbDensity();
	double getProbability();
	void setProbabiliy(double prob);
	int getStateDim();

	std::shared_ptr<MMBankItem> clone();
	std::shared_ptr<MMBankItem> clone(VectorXd initial_state);
	std::shared_ptr<MMBankItem> clone(VectorXd initial_state, MatrixXd measurementCov);

	int getObsDim()
	{
		return filter->getObsDim();
	}

protected:
	double probDensity;
	double probability;
	int stateDim;


};

#endif //MMAEItem

