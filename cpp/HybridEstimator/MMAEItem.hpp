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

	void predict(VectorXd &control = VectorXd());
	void update(VectorXd &measurement = VectorXd());

	void computeProbabilityDensity(VectorXd &measurement=VectorXd());
	double getProbDensity();
	double getProbability();
	void setProbabiliy(double prob);
	double getStateDim();

protected:
	double probDensity;
	double probability;
	double stateDim;


};

#endif //MMAEItem

