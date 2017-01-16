#ifndef MMAEItem_HPP
#define MMAEItem_HPP

#include "MMBankItem.hpp"
#include <eigen3/Eigen/Core>


class MMAEItem :
	MMBankItem
{
public:
	MMAEItem(std::shared_ptr<BaseBayesianFilter> model, std::string modelName);
	~MMAEItem();

	void computeProbabilityDensity(VectorXd &measurement=VectorXd());

	double getProbDensity();

protected:
	double probDensity;


};

#endif //MMAEItem

