#ifndef MMBANKITEM_HPP
#define MMBANKITEM_HPP

#include "BaseBayesianFilter.hpp"
#include <eigen3/Eigen/Core>

using namespace Eigen;

class MMBankItem
{
public:
	MMBankItem();
	~MMBankItem();
	virtual VectorXd getStateEstimate() = 0;
	void predict(VectorXd &control=VectorXd());
	void update(VectorXd &measure = VectorXd());

protected:
	std::shared_ptr<BaseBayesianFilter> filter;
	

};

#endif // MMBANKITEM_HPP
