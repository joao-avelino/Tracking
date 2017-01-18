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
	VectorXd getStatePred();
	VectorXd getStatePost();
	MatrixXd getCovPred();
	MatrixXd getCovPost();

	void predict(VectorXd &control=VectorXd());
	void update(VectorXd &measure = VectorXd());

	std::string getModelName();


protected:
	std::shared_ptr<BaseBayesianFilter> filter;
	

};

#endif // MMBANKITEM_HPP
