#include "MMBankItem.hpp"



MMBankItem::MMBankItem()
{
}


MMBankItem::~MMBankItem()
{
}

VectorXd MMBankItem::getStatePred()
{
	return filter->getStatePred();
}

VectorXd MMBankItem::getStatePost()
{
	return filter->getStatePost();
}

void MMBankItem::predict(VectorXd &control)
{
	this->filter->predict(control);
}

void MMBankItem::update(VectorXd & measure)
{
	this->filter->update(measure);
}
