#include "MMBankItem.hpp"



MMBankItem::MMBankItem()
{
}


MMBankItem::~MMBankItem()
{
}

void MMBankItem::predict(VectorXd &control)
{
	this->filter->predict(control);
}

void MMBankItem::update(VectorXd & measure)
{
	this->filter->update(measure);
}
