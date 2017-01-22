#ifndef PERSON3DBVT_HPP
#define PERSON3DBVT_HPP

#include "Object.hpp"
#include <eigen3/Eigen/Core>

using namespace Eigen;

class Person3dBVT :
	public Object
{
public:
	Person3dBVT(VectorXd position, VectorXd bvtHist, MatrixXd positionErrorCovariance);
	~Person3dBVT();

	void setPosition(VectorXd position);
	void setBvtHist(VectorXd bvtHist);
	void setPositionErrorCovariance(MatrixXd posErrorCov);

	VectorXd getPosition();
	VectorXd getBvtHist();
	MatrixXd getPositionErrorCovariance();

protected:
	VectorXd position;
	VectorXd bvtHist;
	MatrixXd positionErrorCovariance;
};


#endif //PERSON3DBVT
