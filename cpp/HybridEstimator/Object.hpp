#ifndef OBJECT_HPP
#define OBJECT_HPP

class Object
{
public:
	Object() {};
	~Object() {};

	//Object type consts
	const int PERSON = 1;
	const int TORSO = 2;
	const int FACE = 3;
	const int HEAD_AND_SHOULDERS = 4;
	const int LEGS = 5;
	const int BALL = 6;
	const int CYLINDER = 7;
	const int CUBE = 8;
	const int MARKER = 7;
	

	//Mode can be defined in derived classes
	virtual double compareWith(const Object &otherObject, const int mode) = 0;

	int getObjectType()
	{
		return objectType;
	};

protected:

	int objectType;

};

#endif //OBJECT_HPP