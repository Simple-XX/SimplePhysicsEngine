#ifndef DTKOBJECT_H
#define DTKOBJECT_H
#include <Eigen\Dense>

class dtkObject
{
protected:
	Eigen::Vector2f center;
public:
	virtual void Init() = 0;
};

#endif
