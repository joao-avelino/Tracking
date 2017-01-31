#ifndef BASEMOT_HPP
#define BASEMOT_HPP

#include <vector>

template <class Obj, class Trk>
class BaseMot
{
public:
	BaseMot() {};

	virtual void createTracker(Detection<Obj> detection) = 0;
	virtual void deleteTracker(int pos) = 0;
	virtual vector<std::shared_ptr<Trk>> getTrackersVector() = 0;

protected:
	std::vector<std::shared_ptr<Trk>> trackersVector;

};

#endif // !BASEMOT_HPP
