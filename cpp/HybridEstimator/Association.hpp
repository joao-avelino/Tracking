#ifndef ASSOCIATION_HPP
#define ASSOCIATION_HPP

#include <memory>
#include <vector>

template <class Track, class Detect> class Association
{
public:
	Association(std::shared_ptr<Track> trackPTR, std::shared_ptr<Detect> detecPTR, std::vector<double> metricsList)
	{
		this->trackerPTR = trackPTR;
		this->detectionPTR = detecPTR;

		if (detecPTR == nullptr)
			is_associated = false;
		else
			is_associated = true;


	}

	~Association() {};

	bool is_associated()
	{
		return is_associated;
	}

	std::vector<double> metricsList;

private:
	std::shared_ptr<Track> trackerPTR;
	std::shared_ptr<Detect> detectionPTR;
	bool is_associated;

};



#endif // ASSOCIATION_HPP