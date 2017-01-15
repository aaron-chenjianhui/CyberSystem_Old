#ifndef KineCal_H
#define KineCal_H

#include "kine7.hpp"

class KineCal{
	public:
		KineCal();
		~KineCal();



	private:
		rpp::kine::Kine7<double> m_Kine;
		rpp::kine::SingularityHandler<double> sh;
		rpp::kine::Kine7<double>::angular_interval_vector joint_limits;
	};




#endif