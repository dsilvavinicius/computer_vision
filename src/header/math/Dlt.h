#ifndef DLT_H
#define DLT_H

#include <memory>
#include <Eigen/Dense>
#include "Correspondence.h"
#include "DltBase.h"

using namespace std;
using namespace Eigen;

namespace math
{
	/** Implements the Direct Linear Transformation to solve the image panorama problem. */
	class Dlt : public DltBase
	{
	public:
		Dlt( vector< Correspondence > sample );
		
		int scoreSolution( vector< Correspondence > allCorrespondences);
	
	protected:
		MatrixXd createLinearSystem();
	};
}

#endif