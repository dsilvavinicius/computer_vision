#ifndef CORRESPONDENCE_H
#define CORRESPONDENCE_H

#include <utility>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace math
{
	using Correspondence = pair< VectorXd, VectorXd >;

	ostream& operator<<( ostream& out, const Correspondence& correspondence );
	ostream& operator<<( ostream& out, const vector<Correspondence>& correspondences );
}
	
#endif