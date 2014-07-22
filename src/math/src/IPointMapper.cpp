#include "IPointMapper.h"

namespace math
{
	shared_ptr<MatrixXd> IPointMapper::getTransformation()
	{
		return m_transformation;
	}
	
	vector<pair<Vector3d, Vector3d>> IPointMapper::toVector3d(const vector<pair<VectorXd, VectorXd>>& correlations)
	{
		VectorXd templ0 = correlations[0].first;
		VectorXd templ1 = correlations[0].second;
		VectorXd templ2 = correlations[1].first;
		VectorXd templ3 = correlations[1].second;
		
		Vector3d l0(templ0[0], templ0[1], templ0[2]);
		Vector3d l1(templ1[0], templ1[1], templ1[2]);
		Vector3d l2(templ2[0], templ2[1], templ2[2]);
		Vector3d l3(templ3[0], templ3[1], templ3[2]);
		
		vector<pair<Vector3d, Vector3d>> vector3dPairs(2);
		vector3dPairs[0] = pair<Vector3d, Vector3d>(l0, l1);
		vector3dPairs[1] = pair<Vector3d, Vector3d>(l2, l3);
		
		return vector3dPairs;
	}
}