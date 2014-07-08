#include <iostream>
#include "math/header/ProjectionRectificator.h"

namespace math
{
	ProjectionRectificator::ProjectionRectificator(vector<pair<VectorXd, VectorXd>> correlations)
	{
		buildTransformation(correlations);
	}

	shared_ptr<MatrixXd> ProjectionRectificator::buildTransformation(vector<pair<VectorXd, VectorXd>> correlations)
	{
		MatrixXd A(8, 8);
		VectorXd b(8);
		for (int i = 0; i < 4; ++i)
		{
			VectorXd from = correlations[i].first;
			VectorXd to = correlations[i].second;
			
			Block<MatrixXd> block = A.block(i*2, 0, 2, 8);
			
			// The two matrix lines associates with the current point correlation.
			block << 	from[0], from[1],	from[2], 0.,		from[1] * to[0], 0, 		from[0] * to[0], 0.,
						0,		 0,			0,		 from[0], 	from[1],		 from[2],	from[0] * to[1], from[1] * to[1];
			
			b[i * 2]	= from[2] * to[0];
			b[i*2 + 1]	= from[2] * to[1];
		}
		cout 	<< "Linear system A matrix: " << endl << A << endl
				<< "Linear system b vector: " << endl << b << endl;
		
		VectorXd x = A.colPivHouseholderQr().solve(b);		
		cout << "Linear system x vector: " << endl << x << endl;
		
		m_transformation = make_shared<MatrixXd>(8, 8);
		(*m_transformation) << x;
		cout << "Rectification transformation: " << endl << (*m_transformation) << endl;
		
		return m_transformation;
	}

	shared_ptr<MatrixXd> ProjectionRectificator::getTransformation()
	{
		return m_transformation;
	}
}