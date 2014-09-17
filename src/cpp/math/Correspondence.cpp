#include "Correspondence.h"

namespace math
{
	ostream& operator<<( ostream& out, const Correspondence& correspondence )
	{
		out << "Correspondence: " << endl << correspondence.first << endl << ", " << endl << correspondence.second << endl;
		return out;
	}

	ostream& operator<<( ostream& out, const vector< Correspondence >& correspondences )
	{
		for( Correspondence correspondence : correspondences )
		{
			out << correspondence << endl;
		}
		return out;
	}
}