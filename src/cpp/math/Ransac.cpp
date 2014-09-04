#include "Ransac.h"

#include <stdlib.h>
#include <time.h>
#include <algorithm>

namespace math
{	
	template< typename T>
	RansacIterTable Ransac< T >::m_iterTable = Ransac< T >::genIterTable();
	
	template< typename T>
	Ransac< T >::Ransac( vector< T > set, VectorXd ( evaluator )( vector< T >, double& ), int nElementsPerSample,
					double epsilon ) :
		m_set( set ),
		m_evaluator( evaluator ),
		m_nElementsPerSample( nElementsPerSample ),
		m_epsilon( epsilon )
	{
		m_nIter = Ransac::m_iterTable[ RansacIterTableKey( m_nElementsPerSample, m_epsilon ) ];
	}
	
	template< typename T>
	VectorXd Ransac< T >::compute()
	{
		VectorXd bestSol;
		for( int iter = 0; iter < m_nIter; ++iter ) 
		{
			vector< T > sample;
			srand( time( NULL ) );
			
			for( int i = 0; i < m_nElementsPerSample; ++i )
			{
				sample.push_back( m_set[rand() % m_set.size()] );
			}
			
			double newEpsilon;
			VectorXd systemSol = m_evaluator( sample, newEpsilon );
			
			if( newEpsilon < m_epsilon )
			{
				// Round epsilon to 2 decimal places.
				m_epsilon = round( newEpsilon * 100 ) / 100;
				m_nIter = Ransac::m_iterTable[ RansacIterTableKey( m_nElementsPerSample, m_epsilon) ];
				bestSol = systemSol;
			}
		}
		
		return bestSol;
	}
	
	template< typename T>
	RansacIterTable Ransac< T >::genIterTable()
	{
		RansacIterTable table;
		
		table[ RansacIterTableKey( 3, 0.05f ) ] = 3;
		table[ RansacIterTableKey( 3, 0.10f ) ] = 4;
		table[ RansacIterTableKey( 3, 0.20f ) ] = 7;
		table[ RansacIterTableKey( 3, 0.25f ) ] = 9;
		table[ RansacIterTableKey( 3, 0.30f ) ] = 11;
		table[ RansacIterTableKey( 3, 0.40f ) ] = 19;
		table[ RansacIterTableKey( 3, 0.50f ) ] = 50;
		
		table[ RansacIterTableKey( 4, 0.05f ) ] = 3;
		table[ RansacIterTableKey( 4, 0.10f ) ] = 5;
		table[ RansacIterTableKey( 4, 0.20f ) ] = 9;
		table[ RansacIterTableKey( 4, 0.25f ) ] = 13;
		table[ RansacIterTableKey( 4, 0.30f ) ] = 17;
		table[ RansacIterTableKey( 4, 0.40f ) ] = 34;
		table[ RansacIterTableKey( 4, 0.50f ) ] = 72;
		
		table[ RansacIterTableKey( 5, 0.05f ) ] = 4;
		table[ RansacIterTableKey( 5, 0.10f ) ] = 6;
		table[ RansacIterTableKey( 5, 0.20f ) ] = 12;
		table[ RansacIterTableKey( 5, 0.25f ) ] = 17;
		table[ RansacIterTableKey( 5, 0.30f ) ] = 26;
		table[ RansacIterTableKey( 5, 0.40f ) ] = 57;
		table[ RansacIterTableKey( 5, 0.50f ) ] = 146;
		
		table[ RansacIterTableKey( 6, 0.05f ) ] = 4;
		table[ RansacIterTableKey( 6, 0.10f ) ] = 7;
		table[ RansacIterTableKey( 6, 0.20f ) ] = 16;
		table[ RansacIterTableKey( 6, 0.25f ) ] = 24;
		table[ RansacIterTableKey( 6, 0.30f ) ] = 37;
		table[ RansacIterTableKey( 6, 0.40f ) ] = 97;
		table[ RansacIterTableKey( 6, 0.50f ) ] = 293;
		
		table[ RansacIterTableKey( 7, 0.05f ) ] = 4;
		table[ RansacIterTableKey( 7, 0.10f ) ] = 8;
		table[ RansacIterTableKey( 7, 0.20f ) ] = 20;
		table[ RansacIterTableKey( 7, 0.25f ) ] = 33;
		table[ RansacIterTableKey( 7, 0.30f ) ] = 54;
		table[ RansacIterTableKey( 7, 0.40f ) ] = 163;
		table[ RansacIterTableKey( 7, 0.50f ) ] = 588;
		
		table[ RansacIterTableKey( 8, 0.05f ) ] = 5;
		table[ RansacIterTableKey( 8, 0.10f ) ] = 9;
		table[ RansacIterTableKey( 8, 0.20f ) ] = 26;
		table[ RansacIterTableKey( 8, 0.25f ) ] = 44;
		table[ RansacIterTableKey( 8, 0.30f ) ] = 78;
		table[ RansacIterTableKey( 8, 0.40f ) ] = 272;
		table[ RansacIterTableKey( 8, 0.50f ) ] = 1177;
		
		return table;
	}
}