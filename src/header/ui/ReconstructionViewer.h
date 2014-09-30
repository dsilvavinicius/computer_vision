#ifndef RECONSTRUCTION_VIEWER_H
#define RECONSTRUCTION_VIEWER_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <QtGui/QGuiApplication>
#include <Qt3D/QGLCamera>
#include <Qt3D/QGLView>

using namespace std;
using namespace Eigen;

namespace ui
{
	class ReconstructionViewer : public QGLView
	{
	public:
		/** Note: Each sequential pair in lines vector forms a line. */
		ReconstructionViewer( const vector< VectorXd >& points, const vector< VectorXd >& lines ,
							  const QSurfaceFormat &format, QWindow *parent = 0 );

	protected:
		void paintGL( QGLPainter *painter );
		void initializeGL( QGLPainter *painter );
		void mouseMoveEvent( QMouseEvent* ev );
		void mousePressEvent( QMouseEvent* ev );
		void wheelEvent( QWheelEvent * ev );
		
	private:
		QPoint m_lastMousePos;
		shared_ptr< QVector3DArray > m_points;
		/** Each sequential pair in this buffer forms a line. */
		shared_ptr< QVector3DArray > m_lines;
	};

	using ReconstructionViewerPtr = shared_ptr< ReconstructionViewer >;
}

#endif