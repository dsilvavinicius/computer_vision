#include "ReconstructionViewer.h"
#include <QMouseEvent>

namespace ui
{
	ReconstructionViewer::ReconstructionViewer( const vector< VectorXd >& points, const vector< VectorXd >& lines,
												const QSurfaceFormat &format, QWindow *parent )
		: QGLView(format, parent)
	{
		m_lines = make_shared< QVector3DArray >();
		for( VectorXd point : lines )
		{
			m_lines->append( point[0], point[1], point[2] );
		}
		
		m_points = make_shared< QVector3DArray >();
		for( VectorXd point : points )
		{
			m_points->append( point[0], point[1], point[2] );
		}
	}

	void ReconstructionViewer::initializeGL( QGLPainter *painter )
	{	
		QGLCamera* cam = camera();
		cam->setProjectionType( QGLCamera::Perspective );
		cam->setFieldOfView( 60.0f );
		cam->setNearPlane( 0.1f );
		cam->setFarPlane( 10000.0f );
		
		painter->setCamera( cam );
	}

	void ReconstructionViewer::paintGL(QGLPainter *painter)
	{
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		
		painter->setStandardEffect( QGL::FlatColor );
		painter->setColor( QColor( 255, 255, 255 ) );
		painter->clearAttributes();
		painter->setVertexAttribute( QGL::Position, *m_lines );
		painter->draw( QGL::Lines, m_lines->size() );
		
		painter->clearAttributes();
		painter->setVertexAttribute( QGL::Position, *m_points );
		painter->draw( QGL::Points, m_points->size() );
	}

	void ReconstructionViewer::mouseMoveEvent( QMouseEvent * ev )
	{
		QGLCamera* cam = camera();
		Qt::MouseButtons buttons = ev->buttons();
		
		if ( buttons & ( Qt::LeftButton | Qt::RightButton | Qt::MiddleButton ) )
		{
			QPoint currentPos = ev->globalPos();
			QPoint deltaPos = currentPos - m_lastMousePos;
			if ( buttons & Qt::LeftButton )
			{
				QQuaternion rotation = 	cam->pan( -( float )deltaPos.x() * 0.01f ) *
										cam->tilt( -( float )deltaPos.y() * 0.01f );
				cam->rotateEye( rotation );
			}
			if ( buttons & Qt::MiddleButton )
			{
				QQuaternion rotation = cam->roll( -( float )deltaPos.y() * 0.01f );
				cam->rotateEye( rotation );
			}
			if ( buttons & Qt::RightButton )
			{
				QVector3D translation = cam->translation( ( float )deltaPos.x() * 0.001f,
														  ( float )deltaPos.y() * 0.001f, 0 );
				cam->setEye( cam->eye() + translation );
				cam->setCenter( cam->center() + translation );
			}
			
			m_lastMousePos = currentPos;
		}
	}
	
	void ReconstructionViewer::wheelEvent( QWheelEvent * ev )
	{
		QGLCamera* cam = camera();
		QVector3D translation = cam->translation( 0.f, 0.f, ( float )ev->angleDelta().y() * 0.01f );
		cam->setEye( cam->eye() + translation );
		cam->setCenter( cam->center() + translation );
	}
	
	void ReconstructionViewer::mousePressEvent( QMouseEvent* ev )
	{
		m_lastMousePos = ev->globalPos();
	}
}