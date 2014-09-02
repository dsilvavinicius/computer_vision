#ifndef PANORAMA_CONTROLLER_H

#include <QPixmap>
#include <vector>

using namespace std;

namespace model
{
	class PanoramaController
	{
	public:
		/** Computes a panorama from given images. All images will be mapped to centerImg space, in the order of the indices in orderIndices.
		 * @param centerImg is the center image, which will define the space of the panorama final image.
		 * @param images are the other images, which will be mapped to centerImg space.
		 * @param orderIndices are the indices of the images in images. The order of the elements in orderIndices indicates the order in which
		 * the map will be done.
		 */
		static QPixmap computePanorama(QPixmap* centerImg, vector< QPixmap* >& images, vector< int >& orderIndices);
		
		/** Maps currentImg to the panorama space, also putting the first in the later. */
		static void map(QPixmap* panorama, QPixmap* centerImg, QPixmap* currentImg);
	private:
		PanoramaController();
	};
}

#endif