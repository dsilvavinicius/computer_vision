#ifndef CIRCULAR_LIST_H
#define CIRCULAR_LIST_H

#include <memory>
#include <vector>

using namespace std;

namespace utils
{
	/** Circular list. If the list became full, insertions are circularly done in its beginning
	*	(in this case, elements of the beginning of the list are overwritten). Every access is modulated
	*	by the maximum size of the list. */
	template <typename T>
	class CircularList 
	{
	public:
		CircularList(int& maxSize);
		T operator [] (int i);
		void pushBack(T& element);
		void clear();
	private:
		shared_ptr<vector<T>> m_containter;
		int m_size;
		int m_maxSize;
	};

	template <typename T>
	using CircularListPtr = shared_ptr<CircularList<T>>;
}

#endif