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
		CircularList(const int& maxSize);
		T operator [] (int i);
		void pushBack(const T& element);
		void clear();
	private:
		shared_ptr<vector<T>> m_container;
		int m_size;
		int m_maxSize;
	};

	template <typename T>
	using CircularListPtr = shared_ptr<CircularList<T>>;

	template <typename T>
	CircularList<T>::CircularList(const int& maxSize)
	{
		m_container = make_shared<vector<T>>(maxSize);
		m_maxSize = maxSize;
		m_size = 0;
	}

	template <typename T>
	T CircularList<T>::operator [] (int i)
	{
		return *m_container[i % m_maxSize];
	}

	template <typename T>
	void CircularList<T>::pushBack(const T& element)
	{
		(*m_container)[m_size++ % m_maxSize] = element;
	}

	template <typename T>
	void CircularList<T>::clear()
	{
		m_container->clear();
	}
}

#endif