#ifndef CIRCULAR_LIST_H
#define CIRCULAR_LIST_H

#include <stdexcept>
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

		/** Inserts an element at list's end. Returns the element that was previously
		* in that position. */
		T push(const T& element);

		/** Pops the last inserted element. */
		T pop();

		bool empty();
		int size();

		/** Clears the list. Returns the elements in the list at the moment (not sorted). */
		vector<T> clear();
	private:
        void incrementPointer(int increment);

		shared_ptr<vector<T>> m_container;
		int m_pointer;
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
		m_pointer = 0;
	}

	template <typename T>
	T CircularList<T>::operator [] (int i)
	{
        if (i >= m_size)
        {
            throw logic_error("Index out of bounds.");
        }
        if (m_size < m_maxSize)
        {
            return (*m_container)[i];
        }
        else
        {
            return (*m_container)[(m_pointer + i) % m_maxSize];
        }
	}

	template <typename T>
	T CircularList<T>::push(const T& element)
	{
        T previousElement = nullptr;
        if (size() >= m_maxSize)
        {
            previousElement = (*m_container)[m_pointer];
        }

		(*m_container)[m_pointer] = element;
		incrementPointer(1);
		if (m_size < m_maxSize) { ++m_size; }

		return previousElement;
	}

	template <typename T>
	T CircularList<T>::pop()
	{
        if (empty())
        {
            throw logic_error("Trying to pop an empty list.");
        }
        else
        {
            m_pointer = incrementPointer(-1);
            if (m_size > 0) { --m_size; };
            return m_container[m_pointer];
        }

	}

    template <typename T>
    bool CircularList<T>::empty()
    {
        return m_size == 0;
    }

    template <typename T>
    int CircularList<T>::size()
    {
        return m_size;
    }

	template <typename T>
	vector<T> CircularList<T>::clear()
	{
        int size = m_size;
        vector<T> elements(size);
        for (int i = 0; i < size; ++i)
        {
            elements[i] = (*this)[i];
        }

        m_container->clear();
        m_pointer = 0;
        m_size = 0;

        return elements;
	}

	template <typename T>
	void CircularList<T>::incrementPointer(int increment)
	{
        m_pointer = (m_pointer + increment) % m_maxSize;
	}
}

#endif
