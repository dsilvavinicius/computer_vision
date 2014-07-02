#include "CircularList.h"

template <typename T>
CircularList::CircularList(int& maxSize)
{
	m_container = make_shared<vector<T>>(size);
	m_maxSize = maxSize;
	m_size = 0;
}

template <typename T>
T CircularList::operator [] (int i)
{
	return m_container->[i % m_maxSize];
}

template <typename T>
void CircularList::pushBack(T& element)
{
	m_container->[m_size++] = element;
}

template <typename T>
void CircularList::clear()
{
	m_container->clear();
}