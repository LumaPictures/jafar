#include "slam/abstractMapObject.hpp"

using namespace jafar;
using namespace jafar::slam;
using namespace ublas;
using namespace jblas;

AbstractMapObject::~AbstractMapObject()
{
	delete m_x;
	delete m_P;
}

void AbstractMapObject::setState(vec& x_, sym_mat& P_) {

	delete m_x;
	delete m_P;

	range r(filterIndex(), filterIndex() + sizeState());

	m_x = new vec_range(x_, r);
	m_P = new sym_mat_range(P_, r, r);
}

