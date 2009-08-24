/* $Id$ */
#include "kernel/IdMaker.hpp"

using namespace jafar::kernel;

IdMaker::IdMaker()
{
}

IdMaker::~IdMaker()
{
}

unsigned int IdMaker::getId()
{
  return m_lastId++;
}

void IdMaker::releaseId(unsigned int id)
{
  if(id == m_lastId -1 )
  {
    --m_lastId;
  }
}
