/* $Id$ */
#include "kernel/IdFactory.hpp"

using namespace jafar::kernel;

IdFactory::IdFactory()
{
}

IdFactory::~IdFactory()
{
}

unsigned int IdFactory::getId()
{
  return ++m_lastId;
}

void IdFactory::releaseId(unsigned int id)
{
  if(id == m_lastId )
  {
    --m_lastId;
  }
}
