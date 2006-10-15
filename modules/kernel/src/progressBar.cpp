/* $Id$ */

#include <iostream>
#include <iomanip>
#include <sstream>

#include "kernel/progressBar.hpp"

using namespace std;
using namespace jafar::kernel;

ProgressBar::ProgressBar(string title, unsigned int cptMax, unsigned int barWidth) :
  m_barWidth(barWidth),
  m_cpt(0),
  m_cptMax(cptMax),
  chrono()
{
  cout << title << endl;
  bar();
}

ProgressBar::~ProgressBar()
{
  cout << endl;
}

void ProgressBar::operator++()
{
  ++m_cpt;
  bar();
}

void ProgressBar::bar() 
{
  double d = double(m_cpt)/double(m_cptMax);

  ostringstream buf;

  buf << setw(3) << int(100*d) << "%[";
  unsigned int i = 0;
  while (i < d*m_barWidth) {
    buf << "=";
    ++i;
  }
  if (i<m_barWidth) {
    buf << ">";
    ++i;
  }
  while (i<m_barWidth) {
    buf << " ";
    ++i;
  }
  buf << "] " << chrono.elapsedSecond() << "s";

  cout << "\r" << buf.str() << flush;

}
