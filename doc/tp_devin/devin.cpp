#include "devin/devin.hpp"

using namespace jafar::devin;

Devin::Devin(int min_, int max_) :
  nbMin(min_), nbMax(max_),
  secret(0),
  nbTry(0)
{}

void Devin::newGame() {
  nbTry=0;
  // select a random integer
  secret = 14;
}

Devin::DevinAnswer Devin::tryNumber(int n_) {
  nbTry++;
  if (n_ == secret)
    return WIN;
  else if (n_ < secret)
    return SMALLER;
  else
    return GREATER;
}
