
class Devin {

 public:

  enum DevinAnswer {WIN, GREATER, SMALLER};

  Devin(int min_=0, int max_=100);

  int getNbTry() const {return nbTry;};
  int getSecret() const {return secret;};

  void newGame();
  DevinAnswer tryNumber(int n_);
  
 private:

  int nbMin, nbMax;
  int secret;
  int nbTry;
  
};
