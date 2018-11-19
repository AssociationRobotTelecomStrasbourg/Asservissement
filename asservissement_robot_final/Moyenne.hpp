#ifndef MOYENNE_HPP
#define MOYENNE_HPP

template<typename T, unsigned int N>
class Moyenne
{
public:
  Moyenne()
  {
    init();
  }

  void init()
  {
    index = 0;
    somme = 0;
    for(unsigned int i = 0; i < N; i++)
    {
      buffer[i] = 0;
    }
  }

  void ajout(T nouvelleValeur)
  {
    somme -= buffer[index];
    buffer[index] = nouvelleValeur;
    somme += nouvelleValeur;
    index = (index + 1) % N;
  }

  T valeur() const
  {
    return somme / (T)N;
  }

private:
  T buffer[N];
  unsigned int index;
  T somme;
};

#endif
