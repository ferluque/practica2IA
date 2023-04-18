#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <iostream>

// ostream& operator<<(ostream& out, const ubicacion& x) {
//   out << "Fila / Col / Orientacion: " << x.f << " / " << x.c << " / " << x.brujula << endl;
//   return out;
// }

struct stateN0 {
  ubicacion jugador;
  ubicacion sonambulo;
  bool operator== (const stateN0& x) const {
    if (jugador == x.jugador and sonambulo.f==x.sonambulo.f and sonambulo.c==x.sonambulo.c)
      return true;
    else 
      return false;
  };
};

// ostream& operator<<(ostream& out, const stateN0& x) {
//   out << "Ubicación jugador:" << endl;
//   out << x.jugador;
//   out << "Ubicación sonámbulo:" << endl;
//   out << x.sonambulo;
//   return out;
// };

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado
      hayPlan = false;
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);


  private:
    // Declarar Variables de Estado
    list<Action> plan;
    bool hayPlan;
    stateN0 current_state;
    ubicacion goal;
};

bool AnchuraSoloJugador(const stateN0& inicio, const ubicacion& final, const vector<vector<unsigned char>>& mapa);
bool Find(const list<stateN0>& lista, const stateN0& obj);

bool CasillaTransitable(const ubicacion& x, const vector<vector<unsigned char>>& mapa);
ubicacion NextCasilla(const ubicacion& pos);
stateN0 apply(Action action, const stateN0& current_state, const vector<vector<unsigned char>>& mapa);

#endif
