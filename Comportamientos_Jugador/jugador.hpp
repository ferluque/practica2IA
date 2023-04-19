#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <iostream>

ostream& operator<<(ostream& out, const ubicacion& x);

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

struct nodeN0 {
  stateN0 st;
  list<Action> secuencia;
  bool operator==(const nodeN0& n) const {
    return (st==n.st);
  }
  bool operator<(const nodeN0& n) const {
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if(st.jugador.f == n.st.jugador.f and st.jugador.c<n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else
      return false;
  }
};

ostream& operator<<(ostream& out, const stateN0& x);

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
    void VisualizaPlan(const stateN0& st, const list<Action>& plan);
};

list<Action> AnchuraSoloJugador(const stateN0& inicio, const ubicacion& final, const vector<vector<unsigned char>>& mapa);
bool Find(const list<stateN0>& lista, const stateN0& obj);
bool Find(const list<nodeN0>& lista, const stateN0& obj);

bool CasillaTransitable(const ubicacion& x, const vector<vector<unsigned char>>& mapa);
ubicacion NextCasilla(const ubicacion& pos);
stateN0 apply(Action action, const stateN0& current_state, const vector<vector<unsigned char>>& mapa);

void AnularMatriz(vector<vector<unsigned char>>& matriz);

#endif
