#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;

	if (!hayPlan)
	{
		cout << "Calculando un nuevo plan" << endl;
		current_state.jugador.f = sensores.posF;
		current_state.jugador.c = sensores.posC;
		current_state.jugador.brujula = sensores.sentido;
		current_state.sonambulo.f = sensores.SONposF;
		current_state.sonambulo.c = sensores.SONposC;
		current_state.sonambulo.brujula = sensores.SONsentido;
		goal.f = sensores.destinoF;
		goal.c = sensores.destinoC;

		switch (sensores.nivel)
		{
		case 0:
			plan = AnchuraSoloJugador(current_state, goal, mapaResultado);
			break;
		case 1:
			plan = AnchuraSonambulo(current_state, goal, mapaResultado, casillasTerreno);
			break;
		case 2:
			plan = DijkstraSoloJugador(current_state, goal, mapaResultado);
			break;
		case 3:
			plan = AStar(current_state, goal, mapaResultado, casillasTerreno);
			break;
		case 4:
			Nivel4(sensores);
		}
		if (plan.size() > 0)
		{
			VisualizaPlan(current_state, plan);
			hayPlan = true;
		}
	}
	if (hayPlan and plan.size() > 0)
	{
		cout << "Ejecutando siguiente acción del plan" << endl;
		accion = plan.front();
		plan.pop_front();
	}
	if (plan.size() == 0)
	{
		cout << "Se completó el plan" << endl;
		hayPlan = false;
	}

	return accion;
}

// ==============================================
// NIVEL 4 ======================================
// ==============================================
void ComportamientoJugador::Nivel4(Sensores sensores)
{
	return abs(destino.f - origen.f) + abs(destino.c - origen.c);
}

int Distancia(const nodeN3 &origen, Action accion, const vector<vector<unsigned char>> &mapa, bool sonambulo)
{

	int distancia = 0;
	unsigned char casilla = mapa[origen.st.jugador.f][origen.st.jugador.c];
	if (sonambulo)
		casilla = mapa[origen.st.sonambulo.f][origen.st.sonambulo.c];
	switch (accion)
	{
		current_stateN4.jugador.f = sensores.posF;
		current_stateN4.jugador.c = sensores.posC;
		current_stateN4.jugador.brujula = sensores.sentido;
		current_stateN4.bienSituado = true;
		current_stateN4.sonambulo.f = sensores.SONposF;
		current_stateN4.sonambulo.c = sensores.SONposC;
		current_stateN4.sonambulo.brujula = sensores.SONsentido;
	}
	if (current_stateN4.bienSituado)
	{
		switch (last_action)
		{
		case actFORWARD:
			switch (current_stateN4.jugador.brujula)
			{
			case norte:
				current_stateN4.jugador.f--;
				break;
			case noreste:
				current_stateN4.jugador.f--;
				current_stateN4.jugador.c++;
				break;
			case este:
				current_stateN4.jugador.c++;
				break;
			case sureste:
				current_stateN4.jugador.f++;
				current_stateN4.jugador.c++;
				break;
			case sur:
				current_stateN4.jugador.f++;
				break;
			case suroeste:
				current_stateN4.jugador.f++;
				current_stateN4.jugador.c--;
				break;
			case oeste:
				current_stateN4.jugador.c--;
				break;
			case noroeste:
				current_stateN4.jugador.c--;
				current_stateN4.jugador.f--;
				break;
			}
		case actSON_FORWARD:
			switch (current_stateN4.sonambulo.brujula)
			{
			case norte:
				current_stateN4.sonambulo.f--;
				break;
			case noreste:
				current_stateN4.sonambulo.f--;
				current_stateN4.sonambulo.c++;
				break;
			case este:
				current_stateN4.sonambulo.c++;
				break;
			case sureste:
				current_stateN4.sonambulo.f++;
				current_stateN4.sonambulo.c++;
				break;
			case sur:
				current_stateN4.sonambulo.f++;
				break;
			case suroeste:
				current_stateN4.sonambulo.f++;
				current_stateN4.sonambulo.c--;
				break;
			case oeste:
				current_stateN4.sonambulo.c--;
				break;
			case noroeste:
				current_stateN4.sonambulo.c--;
				current_stateN4.sonambulo.f--;
				break;
			}
			break;
		}
	}
	cout << "Cambiamos orientación" << endl;
	int a;
	switch (last_action)
	{
	case actTURN_R:
		a = current_stateN4.jugador.brujula;
		a = (a + 2) % 8;
		current_stateN4.jugador.brujula = static_cast<Orientacion>(a);
		break;
	case actTURN_L:
		a = current_stateN4.jugador.brujula;
		a = (a + 6) % 8;
		current_stateN4.jugador.brujula = static_cast<Orientacion>(a);
		break;
	case actSON_TURN_SR:
		a = current_stateN4.sonambulo.brujula;
		a = (a + 1) % 8;
		current_stateN4.sonambulo.brujula = static_cast<Orientacion>(a);
		break;
	case actSON_TURN_SL:
		a = current_stateN4.sonambulo.brujula;
		a = (a + 7) % 8;
		current_stateN4.sonambulo.brujula = static_cast<Orientacion>(a);
		break;
	}

	
	// Rellenar el mapa con la visión si estamos bien situados
	if (current_stateN4.bienSituado)
		rellenaMapa(sensores.terreno, mapaResultado, current_stateN4, casillasTerreno, sensores);
}

bool cmpN3(nodeN3 a, nodeN3 n) {
	stateN0 st = a.st;
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if(st.jugador.f == n.st.jugador.f and st.jugador.c<n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f < n.st.sonambulo.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c < n.st.sonambulo.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula<n.st.sonambulo.brujula)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula==n.st.sonambulo.brujula and
	  a.tieneBikini<n.tieneBikini)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula==n.st.sonambulo.brujula and
	  a.tieneBikini==n.tieneBikini and a.tieneZapatillas<n.tieneZapatillas)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula==n.st.sonambulo.brujula and
	  a.tieneBikini==n.tieneBikini and a.tieneZapatillas==n.tieneZapatillas and a.tieneBikiniSon<n.tieneBikiniSon)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula==n.st.sonambulo.brujula and
	  a.tieneBikini==n.tieneBikini and a.tieneZapatillas==n.tieneZapatillas and a.tieneBikiniSon==n.tieneBikiniSon and a.tieneZapatillasSon<n.tieneZapatillasSon)
      return true;
    else
      return false;

}

list<Action> AStar(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa, const vector<vector<pair<int, int>>> &casillasTerreno)
{
	list<Action> plan;
	// list<nodeN3> cerrados;
	set<nodeN3, decltype(cmpN3)*>cerrados(cmpN3);	
	priority_queue<nodeN3> abiertos;
	nodeN3 actual;
	actual.g = 0;
	actual.st = inicio;
	actual.h = DistanciaCartesiana(actual.st.sonambulo, final);
	actual.tieneBikini = actual.tieneZapatillas = actual.tieneBikiniSon = actual.tieneZapatillasSon = false;
	bool fin = (actual.st.sonambulo.f == final.f and actual.st.sonambulo.c == final.c);
	abiertos.push(actual);
	int contador = 0;
	int limit = -1;

	while (!fin)
	{
		actual = abiertos.top();
		abiertos.pop();
		// if (actual.st.sonambulo.f==9 and actual.st.sonambulo.c==23 and actual.st.sonambulo.brujula==3 and EsVisible(actual.st, casillasTerreno))
		// 	limit = contador;
		if (contador == limit)
		{
			cout << "Actual: " << endl
				 << actual.st << "g: " << actual.g << " // h: " << actual.h << endl
				 << endl;
			cout << "Casilla actual: " << mapa[actual.st.sonambulo.f][actual.st.sonambulo.c] << endl;
			
		}
		// OBJETOS JUGADOR
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'K')
		{
			if (!actual.tieneZapatillas)
				actual.tieneBikini = true;
			else
			{
				// Genera hijo que coge bikini y sigue analizando el que se queda con las zapatillas
				nodeN3 childCogeBikini = actual;
				childCogeBikini.tieneBikini = true;
				childCogeBikini.tieneZapatillas = false;
				// if (!Find(cerrados, childCogeBikini))
				if (cerrados.find(childCogeBikini)==cerrados.end())
					abiertos.push(childCogeBikini);
			}
		}
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'D')
		{
			if (!actual.tieneBikini)
			{
				actual.tieneZapatillas = true;
			}
			else
			{
				nodeN3 childCogeZapatillas = actual;
				childCogeZapatillas.tieneBikini = false;
				childCogeZapatillas.tieneZapatillas = true;
				if (cerrados.find(childCogeZapatillas)==cerrados.end())
					abiertos.push(childCogeZapatillas);
			}
		}
		// OBJETOS SONÁMBULO
		if (EsVisible(actual.st, casillasTerreno))
		{
			if (mapa[actual.st.sonambulo.f][actual.st.sonambulo.c] == 'K')
			{
				if (!actual.tieneZapatillasSon)
					actual.tieneBikiniSon = true;
				else
				{
					nodeN3 childCogeBikiniSon = actual;
					if (contador == limit)
					{
						cout << "childCogeBikiniSon: " << endl
							 << childCogeBikiniSon.st << "g: " << childCogeBikiniSon.g << " // h: " << childCogeBikiniSon.h << endl
							 << endl;
					}
					childCogeBikiniSon.tieneBikiniSon = true;
					childCogeBikiniSon.tieneZapatillasSon = false;
					if (cerrados.find(childCogeBikiniSon)==cerrados.end())
							abiertos.push(childCogeBikiniSon);
				}
			}
			if (mapa[actual.st.sonambulo.f][actual.st.sonambulo.c] == 'D')
			{
				if (!actual.tieneBikiniSon)
					actual.tieneZapatillasSon = true;
				else
				{
					nodeN3 childCogeZapatillasSon = actual;
					if (contador == limit)
					{
						cout << "childCogeZapatillasSon: " << endl
							 << childCogeZapatillasSon.st << "g: " << childCogeZapatillasSon.g << " // h: " << childCogeZapatillasSon.h << endl
							 << endl;
					}
					childCogeZapatillasSon.tieneBikiniSon = false;
					childCogeZapatillasSon.tieneZapatillasSon = true;
					if (cerrados.find(childCogeZapatillasSon)==cerrados.end())
						abiertos.push(childCogeZapatillasSon);
				}
			}
		}
		fin = (actual.st.sonambulo.f == final.f and actual.st.sonambulo.c == final.c);
		if (fin)
		{
			plan = actual.secuencia;
		}
		// GENERA HIJOS
		else
		{
			nodeN3 childForward = actual;
			childForward.st = apply(actFORWARD, childForward.st, mapa);
			childForward.g = actual.g + Distancia(actual, actFORWARD, mapa, false);
			childForward.h = DistanciaCartesiana(childForward.st.sonambulo, final);
			childForward.secuencia.push_back(actFORWARD);
			if (cerrados.find(childForward)==cerrados.end())
			{
				// cout << "Mete childForward" << endl;
				abiertos.push(childForward);
				if (contador == limit)
				{
					cout << "childForward: " << endl
						<< childForward.st << "g: " << childForward.g << " // h: " << childForward.h << endl
						<< endl;
				}
			}
			nodeN3 childTurnR = actual;
			childTurnR.st = apply(actTURN_R, childTurnR.st, mapa);
			childTurnR.g = actual.g + Distancia(actual, actTURN_R, mapa, false);
			childTurnR.h = DistanciaCartesiana(childTurnR.st.sonambulo, final);
			childTurnR.secuencia.push_back(actTURN_R);
			if (cerrados.find(childTurnR)==cerrados.end())
			{
				// cout << "Mete childTurnR" << endl;
				abiertos.push(childTurnR);
				if (contador == limit)
				{
					cout << "childTurnR: " << endl
						<< childTurnR.st << "g: " << childTurnR.g << " // h: " << childTurnR.h << endl
						<< endl;
				}
			}
			nodeN3 childTurnL = actual;
			childTurnL.st = apply(actTURN_L, childTurnL.st, mapa);
			childTurnL.g = actual.g + Distancia(actual, actTURN_L, mapa, false);
			childTurnL.h = DistanciaCartesiana(childTurnL.st.sonambulo, final);
			childTurnL.secuencia.push_back(actTURN_L);
			if (cerrados.find(childTurnL)==cerrados.end())
			{
				// cout << "Mete childTurnL" << endl;
				abiertos.push(childTurnL);
				if (contador == limit)
				{
					cout << "childTurnL: " << endl
						<< childTurnL.st << "g: " << childTurnL.g << " // h: " << childTurnL.h << endl
						<< endl;
				}
			}
			if (EsVisible(actual.st, casillasTerreno))
			{
				nodeN3 childForwardSon = actual;
				childForwardSon.st = apply(actSON_FORWARD, childForwardSon.st, mapa);
				childForwardSon.g = actual.g + Distancia(actual, actSON_FORWARD, mapa, true);
				childForwardSon.h = DistanciaCartesiana(childForwardSon.st.sonambulo, final);
				childForwardSon.secuencia.push_back(actSON_FORWARD);
				if (cerrados.find(childForwardSon)==cerrados.end())
				{
					// cout << "Mete childForwardSon" << endl;
					abiertos.push(childForwardSon);
					if (contador == limit)
					{
						cout << "childForwardSon: " << endl
							<< childForwardSon.st << "g: " << childForwardSon.g << " // h: " << childForwardSon.h << endl
							<< endl;
					}
				}
				nodeN3 childTurnLSon = actual;
				childTurnLSon.st = apply(actSON_TURN_SL, childTurnLSon.st, mapa);
				childTurnLSon.g = actual.g + Distancia(actual, actSON_TURN_SL, mapa, true);
				// cout << "childTurnLSon.g " << childTurnLSon.g << endl;
				childTurnLSon.h = DistanciaCartesiana(childTurnLSon.st.sonambulo, final);
				childTurnLSon.secuencia.push_back(actSON_TURN_SL);
				if (cerrados.find(childTurnLSon)==cerrados.end())
				{
					// cout << "Mete childTurnLSon" << endl;
					abiertos.push(childTurnLSon);
					if (contador == limit)
					{
						cout << "childTurnLSon: " << endl
							<< childTurnLSon.st << "g: " << childTurnLSon.g << " // h: " << childTurnLSon.h << endl
							<< endl;
					}
				}
				nodeN3 childTurnRSon = actual;
				childTurnRSon.st = apply(actSON_TURN_SR, childTurnRSon.st, mapa);
				childTurnRSon.g = actual.g + Distancia(actual, actSON_TURN_SR, mapa, true);
				childTurnRSon.h = DistanciaCartesiana(childTurnRSon.st.sonambulo, final);
				childTurnRSon.secuencia.push_back(actSON_TURN_SR);
				if (cerrados.find(childTurnRSon)==cerrados.end())
				{
					// cout << "Mete childTurnRSon" << endl;
					abiertos.push(childTurnRSon);
					if (contador == limit)
					{
						cout << "childTurnRSon: " << endl
							<< childTurnRSon.st << "g: " << childTurnRSon.g << " // h: " << childTurnRSon.h << endl
							<< endl;
					}
				}
			}
		}
		cerrados.insert(actual);
		if (contador == limit)
		{
			cout << "Termina" << endl;
			break;
		}
		++contador;
	}
}
// ==============================================
// NIVEL 0 ======================================
// ==============================================
list<Action> AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	nodeN0 current_node;
	list<nodeN0> frontier;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;
	frontier.push_back(current_node);
	bool SolutionFound = (current_node.st.jugador.f == final.f and current_node.st.jugador.c == final.c);
	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);
		// Hijo actFORWARD
		nodeN0 childForward = current_node;
		childForward.st = apply(actFORWARD, current_node.st, mapa);
		if (childForward.st.jugador.f == final.f and childForward.st.jugador.c == final.c)
		{
			childForward.secuencia.push_back(actFORWARD);
			current_node = childForward;
			SolutionFound = true;
		}
		else if (explored.find(childForward) == explored.end())
		{
			childForward.secuencia.push_back(actFORWARD);
			frontier.push_back(childForward);
		}
		if (!SolutionFound)
		{
			nodeN0 childTurnl = current_node;
			childTurnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(childTurnl) == explored.end())
			{
				childTurnl.secuencia.push_back(actTURN_L);
				frontier.push_back(childTurnl);
			}
			nodeN0 childTurnr = current_node;
			childTurnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(childTurnr) == explored.end())
			{
				childTurnr.secuencia.push_back(actTURN_R);
				frontier.push_back(childTurnr);
			}
		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() and explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}
	if (SolutionFound)
		plan = current_node.secuencia;
	return plan;
}

bool Find(const list<stateN0> &lista, const stateN0 &obj)
{
	auto it = lista.begin();
	while (it != lista.end() and !((*it) == obj))
		++it;
	return (!(it == lista.end()));
}

bool Find(const list<nodeN0> &lista, const stateN0 &obj)
{
	auto it = lista.begin();
	while (it != lista.end() and !(it->st == obj))
		++it;
	return (!(it == lista.end()));
}

// ==============================================
// NIVEL 1 ======================================
// ==============================================
list<Action> AnchuraSonambulo(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa,
							  const vector<vector<pair<int, int>>> &casillasTerreno)
{
	nodeN1 current_node;
	list<nodeN1> frontier;
	set<nodeN1> explored;
	list<Action> plan;
	current_node.st = inicio;
	frontier.push_back(current_node);
	bool SolutionFound = (current_node.st.sonambulo.f == final.f and current_node.st.sonambulo.c == final.c);
	bool print = true;
	int limit = -1;
	int cont = 0;
	while (!frontier.empty() and !SolutionFound)
	{
		// if (current_node.st.sonambulo.f==18 and current_node.st.sonambulo.c==13 and current_node.st.sonambulo.brujula==4 and
		// 	current_node.st.jugador.f==21 and current_node.st.jugador.c==15 and current_node.st.jugador.brujula==0)
		// 	limit = cont;
		frontier.pop_front();
		explored.insert(current_node);
		if (print and cont == limit)
		{
			cout << "Nodo actual" << endl;
			cout << current_node.st << endl;
		}
		// Hijo actFORWARD
		nodeN1 childForward = current_node;
		childForward.st = apply(actFORWARD, current_node.st, mapa);

		if (print and cont == limit)
			cout << "Hijo actFORWARD: " << endl
				 << childForward.st << endl;
		if (explored.find(childForward) == explored.end())
		{
			childForward.secuencia.push_back(actFORWARD);
			frontier.push_back(childForward);
		}
		if (!SolutionFound)
		{
			nodeN1 childTurnl = current_node;
			childTurnl.st = apply(actTURN_L, current_node.st, mapa);

			if (print and cont == limit)
				cout << "Hijo actTURN_L: " << endl
					 << childTurnl.st << endl;
			if (explored.find(childTurnl) == explored.end())
			{
				childTurnl.secuencia.push_back(actTURN_L);
				frontier.push_back(childTurnl);
			}
			nodeN1 childTurnr = current_node;
			childTurnr.st = apply(actTURN_R, current_node.st, mapa);

			if (print and cont == limit)
				cout << "Hijo actTURN_R: " << endl
					 << childTurnr.st << endl;

			if (explored.find(childTurnr) == explored.end())
			{
				childTurnr.secuencia.push_back(actTURN_R);
				frontier.push_back(childTurnr);
			}
			if (EsVisible(current_node.st, casillasTerreno))
			{
				nodeN1 childSForward = current_node;
				childSForward.st = apply(actSON_FORWARD, current_node.st, mapa);

				if (print and cont == limit)
					cout << "Hijo actSON_FORWARD: " << endl
						 << childSForward.st << endl;

				if (childSForward.st.sonambulo.f == final.f and childSForward.st.sonambulo.c == final.c)
				{
					childSForward.secuencia.push_back(actSON_FORWARD);
					current_node = childSForward;

					cout << "Solución encontrada: " << endl
						 << current_node.st << endl;
					cout << "Secuencia: ";
					for (auto it = childSForward.secuencia.begin(); it != childSForward.secuencia.end(); it++)
						cout << *(it) << " ";
					cout << endl;
					SolutionFound = true;
				}
				if (cont == limit)
					cout << "Explored.find(childSForward)" << endl
						 << explored.find(childSForward)->st << endl;
				else if (explored.find(childSForward) == explored.end())
				{
					cout << "Mete Hijo actSON_FORWARD" << endl;
					childSForward.secuencia.push_back(actSON_FORWARD);
					frontier.push_back(childSForward);
				}
				nodeN1 childTurnSR = current_node;
				childTurnSR.st = apply(actSON_TURN_SR, current_node.st, mapa);

				if (print and cont == limit)
					cout << "Hijo actSON_TURN_SR: " << endl
						 << childTurnSR.st << endl;
				if (explored.find(childTurnSR) == explored.end())
				{
					// cout << "Mete Hijo actSON_TURN_SR"<<endl;
					childTurnSR.secuencia.push_back(actSON_TURN_SR);
					frontier.push_back(childTurnSR);
				}
				nodeN1 childTurnSL = current_node;
				childTurnSL.st = apply(actSON_TURN_SL, current_node.st, mapa);

				if (print and cont == limit)
					cout << "Hijo actSON_TURN_SL: " << endl
						 << childTurnSL.st << endl;
				if (explored.find(childTurnSL) == explored.end())
				{
					// cout << "Mete Hijo actSON_TURN_SL"<<endl;
					childTurnSL.secuencia.push_back(actSON_TURN_SL);
					frontier.push_back(childTurnSL);
				}
			}
		}
		// if (print and cont==1)
		// {
		// 	cout << "Nodo actual" << endl;
		// 	cout << current_node.st << endl;
		// 	cout << "Lista frontier: " << endl;
		// 	for (auto it = frontier.begin(); it != frontier.end(); ++it)
		// 	{
		// 		cout << it->st << endl;
		// 	}

		// 	cout << "===================================" << endl;
		// 	break;
		// }
		if (cont == limit)
			break;
		++cont;

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() and explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}
	if (SolutionFound)
		plan = current_node.secuencia;
	return plan;
}

bool EsVisible(const stateN0 &st, const vector<vector<pair<int, int>>> &casillasTerreno)
{
	vector<pair<int, int>> v(16);
	vector<pair<int, int>> casillas(casillasTerreno[static_cast<int>(st.jugador.brujula)]);

	bool found = false;
	for (int i = 0; i < 16 and !found; i++)
	{
		int f = st.jugador.f + casillas[i].first, c = st.jugador.c + casillas[i].second;
		if (f == st.sonambulo.f and c == st.sonambulo.c)
			found = true;
	}
	return found;
}

bool cmpN2(nodeN2 a, nodeN2 n) {
	stateN0 st = a.st;
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if(st.jugador.f == n.st.jugador.f and st.jugador.c<n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
	  a.tieneBikini<n.tieneBikini)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
	  a.tieneBikini==n.tieneBikini and a.tieneZapatillas<n.tieneZapatillas)
      return true;
    else
      return false;
}

list<Action> DijkstraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	list<Action> plan;
	// list<nodeN2> cerrados;
	set<nodeN2, decltype(cmpN2)*>cerrados(cmpN2);	
	priority_queue<nodeN2> abiertos;
	nodeN2 actual;
	actual.distancia = 0;
	actual.st = inicio;
	actual.tieneBikini = false;
	actual.tieneZapatillas = false;
	bool fin = (actual.st.jugador.f == final.f and actual.st.jugador.c == final.c);
	abiertos.push(actual);
	int cont = 0;
	int limit = -1;
	while (!fin)
	{
		int i=actual.st.jugador.f, j=actual.st.jugador.c, k=actual.st.jugador.brujula;
		actual = abiertos.top();
		// if (actual.st.jugador.f == 18 and actual.st.jugador.c==13 and actual.st.jugador.brujula==0 and actual.tieneZapatillas)
		// 	limit = cont;

		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'K')
		{
			if (!actual.tieneZapatillas) {
				actual.tieneBikini = true;
			}
			else
			{
				// Genera hijo que coge bikini y sigue analizando el que se queda con las zapatillas
				nodeN2 childCogeBikini = actual;
				childCogeBikini.tieneBikini = true;
				childCogeBikini.tieneZapatillas = false;
				// if (!Find(cerrados, childCogeBikini))
				if (cerrados.find(childCogeBikini)==cerrados.end()) {
					abiertos.push(childCogeBikini);
					if (cont == limit) {
						cout << "childCogeBikini: " << childCogeBikini .st.jugador << endl;
					}
				}
			}
		}
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'D')
		{
			if (!actual.tieneBikini)
			{
				actual.tieneZapatillas = true;
			}
			else
			{
				nodeN2 childCogeZapatillas = actual;
				childCogeZapatillas.tieneBikini = false;
				childCogeZapatillas.tieneZapatillas = true;
				// if (!Find(cerrados, childCogeZapatillas))
				if (cerrados.find(childCogeZapatillas)==cerrados.end()) {
					abiertos.push(childCogeZapatillas);
					if (cont == limit) {
						cout << "childCogeZapatillas: " << childCogeZapatillas .st.jugador << endl;
					}
				}
			}
		}
		if (cont == limit)
		{
			cout << "Nodo actual" << endl;
			cout << actual.st.jugador << "Bikini: " << actual.tieneBikini << " Zapatillas: " << actual.tieneZapatillas << endl << endl;
		}
		abiertos.pop();
		cerrados.insert(actual);
		// cout << "Cerrados.find(actual) " << endl << cerrados.find(actual)->st.jugador << endl;
		
		// cerrados.push(actual);
		fin = (actual.st.jugador.f == final.f and actual.st.jugador.c == final.c);
		if (fin)
		{
			plan = actual.secuencia;
		}
		else
		{
			// Hijo actFORWARD
			nodeN2 childForward = actual;
			childForward.st = apply(actFORWARD, actual.st, mapa);
			if (cerrados.find(childForward)==cerrados.end())
			{
				childForward.distancia += Distancia(actual, actFORWARD, mapa);
				childForward.secuencia.push_back(actFORWARD);
				abiertos.push(childForward);
				if (cont == limit)
					cout << "childForward: " << endl
						<< childForward.st.jugador << endl;
			}
			// Hijo actTURNL
			nodeN2 childTurnL = actual;
			childTurnL.st = apply(actTURN_L, actual.st, mapa);
			if (cerrados.find(childTurnL)==cerrados.end())
			{
				childTurnL.distancia += Distancia(actual, actTURN_L, mapa);
				// cout << "Distancia hijo childTurnL: " << childTurnL.distancia << endl;
				childTurnL.secuencia.push_back(actTURN_L);
				abiertos.push(childTurnL);
				if (cont == limit)
					cout << "childTurnL: " << endl
						<< childTurnL.st.jugador << endl;
			}
			// Hijo actTURNR
			nodeN2 childTurnR = actual;
			childTurnR.st = apply(actTURN_R, actual.st, mapa);
			if (cerrados.find(childTurnR)==cerrados.end())
			{
				// if (cont==limit)
				// 	cout << "Mete childTurnR" << endl<<endl;
				childTurnR.distancia += Distancia(actual, actTURN_R, mapa);
				// cout << "Distancia hijo childTurnR: " << childTurnR.distancia << endl;
				childTurnR.secuencia.push_back(actTURN_R);
				abiertos.push(childTurnR);
				if (cont == limit)
					cout << "childTurnR: " << endl
						<< childTurnR.st.jugador << endl;
			}
		}
		if (cont == limit)
			break;
		++cont;
	}
	return plan;
}

int Distancia(const nodeN2 &origen, Action accion, const vector<vector<unsigned char>> &mapa)
{
	int distancia = 0;
	unsigned char casilla = mapa[origen.st.jugador.f][origen.st.jugador.c];
	switch (accion)
	{
	case actFORWARD:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 10 : 100;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 15 : 50;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_L:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_R:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	}
	return distancia;
}

bool Find(const list<nodeN2> &lista, const nodeN2 &obj)
{
	auto it = lista.begin();
	while (it != lista.end() and (!(it->st == obj.st) or (it->tieneBikini != obj.tieneBikini) or (it->tieneZapatillas != obj.tieneZapatillas)))
		++it;
	return (!(it == lista.end()));
}

// ==============================================
// NIVEL 3 ======================================
// ==============================================
int DistanciaCartesiana(const ubicacion &origen, const ubicacion &destino)
{
	return abs(destino.f - origen.f) + abs(destino.c - origen.c);
}

int Distancia(const nodeN3 &origen, Action accion, const vector<vector<unsigned char>> &mapa)
{

	int distancia = 0;
	unsigned char casilla = mapa[origen.st.jugador.f][origen.st.jugador.c];
	switch (accion)
	{
	case actFORWARD:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 10 : 100;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 15 : 50;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_L:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_R:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actSON_FORWARD:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikiniSon) ? 10 : 100;
			break;
		case 'B':
			distancia = (origen.tieneZapatillasSon) ? 15 : 50;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actSON_TURN_SL:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikiniSon) ? 2 : 7;
			break;
		case 'B':
			distancia = (origen.tieneZapatillasSon) ? 1 : 3;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actSON_TURN_SR:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikiniSon) ? 2 : 7;
			break;
		case 'B':
			distancia = (origen.tieneZapatillasSon) ? 1 : 3;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	}
	return distancia;
}

list<Action> AStar(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa, const vector<vector<pair<int, int>>> &casillasTerreno)
{
	list<Action> plan;
	// list<nodeN3> cerrados;
	// i: fila, j:columna, k: brujula, l: filaSON, m: colSOn, n: brujulaSON, o:  0 nada, 1 bikini, 2 zapatillas, p: 0 nadaSON, 1 bikiniSON, 2 zapatillasSON
	vector<vector<vector<vector<vector<vector<vector<vector<bool>>>>>>>>cerrados(mapa.size()); // xd
	for (int i=0; i<mapa.size(); i++) {
		cerrados[i] = vector<vector<vector<vector<vector<vector<vector<bool>>>>>>>(mapa.size());
		for (int j=0; j<mapa.size(); j++) {
			cerrados[i][j] = vector<vector<vector<vector<vector<vector<bool>>>>>>(8);
			for (int k=0; k<8; k++) {
				cerrados[i][j][k] = vector<vector<vector<vector<vector<bool>>>>>(mapa.size());
				for (int l=0; l<mapa.size(); l++) {
					cerrados[i][j][k][l] = vector<vector<vector<vector<bool>>>>(mapa.size());
					for (int m=0; m<mapa.size(); m++) {
						cerrados[i][j][k][l][m] = vector<vector<vector<bool>>>(8);
						for (int n=0; n<8; n++) {
							cerrados[i][j][k][l][m][n] = vector<vector<bool>>(3);
							for (int o=0; o<3; o++) {
								cerrados[i][j][k][l][m][n][o] = vector<bool>(3);
								for (int p=0; p<3; p++)
									cerrados[i][j][k][l][m][n][o][p] = false;
							}
						}
					}
				}
			}
		}
	}
	priority_queue<nodeN3> abiertos;
	nodeN3 actual;
	actual.g = 0;
	actual.st = inicio;
	actual.h = DistanciaCartesiana(actual.st.sonambulo, final);
	actual.tieneBikini = actual.tieneZapatillas = actual.tieneBikiniSon = actual.tieneZapatillasSon = false;
	bool fin = (actual.st.sonambulo.f == final.f and actual.st.sonambulo.c == final.c);
	abiertos.push(actual);
	int contador = 0;
	int limit = -1;

	while (!fin)
	{
		actual = abiertos.top();
		int i=actual.st.jugador.f, j=actual.st.jugador.c, k=actual.st.jugador.brujula;
		int l=actual.st.sonambulo.f, m=actual.st.sonambulo.c, n=actual.st.sonambulo.brujula;
		int o = 0;
		if (actual.tieneBikini)
			o=1;
		else if (actual.tieneZapatillas)
			o=2;
		int p = 0;
		if (actual.tieneBikiniSon)
			p=1;
		else if (actual.tieneZapatillasSon)
			p=2;
		abiertos.pop();
		// if (actual.st.sonambulo.f==8 and actual.st.sonambulo.c==11 and actual.st.sonambulo.brujula==7)
		// 	limit = contador;
		if (contador == limit)
		{
			cout << "Actual: " << endl
				 << actual.st << "g: " << actual.g << " // h: " << actual.h << endl
				 << endl;
		}
		// OBJETOS JUGADOR
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'K')
		{
			if (!actual.tieneZapatillas){
				actual.tieneBikini = true;
				o=1;
			}
			else
			{
				// Genera hijo que coge bikini y sigue analizando el que se queda con las zapatillas
				nodeN3 childCogeBikini = actual;
				childCogeBikini.tieneBikini = true;
				childCogeBikini.tieneZapatillas = false;
				// if (!Find(cerrados, childCogeBikini))
				if (!cerrados[i][j][k][l][m][n][1][p])
					abiertos.push(childCogeBikini);
			}
		}
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'D')
		{
			if (!actual.tieneBikini)
			{
				actual.tieneZapatillas = true;
				o=2;
			}
			else
			{
				nodeN3 childCogeZapatillas = actual;
				childCogeZapatillas.tieneBikini = false;
				childCogeZapatillas.tieneZapatillas = true;
				// if (!Find(cerrados, childCogeZapatillas))
				if (!cerrados[i][j][k][l][m][n][2][p])
					abiertos.push(childCogeZapatillas);
			}
		}
		// OBJETOS SONÁMBULO
		if (EsVisible(actual.st, casillasTerreno))
		{
			if (mapa[actual.st.sonambulo.f][actual.st.sonambulo.c] == 'K')
			{
				if (!actual.tieneZapatillasSon) {
					actual.tieneBikiniSon = true;
					p=1;
				}
				else
				{
					nodeN3 childCogeBikiniSon = actual;
					if (contador == limit)
					{
						cout << "childCogeBikiniSon: " << endl
							 << childCogeBikiniSon.st << "g: " << childCogeBikiniSon.g << " // h: " << childCogeBikiniSon.h << endl
							 << endl;
					}
					childCogeBikiniSon.tieneBikiniSon = true;
					childCogeBikiniSon.tieneZapatillasSon = false;
					// if (!Find(cerrados, childCogeBikiniSon))
					if (!cerrados[i][j][k][l][m][n][o][1])
						abiertos.push(childCogeBikiniSon);
				}
			}
			if (mapa[actual.st.sonambulo.f][actual.st.sonambulo.c] == 'D')
			{
				if (!actual.tieneBikiniSon) {
					actual.tieneZapatillasSon = true;
					p=2;
				}
				else
				{
					nodeN3 childCogeZapatillasSon = actual;
					if (contador == limit)
					{
						cout << "childCogeZapatillasSon: " << endl
							 << childCogeZapatillasSon.st << "g: " << childCogeZapatillasSon.g << " // h: " << childCogeZapatillasSon.h << endl
							 << endl;
					}
					childCogeZapatillasSon.tieneBikiniSon = false;
					childCogeZapatillasSon.tieneZapatillasSon = true;
					// if (!Find(cerrados, childCogeZapatillasSon))
					if (!cerrados[i][j][k][l][m][n][o][2])
						abiertos.push(childCogeZapatillasSon);
				}
			}
		}
		fin = (actual.st.sonambulo.f == final.f and actual.st.sonambulo.c == final.c);
		if (fin)
		{
			plan = actual.secuencia;
		}
		// GENERA HIJOS
		else
		{
			nodeN3 childForward = actual;
			childForward.st = apply(actFORWARD, childForward.st, mapa);
			childForward.g = actual.g + Distancia(actual, actFORWARD, mapa);
			childForward.h = DistanciaCartesiana(childForward.st.sonambulo, final);
			childForward.secuencia.push_back(actFORWARD);
			if (contador == limit)
			{
				cout << "childForward: " << endl
					 << childForward.st << "g: " << childForward.g << " // h: " << childForward.h << endl
					 << endl;
			}
			// if (!Find(cerrados, childForward))
			if (!cerrados[childForward.st.jugador.f][childForward.st.jugador.c][k][l][m][n][o][p])
			{
				// cout << "Mete childForward" << endl;
				abiertos.push(childForward);
			}
			nodeN3 childTurnR = actual;
			childTurnR.st = apply(actTURN_R, childTurnR.st, mapa);
			childTurnR.g = actual.g + Distancia(actual, actTURN_R, mapa);
			childTurnR.h = DistanciaCartesiana(childTurnR.st.sonambulo, final);
			childTurnR.secuencia.push_back(actTURN_R);
			if (contador == limit)
			{
				cout << "childTurnR: " << endl
					 << childTurnR.st << "g: " << childTurnR.g << " // h: " << childTurnR.h << endl
					 << endl;
			}
			// if (!Find(cerrados, childTurnR))
			if (!cerrados[i][j][childForward.st.jugador.brujula][l][m][n][o][p])
			{
				// cout << "Mete childTurnR" << endl;
				abiertos.push(childTurnR);
			}
			nodeN3 childTurnL = actual;
			childTurnL.st = apply(actTURN_L, childTurnL.st, mapa);
			childTurnL.g = actual.g + Distancia(actual, actTURN_L, mapa);
			childTurnL.h = DistanciaCartesiana(childTurnL.st.sonambulo, final);
			childTurnL.secuencia.push_back(actTURN_L);
			if (contador == limit)
			{
				cout << "childTurnR: " << endl
					 << childTurnR.st << "g: " << childTurnR.g << " // h: " << childTurnR.h << endl
					 << endl;
			}
			// if (!Find(cerrados, childTurnL))
			if (!cerrados[i][j][childForward.st.jugador.brujula][l][m][n][o][p])
			{
				// cout << "Mete childTurnL" << endl;
				abiertos.push(childTurnL);
			}
			if (EsVisible(actual.st, casillasTerreno))
			{
				nodeN3 childForwardSon = actual;
				childForwardSon.st = apply(actSON_FORWARD, childForwardSon.st, mapa);
				childForwardSon.g = actual.g + Distancia(actual, actSON_FORWARD, mapa);
				childForwardSon.h = DistanciaCartesiana(childForwardSon.st.sonambulo, final);
				childForwardSon.secuencia.push_back(actSON_FORWARD);
				if (contador == limit)
				{
					cout << "childForwardSon: " << endl
						 << childForwardSon.st << "g: " << childForwardSon.g << " // h: " << childForwardSon.h << endl
						 << endl;
				}
				// if (!Find(cerrados, childForwardSon))
				if (!cerrados[childForward.st.sonambulo.f][childForward.st.sonambulo.c][k][l][m][n][o][p])
				{
					// cout << "Mete childForwardSon" << endl;
					abiertos.push(childForwardSon);
				}
				nodeN3 childTurnLSon = actual;
				childTurnLSon.st = apply(actSON_TURN_SL, childTurnLSon.st, mapa);
				childTurnLSon.g = actual.g + Distancia(actual, actSON_TURN_SL, mapa);
				childTurnLSon.h = DistanciaCartesiana(childTurnLSon.st.sonambulo, final);
				childTurnLSon.secuencia.push_back(actSON_TURN_SL);
				if (contador == limit)
				{
					cout << "childTurnLSon: " << endl
						 << childTurnLSon.st << "g: " << childTurnLSon.g << " // h: " << childTurnLSon.h << endl
						 << endl;
				}
				// if (!Find(cerrados, childTurnLSon))
				if (!cerrados[i][j][childForward.st.sonambulo.brujula][l][m][n][o][p])
				{
					// cout << "Mete childTurnLSon" << endl;
					abiertos.push(childTurnLSon);
				}
				nodeN3 childTurnRSon = actual;
				childTurnRSon.st = apply(actSON_TURN_SR, childTurnRSon.st, mapa);
				childTurnRSon.g = actual.g + Distancia(actual, actSON_TURN_SR, mapa);
				childTurnRSon.h = DistanciaCartesiana(childTurnRSon.st.sonambulo, final);
				childTurnRSon.secuencia.push_back(actSON_TURN_SR);
				if (contador == limit)
				{
					cout << "childTurnRSon: " << endl
						 << childTurnRSon.st << "g: " << childTurnRSon.g << " // h: " << childTurnRSon.h << endl
						 << endl;

					// auto it = cerrados.begin();
					// while (it != cerrados.end() and (!(it->st == childTurnRSon.st) or (it->st.sonambulo.brujula!=childTurnRSon.st.sonambulo.brujula) or (it->tieneBikini != childTurnRSon.tieneBikini) or (it->tieneZapatillas != childTurnRSon.tieneZapatillas) or
					// 							(it->tieneBikiniSon != childTurnRSon.tieneBikiniSon) or (it->tieneZapatillasSon != childTurnRSon.tieneZapatillasSon)))
					// 	++it;
					// cout << "Find(childTurnRSon): " << endl
					// 	 <<  it->st << "g: " << it->g << " // h: " << it->h << endl
					// 	 << endl;
				}
				// if (!Find(cerrados, childTurnRSon))
				if (!cerrados[i][j][childForward.st.sonambulo.brujula][l][m][n][o][p])
				{
					// cout << "Mete childTurnRSon" << endl;
					abiertos.push(childTurnRSon);
				}
			}
		}
		// cerrados.push_back(actual);
		cerrados[i][j][k][l][m][n][o][p] = true;
		if (contador == limit)
		{
			cout << "Termina" << endl;
			break;
		}
		++contador;
	}
	return plan;
}

bool Find(const list<nodeN3> &lista, const nodeN3 &obj)
{
	auto it = lista.begin();
	while (it != lista.end() and (!(it->st == obj.st) or (it->st.sonambulo.brujula != obj.st.sonambulo.brujula) or (it->tieneBikini != obj.tieneBikini) or (it->tieneZapatillas != obj.tieneZapatillas) or
								  (it->tieneBikiniSon != obj.tieneBikiniSon) or (it->tieneZapatillasSon != obj.tieneZapatillasSon)))
		++it;
	return (!(it == lista.end()));
}

// ==============================================
// AUXILIARES ======================================
// ==============================================
void print_queue(priority_queue<nodeN2> q)
{
	while (!q.empty())
	{
		cout << q.top().st.jugador << " Distancia: " << q.top().distancia << endl;
		q.pop();
	}
	cout << endl;
}

void AnularMatriz(vector<vector<unsigned char>> &matriz)
{
	for (int i = 0; i < matriz.size(); i++)
		for (int j = 0; j < matriz[0].size(); j++)
			matriz[i][j] = 0;
}

void ComportamientoJugador::VisualizaPlan(const stateN0 &st, const list<Action> &plan)
{
	AnularMatriz(mapaConPlan);
	stateN0 cst = st;
	auto it = plan.begin();
	while (it != plan.end())
	{
		switch (*it)
		{
		case actFORWARD:
			cst.jugador = NextCasilla(cst.jugador);
			mapaConPlan[cst.jugador.f][cst.jugador.c] = 1;
			break;
		case actTURN_R:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 2) % 8);
			break;
		case actTURN_L:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 6) % 8);
			break;
		case actSON_FORWARD:
			cst.sonambulo = NextCasilla(cst.sonambulo);
			mapaConPlan[cst.sonambulo.f][cst.sonambulo.c] = 2;
			break;
		case actSON_TURN_SR:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 1) % 8);
			break;
		case actSON_TURN_SL:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 7) % 8);
			break;
		}
		++it;
	}
}

stateN0 apply(Action action, const stateN0 &current_state, const vector<vector<unsigned char>> &mapa)
{
	stateN0 stResult = current_state;
	ubicacion sigUbicacion;
	switch (action)
	{
	case actFORWARD:
		sigUbicacion = NextCasilla(current_state.jugador);
		if (CasillaTransitable(sigUbicacion, mapa) and
			!(sigUbicacion.f == current_state.sonambulo.f and sigUbicacion.c == current_state.sonambulo.c))
		{
			stResult.jugador = sigUbicacion;
		}
		break;
	case actTURN_L:
		stResult.jugador.brujula = static_cast<Orientacion>((stResult.jugador.brujula + 6) % 8);
		break;
		break;
	case actTURN_R:
		stResult.jugador.brujula = static_cast<Orientacion>((stResult.jugador.brujula + 2) % 8);
		break;
	case actSON_FORWARD:
		sigUbicacion = NextCasilla(current_state.sonambulo);
		if (CasillaTransitable(sigUbicacion, mapa) and
			!(sigUbicacion.f == current_state.jugador.f and sigUbicacion.c == current_state.jugador.c))
			stResult.sonambulo = sigUbicacion;
		break;
	case actSON_TURN_SL:
		stResult.sonambulo.brujula = static_cast<Orientacion>((stResult.sonambulo.brujula + 7) % 8);
		break;
	case actSON_TURN_SR:
		stResult.sonambulo.brujula = static_cast<Orientacion>((stResult.sonambulo.brujula + 1) % 8);
		break;
	}
	return stResult;
}

bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa)
{
	return (mapa[x.f][x.c] != 'P' and mapa[x.f][x.c] != 'M');
}

ubicacion NextCasilla(const ubicacion &pos)
{
	ubicacion next = pos;
	switch (next.brujula)
	{
	case norte:
		next.f--;
		break;
	case noreste:
		next.f--;
		next.c++;
		break;
	case este:
		next.c++;
		break;
	case sureste:
		next.f++;
		next.c++;
		break;
	case sur:
		next.f++;
		break;
	case suroeste:
		next.f++;
		next.c--;
		break;
	case oeste:
		next.c--;
		break;
	case noroeste:
		next.c--;
		next.f--;
		break;
	default:
		break;
	}
	return next;
}

int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}

ostream &operator<<(ostream &out, const ubicacion &x)
{
	out << "Fila / Col / Orientacion: " << x.f << " / " << x.c << " / " << x.brujula << endl;
	return out;
}

ostream &operator<<(ostream &out, const stateN0 &x)
{
	out << "Ubicación jugador:" << endl;
	out << x.jugador;
	out << "Ubicación sonámbulo:" << endl;
	out << x.sonambulo;
	return out;
};
