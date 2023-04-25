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
	if (sensores.nivel != 4)
	{
		if (!hayPlan)
		{
			cout << "Calculando un nuevo plan" << endl;
			current_state.jugador.f = sensores.posF;
			current_state.jugador.c = sensores.posC;
			current_state.jugador.brujula = sensores.sentido;
			current_state.sonambulo.f = sensores.SONposC;
			current_state.sonambulo.c = sensores.SONposC;
			current_state.sonambulo.brujula = sensores.sentido;
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
	}
	return accion;
}


list<Action> AnchuraSonambulo(const stateN0& inicio, const ubicacion& final, const vector<vector<unsigned char>>& mapa, 
 const vector<vector<pair<int,int>>>& casillasTerreno) {
	nodeN1 current_node;
	list<nodeN1> frontier;
	set<nodeN1> explored;
	list<Action> plan;
	current_node.st = inicio;
	frontier.push_back(current_node);
	bool SolutionFound = (current_node.st.sonambulo.f == final.f and current_node.st.sonambulo.c == final.c);
	bool print = false;
	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);
		if (print){
			cout << "Nodo actual" << endl;
			cout << current_node.st;
		}
		// Hijo actFORWARD
		nodeN1 childForward = current_node;
		childForward.st = apply(actFORWARD, current_node.st, mapa);
		if (childForward.st.sonambulo.f == final.f and childForward.st.sonambulo.c == final.c)
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
			nodeN1 childTurnl = current_node;
			childTurnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(childTurnl) == explored.end())
			{
				childTurnl.secuencia.push_back(actTURN_L);
				frontier.push_back(childTurnl);
			}
			nodeN1 childTurnr = current_node;
			childTurnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(childTurnr) == explored.end())
			{
				childTurnr.secuencia.push_back(actTURN_R);
				frontier.push_back(childTurnr);
			}
			if (EsVisible(current_node.st, casillasTerreno)) {
				nodeN1 childTurnSForward = current_node;
				childTurnSForward.st = apply(actSON_FORWARD, current_node.st, mapa);
				if (explored.find(childTurnSForward) == explored.end())
				{
					childTurnSForward.secuencia.push_back(actSON_FORWARD);
					frontier.push_back(childTurnSForward);
				}
				nodeN1 childTurnSR = current_node;
				childTurnSR.st = apply(actSON_TURN_SR, current_node.st, mapa);
				if (explored.find(childTurnSR) == explored.end())
				{
					childTurnSR.secuencia.push_back(actTURN_SR);
					frontier.push_back(childTurnSR);
				}
				nodeN1 childTurnSL = current_node;
				childTurnSL.st = apply(actSON_TURN_SL, current_node.st, mapa);
				if (explored.find(childTurnSL) == explored.end())
				{
					childTurnSL.secuencia.push_back(actTURN_SL);
					frontier.push_back(childTurnSL);
				}
			}
		}

		if (print) {
			cout << "Lista frontier: " << endl;
			for (auto it=frontier.begin(); it!=frontier.end(); ++it) {
				cout << it->st;
			}

			cout << "===================================" << endl;
			break;
		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() and explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				current_node = frontier.front();
			}
		}
	}
	if (SolutionFound)
		plan = current_node.secuencia;
	return plan;

}

bool EsVisible(const stateN0& st, const vector<vector<pair<int,int>>>& casillasTerreno) {
	vector<pair<int,int>> v(16);
	vector<pair<int,int>> casillas(casillasTerreno[static_cast<int>(st.jugador.brujula)]);

	bool found = false; 
	for (int i=0; i<16 and !found; i++) {
		int f = st.jugador.f+casillas[i].first, c = st.jugador.c+casillas[i].second;
		if (f==st.sonambulo.f and c==st.sonambulo.c)
			found = true;
	}
	return found;
}
void print_queue(priority_queue<nodeN2> q)
{
  while (!q.empty())
  {
    cout << q.top().st.jugador << " Distancia: " << q.top().distancia << endl;
    q.pop();
  }
  cout << endl;
}

list<Action> DijkstraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	list<Action> plan;
	list<nodeN2> cerrados;
	priority_queue<nodeN2> abiertos;
	vector<vector<int>> distancias(mapa.size());
	for (int i = 0; i < mapa.size(); i++)
	{
		distancias[i].resize(mapa.size());
	}
	MatrizAlInfinito(distancias);
	nodeN2 actual;
	actual.distancia = 0;
	actual.st = inicio;
	actual.tieneBikini = false;
	actual.tieneZapatillas = false;
	distancias[actual.st.jugador.f][actual.st.jugador.c] = actual.distancia;
	bool fin = (actual.st.jugador.f == final.f and actual.st.jugador.c == final.c);
	abiertos.push(actual);
	int cont = 0;
	bool print = false;
	while (!fin)
	{
		++cont;
		actual = abiertos.top();
		if (print){
			cout << "Nodo actual" << endl;
			cout << actual.st.jugador << " Distancia: " << actual.distancia << endl;
		}
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'K')
			actual.tieneBikini = true;
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'D')
			actual.tieneZapatillas = true;
		abiertos.pop();
		cerrados.push_back(actual);
		fin = (actual.st.jugador.f == final.f and actual.st.jugador.c == final.c);
		if (fin) {
			plan = actual.secuencia;
		}
		else 
		{
			// Hijo actFORWARD
			nodeN2 childForward = actual;
			childForward.st = apply(actFORWARD, actual.st, mapa);
			if (!Find(cerrados, childForward.st))
			{
				childForward.distancia += Distancia(actual, actFORWARD, mapa);
				// cout << "Distancia hijo childForward: " << childForward.distancia << endl;
				childForward.secuencia.push_back(actFORWARD);
				abiertos.push(childForward);
			}
			// Hijo actTURNL
			nodeN2 childTurnL = actual;
			childTurnL.st = apply(actTURN_L, actual.st, mapa);
			if (!Find(cerrados, childTurnL.st))
			{
				childTurnL.distancia += Distancia(actual, actTURN_L, mapa);
				// cout << "Distancia hijo childTurnL: " << childTurnL.distancia << endl;
				childTurnL.secuencia.push_back(actTURN_L);
				abiertos.push(childTurnL);
			}
			// Hijo actTURNR
			nodeN2 childTurnR = actual;
			childTurnR.st = apply(actTURN_R, actual.st, mapa);
			if (!Find(cerrados, childTurnR.st))
			{
				childTurnR.distancia += Distancia(actual, actTURN_R, mapa);
				// cout << "Distancia hijo childTurnR: " << childTurnR.distancia << endl;
				childTurnR.secuencia.push_back(actTURN_R);
				abiertos.push(childTurnR);
			}
		}
		if (print){
			cout << endl << "Cola abiertos " << endl;
			print_queue(abiertos);
			cout << "====================================" << endl << endl;
		}
		if (cont == 3 and print) {
			break;
		}
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
				current_node = frontier.front();
			}
		}
	}
	if (SolutionFound)
		plan = current_node.secuencia;
	return plan;
}

void MatrizAlInfinito(vector<vector<int>> &matriz)
{
	for (int i = 0; i < matriz.size(); i++)
		for (int j = 0; j < matriz[0].size(); j++)
			matriz[i][j] = INT32_MAX;
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
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 2) % 8);
			break;
		case actSON_TURN_SL:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 6) % 8);
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
		stResult.sonambulo.brujula = static_cast<Orientacion>((stResult.sonambulo.brujula+7)%8);
		break;
	case actSON_TURN_SR:
		stResult.sonambulo.brujula = static_cast<Orientacion>((stResult.sonambulo.brujula+1)%8);
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

bool Find(const list<nodeN2> &lista, const stateN0 &obj)
{
	auto it = lista.begin();
	while (it != lista.end() and !(it->st == obj))
		++it;
	return (!(it == lista.end()));
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
