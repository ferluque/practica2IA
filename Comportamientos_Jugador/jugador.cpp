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
			!(sigUbicacion.f == current_state.sonambulo.c and sigUbicacion.c == current_state.sonambulo.c))
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
