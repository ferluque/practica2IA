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
		current_state.sonambulo.f = sensores.SONposC;
		current_state.sonambulo.c = sensores.SONposC;
		current_state.sonambulo.brujula = sensores.sentido;
		goal.f = sensores.destinoF;
		goal.c = sensores.destinoC;

		hayPlan = AnchuraSoloJugador(current_state, goal, mapaResultado);
		if (hayPlan) {
			cout << "Se encontró un plan" << endl;
		}
	}
	if (hayPlan and plan.size() > 0)
	{
		cout << "Ejecutando siguiente acción del plan" << endl;
		accion = plan.front();
		plan.pop_front();
	}
	if (plan.size() == 0) {
		cout << "Se completó el plan" << endl;
		hayPlan = false;
	}
	return accion;
}

bool AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	stateN0 current_state = inicio;
	list<stateN0> frontier;
	list<stateN0> explored;
	frontier.push_back(current_state);
	bool SolutionFound = (current_state.jugador.f == final.f and current_state.jugador.c == final.c);
	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop_front();
		explored.push_back(current_state);
		// Hijo actFORWARD
		stateN0 childForward = apply(actFORWARD, current_state, mapa);
		if (childForward.jugador.f == final.f and childForward.jugador.c == final.c)
		{
			current_state = childForward;
			SolutionFound = true;
		}
		else if (!Find(frontier, childForward) and !Find(explored, childForward))
			frontier.push_back(childForward);
		if (!SolutionFound)
		{
			stateN0 childTurnl = apply(actTURN_L, current_state, mapa);
			if (!Find(frontier, childTurnl) and !Find(explored, childTurnl))
				frontier.push_back(childTurnl);
			stateN0 childTurnr = apply(actTURN_R, current_state, mapa);
			if (!Find(frontier, childTurnr) and !Find(explored, childTurnr))
				frontier.push_back(childTurnr);
		}

		if (!SolutionFound and !frontier.empty())
			current_state = frontier.front();
	}
	return SolutionFound;
}

stateN0 apply(Action action, const stateN0& current_state, const vector<vector<unsigned char>>& mapa) {
	stateN0 stResult = current_state;
	ubicacion sigUbicacion;
	switch (action) {
		case actFORWARD:
			sigUbicacion = NextCasilla(current_state.jugador);
			if (CasillaTransitable(sigUbicacion, mapa) and
			!(sigUbicacion.f == current_state.sonambulo.c and sigUbicacion.c == current_state.sonambulo.c)) {
				stResult.jugador = sigUbicacion;
			}
			break;
		case actTURN_L:
			stResult.jugador.brujula = static_cast<Orientacion>((stResult.jugador.brujula + 6) % 8);break;
			break;
		case actTURN_R:
			stResult.jugador.brujula = static_cast<Orientacion>((stResult.jugador.brujula + 2) % 8);break;	
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

bool Find(const list<stateN0>& lista, const stateN0& obj) {
	auto it = lista.begin();
	while (it!=lista.end() and !((*it)==obj))
		++it;
	return (!(it==lista.end()));
}

int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}
