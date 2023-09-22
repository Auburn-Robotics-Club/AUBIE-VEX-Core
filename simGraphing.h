#pragma once
#include "robotmath/robotmath.h"


void plot(simulator::FieldGraph& graph, Point2d p, sf::Color color = simulator::colors::BLACK);
void scatter(simulator::FieldGraph& graph, Point2d p, sf::Color color = simulator::colors::BLACK);
void quiver(simulator::FieldGraph& graph, Point2d p, Vector2d v, sf::Color color = simulator::colors::BLACK);
void scatter(simulator::FieldGraph& graph, positionSet p, sf::Color color = simulator::colors::BLACK);
void quiver(simulator::FieldGraph& graph, positionSet p, sf::Color color = simulator::colors::BLACK, double mag = 6);

void plot(simulator::FieldGraph& graph, std::vector<Point2d>& p, sf::Color color = simulator::colors::BLACK);
void scatter(simulator::FieldGraph& graph, std::vector<Point2d>& p, sf::Color color = simulator::colors::BLACK);
void quiver(simulator::FieldGraph& graph, std::vector <Point2d>& p, std::vector <Vector2d>& v, sf::Color color = simulator::colors::BLACK);

void plot(simulator::FieldGraph& graph, std::vector <positionSet>& p, sf::Color color = simulator::colors::BLACK);
void scatter(simulator::FieldGraph& graph, std::vector <positionSet>& p, sf::Color color = simulator::colors::BLACK);
void quiver(simulator::FieldGraph& graph, std::vector <positionSet>& p, sf::Color color = simulator::colors::BLACK, double mag = 6);

void plot(simulator::FieldGraph& graph, Path& path, sf::Color color = simulator::colors::BLACK);
void scatter(simulator::FieldGraph& graph, Path& path, sf::Color color = simulator::colors::BLACK);
void quiver(simulator::FieldGraph& graph, Path& path, sf::Color color = simulator::colors::BLACK, double mag = 6);

void plot(simulator::FieldGraph& graph, Point2d p, sf::Color color) {
	graph.plot(p.x, p.y, color);
}

void scatter(simulator::FieldGraph& graph, Point2d p, sf::Color color) {
	graph.scatter(p.x, p.y, color);
};

void quiver(simulator::FieldGraph& graph, Point2d p, Vector2d v, sf::Color color) {
	graph.quiver(p.x, p.y, v.getX(), v.getY(), color);
};

void scatter(simulator::FieldGraph& graph, positionSet p, sf::Color color) {
	scatter(graph, p.p, color);
};

void quiver(simulator::FieldGraph& graph, positionSet p, sf::Color color, double mag) {
	quiver(graph, p.p, Vector2d(mag, p.head, false), color);
};

void plot(simulator::FieldGraph& graph, std::vector<Point2d>& p, sf::Color color) {
	graph.clearPlotBuffer();
	for (int i = 0; i < p.size(); i++) {
		graph.plot(p[i].x, p[i].y, color);
	}
};

void scatter(simulator::FieldGraph& graph, std::vector<Point2d>& p, sf::Color color) {
	for (int i = 0; i < p.size(); i++) {
		scatter(graph, p[i], color);
	}
};

void quiver(simulator::FieldGraph& graph, std::vector <Point2d>& p, std::vector <Vector2d>& v, sf::Color color) {
	if (p.size() <= v.size()) {
		for (int i = 0; i < p.size(); i++) {
			quiver(graph, p[i], v[i], color);
		}
	} else {
		for (int i = 0; i < v.size(); i++) {
			quiver(graph, p[i], v[i], color);
		}
		for (int i = v.size(); i < p.size(); i++) {
			scatter(graph, p[i], color);
		}
	}
};

void plot(simulator::FieldGraph& graph, std::vector <positionSet>& p, sf::Color color) {
	graph.clearPlotBuffer();
	for (int i = 0; i < p.size(); i++) {
		graph.plot(p[i].p.x, p[i].p.y, color);
	}
};

void scatter(simulator::FieldGraph& graph, std::vector <positionSet>& p, sf::Color color) {
	for (int i = 0; i < p.size(); i++) {
		scatter(graph, p[i].p, color);
	}
};

void quiver(simulator::FieldGraph& graph, std::vector <positionSet>& p, sf::Color color, double mag) {
	std::vector<double> x, y, a, b;
	for (int i = 0; i < p.size(); i++) {
		quiver(graph, p[i], color, mag);
	}
};

void plot(simulator::FieldGraph& graph, Path& path, sf::Color color) {
	plot(graph, path.getList(), color);
};
void scatter(simulator::FieldGraph& graph, Path& path, sf::Color color) {
	scatter(graph, path.getList(), color);
};
void quiver(simulator::FieldGraph& graph, Path& path, sf::Color color, double mag) {
	quiver(graph, path.getList(), color, mag);
};
