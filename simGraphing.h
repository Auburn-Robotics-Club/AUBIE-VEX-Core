#pragma once
#include "BaseGUI.h"
#include "robotmath.h"


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
