#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <SFML/Graphics.hpp>
#include <vector>
#include "robot_arm.h"

class Visualizer {
public:
    Visualizer(int width, int height);
    void drawRobot(const std::vector<JointState>& joint_states);
    void saveToFile(const std::string& filename);

private:
    sf::RenderWindow window;
    const float scale = 100.0f;  // pixels per unit
    const sf::Vector2f origin;

    sf::Vector2f toScreenCoordinates(const Eigen::Vector2d& v);
    void drawLink(const sf::Vector2f& start, const sf::Vector2f& end);
    void drawJoint(const sf::Vector2f& position);
    void drawCoordinateFrame(const sf::Vector2f& position, const Eigen::Matrix2d& orientation);
};

#endif // VISUALIZER_H