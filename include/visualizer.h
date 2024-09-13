#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include "robot_arm.h"

class Visualizer {
public:
    Visualizer(int width, int height);
    void drawRobot(const std::vector<JointState>& joint_states);
    void saveToFile(const std::string& filename);
    void display();

private:
    sf::RenderWindow window;
    float scale;  // Now a variable, not a constant
    sf::Vector2f origin;
    const float link_width = 5.0f;  // Width of the link lines

    sf::Font font;

    void calculateScale(const std::vector<JointState>& joint_states);
    sf::Vector2f toScreenCoordinates(const Eigen::Vector2d& v);
    void drawGrid();
    void drawLink(const sf::Vector2f& start, const sf::Vector2f& end);
    void drawJoint(const sf::Vector2f& position, const std::string& name);
    void drawCoordinateFrame(const sf::Vector2f& position, const Eigen::Matrix2d& orientation, const std::string& name);
    void drawText(const std::string& text, const sf::Vector2f& position, float rotation = 0.0f);
    void drawAngle(const sf::Vector2f& position, float angle, const std::string& label);
};

#endif // VISUALIZER_H