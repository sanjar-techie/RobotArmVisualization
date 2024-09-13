#include "visualizer.h"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <sstream>

Visualizer::Visualizer(int width, int height)
    : window(sf::VideoMode(width, height), "Robot Arm Visualization"),
      origin(width / 2.0f, height / 2.0f) {
    window.clear(sf::Color::White);
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        std::cerr << "Error loading font. Trying fallback font..." << std::endl;
        if (!font.loadFromFile("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf")) {
            std::cerr << "Error loading fallback font. Text will not be displayed." << std::endl;
        }
    }
}

void Visualizer::drawRobot(const std::vector<JointState>& joint_states) {
    window.clear(sf::Color::White);

    // Draw grid
    drawGrid();

    if (joint_states.size() >= 3) {
        // Draw link 1 (A to B)
        sf::Vector2f posA = toScreenCoordinates(joint_states[0].position);  // A is at the origin
        sf::Vector2f posB = toScreenCoordinates(joint_states[1].position);
        drawLink(posA, posB);

        // Draw link 2 (B to E)
        sf::Vector2f posE = toScreenCoordinates(joint_states[2].position);
        drawLink(posB, posE);

        // Draw joints and coordinate frames
        drawJoint(posA, "W/A");  // World frame and robot base are at the same point
        drawCoordinateFrame(posA, joint_states[0].orientation, "W/A");

        drawJoint(posB, "B");
        drawCoordinateFrame(posB, joint_states[1].orientation, "B");

        drawJoint(posE, "E");
        drawCoordinateFrame(posE, joint_states[2].orientation, "E");

        // Draw joint angles
        // Angle at A (between vertical and first link)
        Eigen::Vector2d diffAB = joint_states[1].position - joint_states[0].position;
        float angleA = std::atan2(diffAB.x(), diffAB.y());  // Angle from vertical
        std::ostringstream ossA;
        ossA << "θ1 = " << std::fixed << std::setprecision(2) << angleA;
        drawAngle(posA, angleA, ossA.str());

        // Angle at B (between first and second link)
        Eigen::Vector2d diffBE = joint_states[2].position - joint_states[1].position;
        float angleB = std::atan2(diffBE.y(), diffBE.x()) - std::atan2(diffAB.y(), diffAB.x());
        std::ostringstream ossB;
        ossB << "θ2 = " << std::fixed << std::setprecision(2) << angleB;
        drawAngle(posB, angleB, ossB.str());
    }

    window.display();
}

void Visualizer::saveToFile(const std::string& filename) {
    sf::Texture texture;
    texture.create(window.getSize().x, window.getSize().y);
    texture.update(window);
    sf::Image screenshot = texture.copyToImage();
    if (screenshot.saveToFile(filename)) {
        std::cout << "Screenshot saved to " << filename << std::endl;
    } else {
        std::cerr << "Failed to save screenshot" << std::endl;
    }
}

void Visualizer::display() {
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
    }
}

sf::Vector2f Visualizer::toScreenCoordinates(const Eigen::Vector2d& v) {
    return sf::Vector2f(origin.x + v.x() * scale, origin.y - v.y() * scale);
}

void Visualizer::drawGrid() {
    const int gridSize = 10;
    const float cellSize = scale / 2;  // Half-unit grid cells

    for (int i = -gridSize; i <= gridSize; ++i) {
        float pos = i * cellSize;

        sf::Vertex hLine[] = {
            sf::Vertex(sf::Vector2f(origin.x - gridSize * cellSize, origin.y + pos), sf::Color(200, 200, 200)),
            sf::Vertex(sf::Vector2f(origin.x + gridSize * cellSize, origin.y + pos), sf::Color(200, 200, 200))
        };
        window.draw(hLine, 2, sf::Lines);

        sf::Vertex vLine[] = {
            sf::Vertex(sf::Vector2f(origin.x + pos, origin.y - gridSize * cellSize), sf::Color(200, 200, 200)),
            sf::Vertex(sf::Vector2f(origin.x + pos, origin.y + gridSize * cellSize), sf::Color(200, 200, 200))
        };
        window.draw(vLine, 2, sf::Lines);

        // Draw grid numbers
        if (i != 0 && i % 2 == 0) {  // Draw numbers every whole unit
            drawText(std::to_string(i/2), sf::Vector2f(origin.x + pos, origin.y + 10));
            drawText(std::to_string(-i/2), sf::Vector2f(origin.x - 10, origin.y + pos));
        }
    }

    // Draw x and y axis
    sf::Vertex xAxis[] = {
        sf::Vertex(sf::Vector2f(0, origin.y), sf::Color::Black),
        sf::Vertex(sf::Vector2f(window.getSize().x, origin.y), sf::Color::Black)
    };
    window.draw(xAxis, 2, sf::Lines);

    sf::Vertex yAxis[] = {
        sf::Vertex(sf::Vector2f(origin.x, 0), sf::Color::Black),
        sf::Vertex(sf::Vector2f(origin.x, window.getSize().y), sf::Color::Black)
    };
    window.draw(yAxis, 2, sf::Lines);
}

void Visualizer::drawLink(const sf::Vector2f& start, const sf::Vector2f& end) {
    sf::Vector2f direction = end - start;
    sf::Vector2f unitDirection = direction / std::sqrt(direction.x * direction.x + direction.y * direction.y);
    sf::Vector2f unitPerpendicular(-unitDirection.y, unitDirection.x);

    sf::ConvexShape line;
    line.setPointCount(4);
    line.setPoint(0, start + unitPerpendicular * 5.0f);
    line.setPoint(1, start - unitPerpendicular * 5.0f);
    line.setPoint(2, end - unitPerpendicular * 5.0f);
    line.setPoint(3, end + unitPerpendicular * 5.0f);

    line.setFillColor(sf::Color::Black);
    window.draw(line);
}

void Visualizer::drawJoint(const sf::Vector2f& position, const std::string& name) {
    sf::CircleShape joint(10);  // Increased from 5 to 10
    joint.setFillColor(sf::Color::Red);
    joint.setOrigin(10, 10);  // Adjusted to match new size
    joint.setPosition(position);
    window.draw(joint);
    
    drawText(name, position + sf::Vector2f(15, 15));
}

void Visualizer::drawCoordinateFrame(const sf::Vector2f& position, const Eigen::Matrix2d& orientation, const std::string& name) {
    sf::Vector2f x_axis(orientation(0, 0), -orientation(1, 0));  // Negate y because SFML y-axis points down
    sf::Vector2f y_axis(orientation(0, 1), -orientation(1, 1));

    sf::Vertex x_line[] = {
        sf::Vertex(position, sf::Color::Red),
        sf::Vertex(position + x_axis * 40.0f, sf::Color::Red)
    };
    sf::Vertex y_line[] = {
        sf::Vertex(position, sf::Color::Green),
        sf::Vertex(position + y_axis * 40.0f, sf::Color::Green)
    };

    window.draw(x_line, 2, sf::Lines);
    window.draw(y_line, 2, sf::Lines);

    drawText("", position + x_axis * 45.0f);
    drawText("", position + y_axis * 45.0f);
}

void Visualizer::drawText(const std::string& text, const sf::Vector2f& position, float rotation) {
    sf::Text sfText(text, font, 16);
    sfText.setFillColor(sf::Color::Black);
    sfText.setPosition(position);
    sfText.setRotation(rotation);
    window.draw(sfText);
}

void Visualizer::drawAngle(const sf::Vector2f& position, float angle, const std::string& label) {
    float radius = 30.0f;
    sf::CircleShape arc(radius);
    arc.setFillColor(sf::Color::Transparent);
    arc.setOutlineColor(sf::Color::Blue);
    arc.setOutlineThickness(2);
    arc.setOrigin(radius, radius);
    arc.setPosition(position);
    window.draw(arc);

    // Draw angle arc
    sf::ConvexShape angleArc;
    angleArc.setPointCount(30);
    for (int i = 0; i < 30; ++i) {
        float t = i / 29.0f;
        float arcAngle = t * angle;
        float x = radius * std::cos(arcAngle);
        float y = -radius * std::sin(arcAngle);  // Negative because SFML y-axis points down
        angleArc.setPoint(i, sf::Vector2f(x, y));
    }
    angleArc.setFillColor(sf::Color(0, 0, 255, 64));  // Semi-transparent blue
    angleArc.setPosition(position);
    window.draw(angleArc);

    sf::Vector2f labelPos = position + sf::Vector2f(std::cos(angle/2) * radius * 1.5f, -std::sin(angle/2) * radius * 1.5f);
    drawText(label, labelPos);
}