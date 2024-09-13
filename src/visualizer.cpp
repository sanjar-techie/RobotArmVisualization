#include "visualizer.h"
#include <iostream>

Visualizer::Visualizer(int width, int height)
    : window(sf::VideoMode(width, height), "Robot Arm Visualization"),
      origin(width / 2.0f, height / 2.0f) {
    window.clear(sf::Color::White);
}

void Visualizer::drawRobot(const std::vector<JointState>& joint_states) {
    window.clear(sf::Color::White);

    // Draw links
    for (size_t i = 1; i < joint_states.size(); ++i) {
        sf::Vector2f start = toScreenCoordinates(joint_states[i-1].position);
        sf::Vector2f end = toScreenCoordinates(joint_states[i].position);
        drawLink(start, end);
    }

    // Draw joints and coordinate frames
    for (const auto& state : joint_states) {
        sf::Vector2f pos = toScreenCoordinates(state.position);
        drawJoint(pos);
        drawCoordinateFrame(pos, state.orientation);
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

sf::Vector2f Visualizer::toScreenCoordinates(const Eigen::Vector2d& v) {
    return sf::Vector2f(origin.x + v.x() * scale, origin.y - v.y() * scale);
}

void Visualizer::drawLink(const sf::Vector2f& start, const sf::Vector2f& end) {
    sf::Vertex line[] = {
        sf::Vertex(start, sf::Color::Black),
        sf::Vertex(end, sf::Color::Black)
    };
    window.draw(line, 2, sf::Lines);
}

void Visualizer::drawJoint(const sf::Vector2f& position) {
    sf::CircleShape joint(5);
    joint.setFillColor(sf::Color::Red);
    joint.setOrigin(5, 5);
    joint.setPosition(position);
    window.draw(joint);
}

void Visualizer::drawCoordinateFrame(const sf::Vector2f& position, const Eigen::Matrix2d& orientation) {
    sf::Vector2f x_axis(orientation(0, 0), orientation(1, 0));
    sf::Vector2f y_axis(orientation(0, 1), orientation(1, 1));

    sf::Vertex x_line[] = {
        sf::Vertex(position, sf::Color::Red),
        sf::Vertex(position + x_axis * 20.0f, sf::Color::Red)
    };
    sf::Vertex y_line[] = {
        sf::Vertex(position, sf::Color::Green),
        sf::Vertex(position + y_axis * 20.0f, sf::Color::Green)
    };

    window.draw(x_line, 2, sf::Lines);
    window.draw(y_line, 2, sf::Lines);
}