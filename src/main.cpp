#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>

struct Object {

    sf::Vector2f pos, last_pos;
    float radius;
    sf::Color color = sf::Color::Green;

    Object() = default;

    Object(sf::Vector2f position_, float radius_)
        : pos{ position_ }
        , last_pos{ position_ }
        , radius{ radius_ }
    {}
    void applyGravity(sf::Vector2f g, float dt) {
        sf::Vector2f t_pos = pos;
        pos = 2.0f * pos - last_pos + g * dt * dt; //Verlet intregation
        last_pos = t_pos;
    }

    void applyCollisions(std::vector<Object> obj_array) {
        float rep_coeff = 1.0f;
        uint64_t n = static_cast<uint64_t>(obj_array.size());
        for (uint64_t j = 0; j < n; ++j) {
            Object obj2 = obj_array[j];
            if (obj2.pos != pos) {
                float max_dis = obj2.radius + radius;
                sf::Vector2f x_12 = pos - obj2.pos;
                float dis = sqrtf(x_12.x * x_12.x + x_12.y * x_12.y);
                if (dis < max_dis) {
                    sf::Vector2f x_hat = x_12 / dis;
                    obj2.pos -= rep_coeff * radius / (radius + obj2.radius) * x_hat;
                    pos += rep_coeff * obj2.radius / (radius + obj2.radius) * x_hat;
                }
            }
        }
    }
    sf::CircleShape getShape() {
        sf::CircleShape s(radius);
        s.setFillColor(color);
        s.setOrigin(radius, radius);
        s.setPosition(pos);
        return s;
    }
};

class Solver {
public:
    Solver() = default;
    Object& addobject(sf::Vector2f pos, float radius) {
        return obj_array.emplace_back(pos, radius);
    }
    std::vector<Object> get_objects() {
        return obj_array;
    }

    void applyConstraints(sf::CircleShape constraint) {
        for (auto& obj : obj_array) {
            float min_dis = constraint.getRadius() - obj.radius;
            sf::Vector2f dis_o = constraint.getPosition() - obj.pos;
            float dis = sqrtf(dis_o.x * dis_o.x + dis_o.y * dis_o.y);
            if (dis > min_dis) {
                obj.pos = -((min_dis / dis) * dis_o) + constraint.getPosition();
            }
        }
    }

    void update() {
        for (auto& obj : obj_array) {
            obj.applyGravity(g, dt);
            obj.applyCollisions(get_objects());
        }

    }
    uint64_t getObjectCount() {
        return static_cast<uint64_t>(obj_array.size());
    }

private:
    std::vector<Object>         obj_array;
    sf::Vector2f                last_pos;
    float                       dt = 0.0001f;
    sf::Vector2f                g = 900000.0f * sf::Vector2f(0.0f, 1.0f);
    uint32_t                    sub_steps = 1;


};

sf::CircleShape setConstraint(float constraint_r) {
    sf::CircleShape constraint(constraint_r);
    constraint.setFillColor(sf::Color::Black);
    constraint.setPointCount(128);
    constraint.setOrigin(250.0f, 250.0f);
    constraint.setPosition(350.0f, 350.0f);
    return constraint;
};


int main() {
    float constraint_r = 250.0f;
    sf::RenderWindow window(sf::VideoMode(1000, 800), "Fijicks Simulator (Bootleg)");
    window.setFramerateLimit(60);
    Solver s;
    uint64_t max_object_count = 1750;
    sf::Clock clock;
    const float object_spawn_delay = 0.025f;
    sf::Vector2f init_pos = sf::Vector2f(250.0f, 350.0f);
    float r = 5.0f;
    // run the program as long as the window is open
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {

            if (event.type == sf::Event::Closed) {
                window.close();
            }


        }

        if (s.getObjectCount() < max_object_count && clock.getElapsedTime().asSeconds() >= object_spawn_delay) {
            clock.restart();
            float dx = 0.1f + init_pos.x;
            float dy = init_pos.y;
            init_pos = sf::Vector2f(dx, dy);
            s.addobject(init_pos, r);

        }

        window.clear(sf::Color::White);

        // draw everything here...
        window.draw(setConstraint(250.0f));

        auto obj_array = s.get_objects();

        s.applyConstraints(setConstraint(250.0f));
        s.update();
        for (auto& obj : obj_array) {
            window.draw(obj.getShape());
        }

        // end the current frame
        window.display();
    }
    return 0;
}