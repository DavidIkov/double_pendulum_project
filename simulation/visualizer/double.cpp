
#include "pendulum/double.hpp"

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QTimer>
#include <QWidget>
#include <chrono>
#include <cmath>
#include <random>

constexpr std::chrono::milliseconds frame_time_length(16);
constexpr uint32_t simulation_substeps = 50;
constexpr float simulation_speed = 1.f;
constexpr float scene_zoom = 0.2f;

class SimulationWindow : public QWidget {
public:
    SimulationWindow(Simulation<2>&& simulation)
        : simulation_(std::move(simulation)) {
        QTimer* renderingTimer = new QTimer(this);
        connect(renderingTimer, &QTimer::timeout, this,
                &SimulationWindow::Render);
        renderingTimer->start(frame_time_length);

        QPalette palete = palette();
        palete.setColor(QPalette::Window, QColor(122, 22, 77));
        setPalette(palete);
        setAutoFillBackground(true);
    }

    Simulation<2> const& GetSimulation() const { return simulation_; }

protected:
    void paintEvent(QPaintEvent* event) override {
        QPainter painter(this);

        QPen pen(Qt::black);
        pen.setWidthF(0.01f / scene_zoom);
        painter.setPen(pen);

        painter.translate(width() / 2.f, height() / 2.f);
        painter.scale(height() / 2.f * scene_zoom,
                      -height() / 2.f * scene_zoom);

        mathcpp::Vector2F p0{0, 0};
        mathcpp::Vector2F p1 = simulation_.GetPendulumPosition(0);
        mathcpp::Vector2F p2 = simulation_.GetPendulumPosition(1);

        mathcpp::Vector2F platform_pos{std::sin(timer) * 2,
                                       std::sin(timer * 2) * 2};

        p0 += platform_pos, p1 += platform_pos, p2 += platform_pos;

        painter.drawLine(QLineF(p0[0], p0[1], p1[0], p1[1]));
        painter.drawLine(QLineF(p1[0], p1[1], p2[0], p2[1]));

        for (size_t step = 0; step < simulation_substeps; ++step)
            simulation_.Step(
                frame_time_length.count() / 1000.f / simulation_substeps *
                    simulation_speed,
                {std::cos(timer) * 2, std::cos(timer * 2) * 2 * 2,
                 -std::sin(timer) * 2, -std::sin(timer * 2) * 4 * 2});
    }
    void Render() {
        update();
        timer += frame_time_length.count() / 1000.f * simulation_speed;
    }

private:
    float timer = 0;
    Simulation<2> simulation_;
};

int main(int argc, char** argv) {
    std::mt19937 rand_gen((std::random_device())());
    std::uniform_real_distribution<float> angle_distr(M_PI, -M_PI),
        length_distr(0.5, 1.5), mass_distr(2, 20), damping_distr(0, 0.3f),
        noise_distr(0, 0.1f);

    QApplication app(argc, argv);
    std::setlocale(LC_NUMERIC, "C");

    SimulationWindow simulation_wind(
        Simulation<2>{{angle_distr(rand_gen), length_distr(rand_gen),
                       mass_distr(rand_gen), damping_distr(rand_gen)},
                      {angle_distr(rand_gen), length_distr(rand_gen),
                       mass_distr(rand_gen), damping_distr(rand_gen)}});
    simulation_wind.resize(800, 600);
    simulation_wind.show();

    float radius = simulation_wind.GetSimulation().GetMaxRadius();

    {
        auto& pends = simulation_wind.GetSimulation().GetPendulums();
        printf(
            "Starting conditions:\nPendulum0: angle = %f, length = %f, mass = "
            "%f, damping = %f\nPendulum1: angle = %f, length = %f, mass = %f, "
            "damping = %f\n",
            pends[0].angle_, pends[0].length_, pends[0].mass_,
            pends[0].dumping_mult_, pends[1].angle_, pends[1].length_,
            pends[1].mass_, pends[1].dumping_mult_);
    }

    return app.exec();
}
