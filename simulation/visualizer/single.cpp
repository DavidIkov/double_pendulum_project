
#include "pendulum/single.hpp"

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QTimer>
#include <QWidget>
#include <chrono>
#include <cmath>
#include <random>

constexpr std::chrono::milliseconds frame_time_length(16);
constexpr uint32_t simulation_substeps = 10;
constexpr float simulation_speed = 1.f;
constexpr float scene_zoom = 0.4f;

class SimulationWindow : public QWidget {
public:
    SimulationWindow(Simulation<1>&& simulation)
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

    Simulation<1> const& GetSimulation() const { return simulation_; }

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
        mathcpp::Vector2F p1 = simulation_.GetPendulumPosition();

        mathcpp::Vector2F platform_pos{std::sin(timer) * 2,
                                       std::sin(timer * 2) * 2};

        p0 += platform_pos, p1 += platform_pos;

        painter.drawLine(QLineF(p0[0], p0[1], p1[0], p1[1]));

        for (size_t step = 0; step < simulation_substeps; ++step)
            simulation_.Step(
                frame_time_length.count() / 1000.f / simulation_substeps *
                    simulation_speed,
                {-std::sin(timer) * 2, -std::sin(timer * 2) * 4 * 2});
    }
    void Render() {
        update();
        timer += frame_time_length.count() / 1000.f * simulation_speed;
    }

private:
    float timer = 0;
    Simulation<1> simulation_;
};

int main(int argc, char** argv) {
    std::mt19937 rand_gen((std::random_device())());
    std::uniform_real_distribution<float> angle_distr(M_PI, -M_PI),
        length_distr(0.5, 1.5), mass_distr(2, 20), damping_distr(0, 0.3f),
        noise_distr(0, 0.1f);

    QApplication app(argc, argv);
    std::setlocale(LC_NUMERIC, "C");

    SimulationWindow simulation_wind(
        Simulation<1>({angle_distr(rand_gen), length_distr(rand_gen),
                       mass_distr(rand_gen), damping_distr(rand_gen)}));
    simulation_wind.resize(800, 600);
    simulation_wind.show();

    float radius = simulation_wind.GetSimulation().GetMaxRadius();

    {
        Pendulum const& pend = simulation_wind.GetSimulation().GetPendulum();
        printf(
            "Starting conditions:\nPendulum0: angle = %f, length = %f, "
            "mass = "
            "%f, damping = %f\n",
            pend.angle_, pend.length_, pend.mass_, pend.dumping_mult_);
    }

    return app.exec();
}
