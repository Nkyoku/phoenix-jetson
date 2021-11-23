#pragma once

#include <vector>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsPathItem>
#include <QtWidgets/QGraphicsLineItem>
#include <QtWidgets/QGraphicsTextItem>
#include <QtCore/QTimer>

class ControlPadScene : public QGraphicsScene {
    Q_OBJECT

public:
    ControlPadScene(QObject *parent = nullptr);

    ~ControlPadScene() {}

    void setup(double max_x_value, double max_y_value, int x_tick_count, int y_tick_count);
    void resizeContent(int viewport_width, int viewport_height);
    void setEnabled(bool enabled);
    void setWheelSensitivity(double sensitivity);
    void drawTrajectory(double vx, double vy, double omega);
    void clearTrajectory(void);

    Q_SIGNAL void mouseUpdated(double vx, double vy, double omega);
    Q_SIGNAL void mouseStopped(void);

private:
    void emitMousePos(void);

    Q_SLOT void filterWheelRotation(void);

protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouse_event) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *mouse_event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouse_event) override;
    void wheelEvent(QGraphicsSceneWheelEvent *wheel_event) override;

private:
    bool _is_mouse_tracking = false;
    QTimer *_timer = nullptr;
    double _max_x_value = 0.0, _max_y_value = 0.0;
    int _x_tick_count = 0, _y_tick_count = 0;
    double _max_label_size = 0.0;
    QGraphicsLineItem *_x_axis, *_y_axis;
    std::vector<QGraphicsTextItem *> _xp_labels;
    std::vector<QGraphicsTextItem *> _xn_labels;
    std::vector<QGraphicsTextItem *> _yp_labels;
    std::vector<QGraphicsTextItem *> _yn_labels;
    std::vector<QGraphicsLineItem *> _xp_ticks;
    std::vector<QGraphicsLineItem *> _xn_ticks;
    std::vector<QGraphicsLineItem *> _yp_ticks;
    std::vector<QGraphicsLineItem *> _yn_ticks;
    QGraphicsEllipseItem *_circle;
    QGraphicsLineItem *_trajectory_line;
    QGraphicsPathItem *_trajectory_arc;
    QGraphicsLineItem *_trajectory_line_a;
    QGraphicsLineItem *_trajectory_line_b;

    int _wheel_integrator = 0;
    int _wheel = 0;
    QPointF _pos;
    double _wheel_sensitivity = 1.0;

    template<typename T>
    class MovingAverage {
    public:
        MovingAverage(size_t length) : _history(length) {}

        T operator()(T x) {
            _total = _total - _history[_pointer] + x;
            _history[_pointer] = x;
            _pointer = (_pointer + 1) % _history.size();
            return _total / static_cast<T>(_history.size());
        }

        void reset(void) {
            std::fill(_history.begin(), _history.end(), 0);
            _pointer = 0;
            _total = 0;
        }

    private:
        std::vector<T> _history;
        size_t _pointer = 0;
        T _total = 0;
    };

    MovingAverage<int> _wheel_average;

    static constexpr int INTERVAL = 20;
    static constexpr double CIRCLE_RADIUS = 15;
    static constexpr double TICK_HEIGHT = 5;
};
