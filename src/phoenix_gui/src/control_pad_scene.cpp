#include "control_pad_scene.hpp"
#include <QtCore/QtDebug>
#include <QtWidgets/QGraphicsSceneMouseEvent>
#include <QtWidgets/QGraphicsSceneWheelEvent>
#include <math.h>

static constexpr double PI = 3.141592653589793;

ControlPadScene::ControlPadScene(QObject *parent) : QGraphicsScene(parent), _wheel_average(5) {}

void ControlPadScene::setup(double max_x_value, double max_y_value, int x_tick_count, int y_tick_count) {
    _max_x_value = max_x_value;
    _max_y_value = max_y_value;
    _x_tick_count = x_tick_count;
    _y_tick_count = y_tick_count;

    // 軸を作成する
    _x_axis = addLine(0, 0, 0, 0, QPen(Qt::lightGray));
    _y_axis = addLine(0, 0, 0, 0, QPen(Qt::lightGray));

    // チックと円を作成する
    _circle = addEllipse(0, 0, 0, 0, QPen(Qt::lightGray));
    _xp_ticks.resize(_x_tick_count - 1);
    _xn_ticks.resize(_x_tick_count - 1);
    _yp_ticks.resize(_y_tick_count - 1);
    _yn_ticks.resize(_y_tick_count - 1);
    for (int index = 0; index < (_x_tick_count - 1); index++) {
        _xp_ticks[index] = addLine(0, 0, 0, -TICK_HEIGHT, QPen(Qt::lightGray));
        _xn_ticks[index] = addLine(0, 0, 0, -TICK_HEIGHT, QPen(Qt::lightGray));
    }
    for (int index = 0; index < (_x_tick_count - 1); index++) {
        _yp_ticks[index] = addLine(0, 0, TICK_HEIGHT, 0, QPen(Qt::lightGray));
        _yn_ticks[index] = addLine(0, 0, TICK_HEIGHT, 0, QPen(Qt::lightGray));
    }

    // 軌道を作成する
    _trajectory_line = addLine(0, 0, 0, 0, QPen(Qt::red, 3));
    _trajectory_line->setVisible(false);
    _trajectory_arc = addPath(QPainterPath(), QPen(Qt::red, 3));
    _trajectory_arc->setVisible(false);
    _trajectory_line_a = addLine(0, 0, 0, 0, QPen(Qt::red));
    _trajectory_line_b = addLine(0, 0, 0, 0, QPen(Qt::red));
    _trajectory_line_a->setVisible(false);
    _trajectory_line_b->setVisible(false);

    // 車体を作成する
    addEllipse(-CIRCLE_RADIUS, -CIRCLE_RADIUS, 2 * CIRCLE_RADIUS, 2 * CIRCLE_RADIUS, QPen(Qt::black));
    QPolygonF polygon;
    polygon << QPointF(0, -CIRCLE_RADIUS);
    polygon << QPointF(CIRCLE_RADIUS * sin(0.75), CIRCLE_RADIUS * cos(0.75));
    polygon << QPointF(0, CIRCLE_RADIUS * 0.5);
    polygon << QPointF(-CIRCLE_RADIUS * sin(0.75), CIRCLE_RADIUS * cos(0.75));
    addPolygon(polygon, QPen(Qt::black));

    // パッドのラベルを作成する
    _xp_labels.resize(_x_tick_count);
    _xn_labels.resize(_x_tick_count);
    _yp_labels.resize(_y_tick_count);
    _yn_labels.resize(_y_tick_count);
    for (int index = 0; index < _x_tick_count; index++) {
        double x = _max_x_value / _x_tick_count * (index + 1);
        _xp_labels[index] = addText(QString::number(x));
        _xn_labels[index] = addText(QString::number(-x));
        _xp_labels[index]->setVisible(false);
        _xn_labels[index]->setVisible(false);
        _max_label_size = std::max(_max_label_size, (double)_xn_labels[index]->boundingRect().width());
    }
    for (int index = 0; index < _y_tick_count; index++) {
        double y = _max_y_value / _y_tick_count * (index + 1);
        _yp_labels[index] = addText(QString::number(y));
        _yn_labels[index] = addText(QString::number(-y));
        _yp_labels[index]->setVisible(false);
        _yn_labels[index]->setVisible(false);
        _yp_labels[index]->setX(-_yp_labels[index]->boundingRect().width());
        _yn_labels[index]->setX(-_yn_labels[index]->boundingRect().width());
        _max_label_size = std::max(_max_label_size, (double)_yp_labels[index]->boundingRect().height());
    }
    _max_label_size *= 1.2;

    // タイマーを作成する
    _timer = new QTimer(this);
    _timer->setInterval(INTERVAL);
    connect(_timer, &QTimer::timeout, this, &ControlPadScene::filterWheelRotation);
}

void ControlPadScene::setEnabled(bool enabled) {
    if (enabled) {
        _timer->start();
    }
    else {
        _wheel = 0;
        _wheel_integrator = 0;
        _wheel_average.reset();
        _timer->stop();
        emit mouseStopped();
        _is_mouse_tracking = false;
    }
}

void ControlPadScene::setWheelSensitivity(double sensitivity) {
    _wheel_sensitivity = sensitivity;
}

void ControlPadScene::drawTrajectory(double vx, double vy, double omega) {
    QRectF rect = sceneRect();
    double w = rect.width() * 0.5;
    double scale_x = w * _x_tick_count / ((_x_tick_count + 1) * _max_x_value);
    double scale_y = w * _y_tick_count / ((_y_tick_count + 1) * _max_y_value);

    // double x = _pos.x() / w * _max_x_value * (_x_tick_count + 1) / _x_tick_count;
    // double y = -_pos.y() / w * _max_y_value * (_y_tick_count + 1) / _y_tick_count;

    double v = sqrt(vx * vx + vy * vy);
    // double omega = _wheel / 120.0 * _wheel_sensitivity;

    // 速度ベクトルを描画する
    _trajectory_line->setLine(0, 0, scale_x * vx, -scale_y * vy);
    _trajectory_line->setVisible(true);

    if ((1e-3 < abs(omega)) && (1e-3 < v)) {
        // 軌跡は円となる

        // 弧の中心座標を計算する
        double r = abs(v / omega);
        double cx, cy;
        if (0.0 < omega) {
            cx = -r * vy / v;
            cy = r * vx / v;
        }
        else {
            cx = r * vy / v;
            cy = -r * vx / v;
        }

        // 弧の位置と角度を計算し描画する
        QRect rect(scale_x * (cx - r), scale_y * (-cy - r), scale_x * 2.0 * r, scale_y * 2.0 * r);
        double start_angle = atan2(cy, cx) / PI * 180 - 180;
        double span_angle = v / r / PI * 180;
        if (omega < 0.0) {
            span_angle = -span_angle;
        }
        QPainterPath path;
        path.arcTo(rect, start_angle, span_angle);
        _trajectory_arc->setPath(path);
        _trajectory_line_a->setLine(0, 0, scale_x * cx, -scale_y * cy);
        _trajectory_line_b->setLine(scale_x * vx, -scale_y * vy, scale_x * cx, -scale_y * cy);


        /*double l = sqrt(r * r - v * v * 0.25);
        if (!isfinite(l)) {
            l = 0.0;
            r = v * 0.5;
            omega = (0 < _wheel) ? (v / r) : (-v / r);
        }
        double cx = x * 0.5;
        double cy = y * 0.5;
        if (0 < _wheel) {
            cx -= y / v * l;
            cy += x / v * l;
        }
        else {
            cx += y / v * l;
            cy -= x / v * l;
        }

        // 弧の位置と角度を計算し描画する
        QRect rect(scale_x * (cx - r), scale_y * (-cy - r), scale_x * 2.0 * r, scale_y * 2.0 * r);
        double start_angle = atan2(cy, cx) / PI * 180 - 180;
        double span_angle = acos(l / r) / PI * 360;
        if (_wheel < 0) {
            span_angle = -span_angle;
        }
        QPainterPath path;
        path.arcTo(rect, start_angle, span_angle);
        _trajectory_arc->setPath(path);

        // 弧の回転中心へ伸びる線分を描画する
        _trajectory_line_a->setLine(0, 0, scale_x * cx, -scale_y * cy);
        _trajectory_line_b->setLine(scale_x * vx, -scale_y * vy, scale_x * cx, -scale_y * cy);

        // 接線ベクトルを速度ベクトルとする
        double clen = sqrt(cx * cx + cy * cy);
        if (0 < _wheel) {
            _translation_x = v * cy / clen;
            _translation_y = -v * cx / clen;
        }
        else {
            _translation_x = -v * cy / clen;
            _translation_y = v * cx / clen;
        }*/

        _trajectory_arc->setVisible(true);
        _trajectory_line_a->setVisible(true);
        _trajectory_line_b->setVisible(true);
    }
    else {
        _trajectory_arc->setVisible(false);
        _trajectory_line_a->setVisible(false);
        _trajectory_line_b->setVisible(false);
    }
}

void ControlPadScene::clearTrajectory(void) {
    _trajectory_line->setVisible(false);
    _trajectory_arc->setVisible(false);
    _trajectory_line_a->setVisible(false);
    _trajectory_line_b->setVisible(false);
}

void ControlPadScene::resizeContent(int viewport_width, int viewport_height) {
    // パッドのグラフィックオブジェクトを再配置する
    double w = std::min(viewport_width, viewport_height) / 2;
    setSceneRect(-w, -w, 2.0 * w, 2.0 * w);

    // 軸を配置する
    _x_axis->setLine(-w, 0, w, 0);
    _y_axis->setLine(0, -w, 0, w);

    // チックと円を配置する
    _circle->setRect({-w * _x_tick_count / (_x_tick_count + 1), -w * _y_tick_count / (_y_tick_count + 1), 2.0 * w * _x_tick_count / (_x_tick_count + 1),
                      2.0 * w * _y_tick_count / (_y_tick_count + 1)});
    for (int index = 0; index < (_x_tick_count - 1); index++) {
        double x = w * (index + 1) / (_x_tick_count + 1);
        _xp_ticks[index]->setPos(x, 0);
        _xn_ticks[index]->setPos(-x, 0);
    }
    for (int index = 0; index < (_x_tick_count - 1); index++) {
        double y = w * (index + 1) / (_y_tick_count + 1);
        _yp_ticks[index]->setPos(0, -y);
        _yn_ticks[index]->setPos(0, y);
    }

    // ラベルを配置する
    // 最大値のラベルは必ず表示するが他のラベルはラベル間の感覚が狭い場合に非表示にする
    double pt_per_xlabel = w / (_x_tick_count + 1);
    int decimation_x = std::max((int)(_max_label_size / pt_per_xlabel), 1);
    for (int index = 0; index < _x_tick_count; index++) {
        double x = w * (index + 1) / (_x_tick_count + 1);
        _xp_labels[index]->setX(x - _xp_labels[index]->boundingRect().width() * 0.5);
        _xn_labels[index]->setX(-x - _xn_labels[index]->boundingRect().width() * 0.5);
        bool visible = (((_x_tick_count - 1 - index) % decimation_x) == 0) && (CIRCLE_RADIUS < (x - _max_label_size * 0.5));
        _xp_labels[index]->setVisible(visible);
        _xn_labels[index]->setVisible(visible);
    }
    double pt_per_ylabel = w / (_y_tick_count + 1);
    int decimation_y = std::max((int)(_max_label_size / pt_per_ylabel), 1);
    for (int index = 0; index < _y_tick_count; index++) {
        double y = w * (index + 1) / (_y_tick_count + 1);
        _yp_labels[index]->setY(-y - _yp_labels[index]->boundingRect().height() * 0.5);
        _yn_labels[index]->setY(y - _yn_labels[index]->boundingRect().height() * 0.5);
        bool visible = (((_y_tick_count - 1 - index) % decimation_y) == 0) && (CIRCLE_RADIUS < (y - _max_label_size * 0.5));
        _yp_labels[index]->setVisible(visible);
        _yn_labels[index]->setVisible(visible);
    }
}

void ControlPadScene::emitMousePos(void) {
    QRectF rect = sceneRect();
    double w = rect.width() * 0.5;
    double vx = _pos.x() / w * _max_x_value * (_x_tick_count + 1) / _x_tick_count;
    double vy = -_pos.y() / w * _max_y_value * (_y_tick_count + 1) / _y_tick_count;
    double omega = _wheel / 120.0 * _wheel_sensitivity;
    emit mouseUpdated(vx, vy, omega);
}

void ControlPadScene::filterWheelRotation(void) {
    int last_wheel = _wheel;
    _wheel = _wheel_average(_wheel_integrator);
    _wheel_integrator = 0;
    if ((_wheel != 0) || _is_mouse_tracking) {
        emitMousePos();
    }
    else if (last_wheel != 0) {
        emit mouseStopped();
    }
}

void ControlPadScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouse_event) {
    if (_is_mouse_tracking) {
        QPointF pos = mouse_event->scenePos();
        _pos = pos;
        emitMousePos();
    }
}

void ControlPadScene::mousePressEvent(QGraphicsSceneMouseEvent *mouse_event) {
    QPointF pos = mouse_event->scenePos();
    if (mouse_event->button() == Qt::LeftButton) {
        _is_mouse_tracking = true;
        _pos = pos;
        emitMousePos();
    }
}

void ControlPadScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouse_event) {
    QPointF pos = mouse_event->scenePos();
    if (mouse_event->button() == Qt::LeftButton) {
        _is_mouse_tracking = false;
        _pos = {0, 0};
        if (_wheel == 0) {
            emit mouseStopped();
        }
    }
}

void ControlPadScene::wheelEvent(QGraphicsSceneWheelEvent *wheel_event) {
    if (wheel_event->orientation() == Qt::Vertical) {
        _wheel_integrator += wheel_event->delta();
    }
}
