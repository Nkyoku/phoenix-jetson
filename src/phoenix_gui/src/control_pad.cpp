#include "control_pad.hpp"
#include "ui_control_pad.h"
#include "common.hpp"
#include "../../phoenix/include/phoenix.hpp"
#include "node_thread.hpp"
#include "control_pad_scene.hpp"
#include <math.h>
#include <algorithm>

ControlPad::ControlPad(QWidget *parent) : QGroupBox(parent) {
    // UIを生成する
    _ui = new Ui_ControlPad;
    _ui->setupUi(this);
    _ui->padGraphics->viewport()->installEventFilter(this);

    // パッド上に表示するグラフィックオブジェクトを作成する
    //_ui->padGraphics->viewport()->grabGesture(Qt::PanGesture);
    //_ui->padGraphics->viewport()->grabGesture(Qt::PinchGesture);
    _scene = new ControlPadScene(this);
    _ui->padGraphics->setScene(_scene);
    _scene->setup(MAX_TRANSLATION, MAX_TRANSLATION, (int)MAX_TRANSLATION, (int)MAX_TRANSLATION);
    connect(this, &QGroupBox::toggled, [this](bool on) {
        _scene->setEnabled(on);
    });
    connect(_scene, &ControlPadScene::mouseUpdated, [this](double vx, double vy, double omega) {
        _is_mouse_enabled = true;
        _mouse_command = limitInput(vx, vy, omega);
        if (!_is_gamepad_enabled) {
            _scene->drawTrajectory(vx, vy, omega);
        }
    });
    connect(_scene, &ControlPadScene::mouseStopped, [this](void) {
        _is_mouse_enabled = false;
        _mouse_command = limitInput(0.0, 0.0, 0.0);
        if (!_is_gamepad_enabled) {
            _scene->clearTrajectory();
        }
    });

    // グループボックスのチェックボックスが変更されたときに送信タイマーの作成・削除を行う
    connect(this, &QGroupBox::toggled, [this](bool on) {
        if (on && (_timer == nullptr)) {
            _timer = new QTimer(this);
            connect(_timer, &QTimer::timeout, this, &ControlPad::transmitCommand);
            _timer->start(TRANSMIT_PERIOD);
        }
        else if (!on && (_timer != nullptr)) {
            delete _timer;
            _timer = nullptr;
            _is_mouse_enabled = false;
            _is_gamepad_enabled = false;
            _scene->clearTrajectory();
            if (GamepadThread::isSupported()) {
                _ui->gamepadCombobox->setCurrentIndex(0);
            }
        }
    });

    // ホイールを回したときの回転速度の感度の設定値を反映するようにする
    connect(_ui->sensitivitySlider, &QSlider::valueChanged, [this](int value) {
        _scene->setWheelSensitivity(value);
    });

    // 緊急停止ボタンが押されたら車体の制御を停止させるコマンドを送信し、
    // このコンポーネントのチェックを外してコマンド送信を停止する
    connect(_ui->stopButton, &QPushButton::clicked, [this](void) {
        if (_velocity_publisher) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = std::numeric_limits<double>::quiet_NaN();
            msg.linear.y = std::numeric_limits<double>::quiet_NaN();
            msg.angular.z = std::numeric_limits<double>::quiet_NaN();
            _velocity_publisher->publish(msg);
        }
        setChecked(false);
    });

    if (GamepadThread::isSupported()) {
        // ゲームパッドスレッドを作成する
        _ui->gamepadCombobox->addItem("None");
        _gamepad_thread = new GamepadThread();

        // ゲームパッドの接続・切断時にコンボボックスを操作する
        connect(_gamepad_thread, &GamepadThread::finished, _gamepad_thread, &QObject::deleteLater);
        connect(
            _gamepad_thread, &GamepadThread::gamepadConnected, this,
            [this](int device_id) {
                // 接続されたゲームパッドをコンボボックスに追加する
                _ui->gamepadCombobox->addItem(QString("XInput %1").arg(device_id), device_id);
            },
            Qt::QueuedConnection);
        connect(
            _gamepad_thread, &GamepadThread::gamepadDisconnected, this,
            [this](int device_id) {
                // 切断されたゲームパッドをコンボボックスから削除する
                int index = _ui->gamepadCombobox->findData(device_id);
                if (index == _ui->gamepadCombobox->currentIndex()) {
                    _ui->gamepadCombobox->setCurrentIndex(0);
                }
                if (0 <= index) {
                    _ui->gamepadCombobox->removeItem(index);
                }
            },
            Qt::QueuedConnection);

        _gamepad_thread->start();
    }
    else {
        _ui->gamepadCombobox->setEnabled(false);
        _ui->gamepadLabel->setEnabled(false);
    }
}

ControlPad::~ControlPad() {
    if (_gamepad_thread) {
        // ゲームパッドスレッドを終了する
        _gamepad_thread->requestInterruption();
        _gamepad_thread->quit();
        _gamepad_thread->wait(NodeThread::QUIT_TIMEOUT);
        _gamepad_thread = nullptr;
    }
}

void ControlPad::initializeNode(rclcpp::Node::SharedPtr node, const std::string &remote_node_namespace) {
    rclcpp::QoS qos(1);
    _velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>(constructName(remote_node_namespace, phoenix::TOPIC_NAME_COMMAND_VELOCITY), qos);
}

void ControlPad::uninitializeNode(void) {
    _velocity_publisher.reset();
}

ControlPad::Command_t ControlPad::limitInput(double vx, double vy, double omega) {
    Command_t result;
    double v = sqrt(vx * vx + vy * vy);
    if (v <= MAX_TRANSLATION) {
        result.vx = vx;
        result.vy = vy;
    }
    else {
        result.vx = vx / v * MAX_TRANSLATION;
        result.vy = vy / v * MAX_TRANSLATION;
    }
    result.omega = std::min(std::max(omega, -MAX_ROTATION), MAX_ROTATION);
    return result;
}

bool ControlPad::eventFilter(QObject *obj, QEvent *event) {
    const QWidget *viewport = _ui->padGraphics->viewport();
    if (obj == viewport) {
        if (event->type() == QEvent::Resize) {
            _scene->resizeContent(viewport->width(), viewport->height());
        }
        /*if (event->type() == QEvent::Gesture) {
            QGestureEvent *gesture_event = static_cast<QGestureEvent *>(event);
            if (QGesture *gesture = gesture_event->gesture(Qt::PanGesture)) {
                QPanGesture *pan = static_cast<QPanGesture *>(gesture);
                QSize size = viewport->size();
                _pad.velocity_scale_x = pan->offset().x() / (size.width() * 0.5);
                _pad.velocity_scale_y = -pan->offset().y() / (size.height() * 0.5);
                if ((pan->state() == Qt::GestureFinished) || (pan->state() == Qt::GestureCanceled)) {
                    _pad.velocity_scale_x = 0.0;
                    _pad.velocity_scale_y = 0.0;
                }
            }
            if (QGesture *gesture = gesture_event->gesture(Qt::PinchGesture)) {
                QPinchGesture *pinch = static_cast<QPinchGesture *>(gesture);
                QPinchGesture::ChangeFlags change_flags = pinch->changeFlags();
                if (change_flags & QPinchGesture::RotationAngleChanged) {
                    double d = pinch->rotationAngle() - pinch->lastRotationAngle();
                    if (d <= -180.0) {
                        d += 360.0;
                    }
                    else if (180.0 <= d) {
                        d -= 360.0;
                    }
                    _pad.velocity_scale_omega -= d;
                }
                if ((pinch->state() == Qt::GestureFinished) || (pinch->state() == Qt::GestureCanceled)) {
                    _pad.velocity_scale_omega = 0.0;
                }
            }
            return true;
        }
        else */
    }
    return false;
}

void ControlPad::transmitCommand(void) {
    bool gamepad_selected;
    int gamepad_device_id = _ui->gamepadCombobox->currentData().toInt(&gamepad_selected);
    if (gamepad_selected) {
        // ゲームパッドの入力値を送信する
        _is_gamepad_enabled = true;
        auto input_state = _gamepad_thread->inputState(gamepad_device_id);
        if (input_state) {
            geometry_msgs::msg::Twist msg;
            double sensitivity = (double)_ui->sensitivitySlider->value() / _ui->sensitivitySlider->maximum();
            msg.linear.x = sensitivity * MAX_TRANSLATION * input_state->left_stick_x;
            msg.linear.y = sensitivity * MAX_TRANSLATION * input_state->left_stick_y;
            msg.linear.z = -input_state->right_trigger;
            msg.angular.z = -sensitivity * MAX_ROTATION * input_state->right_stick_x;
            if (_velocity_publisher) {
                _velocity_publisher->publish(msg);
            }
            _scene->drawTrajectory(msg.linear.x, msg.linear.y, msg.angular.z);
            return;
        }
    }
    else if (_is_mouse_enabled) {
        // マウスの入力を送信する
        geometry_msgs::msg::Twist msg;
        msg.linear.x = _mouse_command.vx;
        msg.linear.y = _mouse_command.vy;
        msg.linear.z = 0.0;
        msg.angular.z = _mouse_command.omega;
        if (_velocity_publisher) {
            _velocity_publisher->publish(msg);
        }
        if (_is_gamepad_enabled) {
            _is_gamepad_enabled = false;
            _scene->drawTrajectory(msg.linear.x, msg.linear.y, msg.angular.z);
        }
    }
    else {
        if (_is_gamepad_enabled) {
            _is_gamepad_enabled = false;
            _scene->clearTrajectory();
        }
    }
}
