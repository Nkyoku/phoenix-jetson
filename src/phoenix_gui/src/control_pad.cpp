#include "control_pad.hpp"
#include "ui_control_pad.h"
#include "common.hpp"
#include "../../phoenix/include/phoenix.hpp"
#include "node_thread.hpp"
#include "control_pad_scene.hpp"

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
    _scene->setup(10.0, 10.0, 10, 10);
    connect(this, &QGroupBox::toggled, [this](bool on) {
        _scene->setEnabled(on);
    });
    connect(_scene, &ControlPadScene::mouseUpdated, [this](double vx, double vy, double omega) {
        _scene->drawTrajectory(vx, vy, omega);
    });
    connect(_scene, &ControlPadScene::mouseStopped, [this](void) {
        _scene->clearTrajectory();
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
            transmitCommand();
        }
    });

    // connect(this, &ControlPad::commandReady, this, &ControlPad::sendCommand);

    // ホイールを回したときの回転速度の感度の設定値を反映するようにする
    connect(_ui->sensitivitySlider, &QSlider::valueChanged, [this](int value) {
        _scene->setWheelSensitivity(value);
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
    delete _graphcis;
}

void ControlPad::initializeNode(rclcpp::Node::SharedPtr node, const std::string &remote_node_namespace) {
    rclcpp::QoS qos(1);
    _velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>(constructName(remote_node_namespace, phoenix::TOPIC_NAME_COMMAND_VELOCITY), qos);
}

void ControlPad::uninitializeNode(void) {}

bool ControlPad::eventFilter(QObject *obj, QEvent *event) {
    const QWidget *viewport = _ui->padGraphics->viewport();
    if (obj == viewport) {
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
        if (event->type() == QEvent::Resize) {
            _scene->resizeContent(viewport->width(), viewport->height());
        }
    }
    return false;
}

void ControlPad::transmitCommand(void) {
    bool gamepad_selected;
    int gamepad_device_id = _ui->gamepadCombobox->currentData().toInt(&gamepad_selected);
    if (gamepad_selected) {
        auto input_state = _gamepad_thread->inputState(gamepad_device_id);
        if (input_state) {
            _target_velocity.linear.x = 4.0f * input_state->left_stick_x;
            _target_velocity.linear.y = 4.0f * input_state->left_stick_y;
            _target_velocity.linear.z = -input_state->right_trigger;
            _target_velocity.angular.z = -10.0f * input_state->right_stick_x;
            // emit commandReady();
        }
    }
}

/*void ControlPad::sendCommand(void) {
    if (_publishers.velocity) {
        geometry_msgs::msg::Twist msg = _Ui->controllerGroup->targetVelocity();
        _publishers.velocity->publish(msg);
    }
}*/
