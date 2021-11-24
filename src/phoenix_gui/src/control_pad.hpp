#pragma once

#include "gamepad_thread.hpp"
#include <QtWidgets/QGroupBox>
#include <QtCore/QTimer>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <QtCore/QTimer>

struct ControllerCommand {};

class Ui_ControlPad;
class ControlPadScene;

class ControlPad : public QGroupBox {
    Q_OBJECT

public:
    ControlPad(QWidget *parent = nullptr);

    ~ControlPad();

    /**
     * @brief ノードの初期化処理を行う
     * @param node ローカルノード
     * @param remote_node_namespace リモートノードの名前空間
     */
    void initializeNode(rclcpp::Node::SharedPtr node, const std::string &remote_node_namespace);

    /**
     * @brief ノードの終了処理を行う
     */
    void uninitializeNode(void);

private:
    struct Command_t {
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
    };

    /**
     * @brief 入力を制限する
     * @param vx 並進速度のX成分
     * @param vy 並進速度のY成分
     * @param omega 角速度
     * @return 制限された入力値
     */
    static Command_t limitInput(double vx, double vy, double omega);

    virtual bool eventFilter(QObject *obj, QEvent *event) override;

    /**
     * @brief 速度指令を送る
     */
    Q_SLOT void transmitCommand(void);

    /// Qt Designerで作成したUI
    Ui_ControlPad *_ui = nullptr;

    /// ゲームパッド入力を処理するスレッド
    GamepadThread *_gamepad_thread = nullptr;

    /// 速度指令のパブリッシャ
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _velocity_publisher;

    /// 指令値の送信タイマー
    QTimer *_timer = nullptr;

    /// パッドに配置するシーン
    ControlPadScene *_scene;

    /// 送信する値
    bool _is_mouse_enabled = false;
    bool _is_gamepad_enabled = false;
    Command_t _mouse_command;

    /// 指令値の送信間隔 [ms]
    static constexpr int TRANSMIT_PERIOD = 20;

    /// 最大並進速度 [m/s]
    static constexpr double MAX_TRANSLATION = 10.0;

    /// 最大角速度 [rad/s]
    static constexpr double MAX_ROTATION = 10.0;
};
