#pragma once

#include "gamepad_thread.hpp"
#include <QtWidgets/QGroupBox>
#include <QtCore/QTimer>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

struct ControllerCommand {};

class Ui_ControlPad;
struct Graphics_ControlPad;
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

    /**
     * @brief 指令速度を取得する
     * @return 指令速度

    const geometry_msgs::msg::Twist &targetVelocity(void) const {
        return _target_velocity;
    } */

    // Q_SLOT void setPoseDisplay(const geometry_msgs::msg::Twist &twist, const geometry_msgs::msg::Pose &pose);

    // Q_SIGNAL void commandReady(void);

private:
    virtual bool eventFilter(QObject *obj, QEvent *event) override;

    // Q_SLOT void connectToGamepad(int index);

    Q_SLOT void transmitCommand(void);

    /// Qt Designerで作成したUI
    Ui_ControlPad *_ui = nullptr;

    /// ゲームパッド入力を処理するスレッド
    GamepadThread *_gamepad_thread = nullptr;

    /// 速度指令のパブリッシャ
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _velocity_publisher;

    /// 指令値の送信タイマー
    QTimer *_timer = nullptr;

    /// 指令速度
    geometry_msgs::msg::Twist _target_velocity;

    /// パッド上に表示するグラフィックオブジェクト
    Graphics_ControlPad *_graphcis = nullptr;

    struct {
        double velocity_scale_x;     // -1.0 ~ +1.0
        double velocity_scale_y;     // -1.0 ~ +1.0
        double velocity_scale_omega; // degree
    } _pad;

    /// パッドに配置するシーン
    ControlPadScene *_scene;

    /// 指令値の送信間隔[ms]
    static constexpr int TRANSMIT_PERIOD = 50;
};
