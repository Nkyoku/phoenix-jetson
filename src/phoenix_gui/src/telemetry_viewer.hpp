#pragma once

#include <QtWidgets/QGroupBox>
#include <QtWidgets/QTreeWidgetItem>
#include <QtCore/QFile>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <phoenix_msgs/msg/stream_data_adc2.hpp>
#include <phoenix_msgs/msg/stream_data_motion.hpp>

class Ui_TelemetryViewer;

class TelemetryViewer : public QGroupBox {
    Q_OBJECT

public:
    TelemetryViewer(QWidget *parent = nullptr);

    ~TelemetryViewer();

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
    /**
     * @brief ツリーを更新する 
     */
    Q_SLOT void updateTelemertyTreeItems(void);

    /**
     * @brief ロギングを開始する
     */
    Q_SLOT void startLogging(void);

    /**
     * @brief ロギングを停止する
     */
    Q_SLOT void stopLogging(void);

    /**
     * @brief ツリーに項目を作成する
     */
    void generateTelemetryTreeItems(void);

    /// Qt Designerで作成したUI
    Ui_TelemetryViewer *_ui = nullptr;

    /// telemetryTreeに表示する項目の定義
    struct TreeItems_t {
        struct Battery_t {
            QTreeWidgetItem *present, *voltage, *current, *temperature;
        } battery;
        struct Adc2_t {
            QTreeWidgetItem *dc48v_voltage, *dribble_voltage, *dribble_current;
        } adc2;
        struct Motion_t {
            QTreeWidgetItem *accelerometer[3], *gyroscope[3], *gravity[3], *body_acceleration[3], *body_velocity[3];
            QTreeWidgetItem *wheel_velocity[4], *wheel_current_d[4], *wheel_current_q[4];
        } motion;
        struct Control_t {
            QTreeWidgetItem *perf_counter, *wheel_current_ref[4], *body_ref_accel[4];
        } control;
    };

    /// telemetryTreeに表示する項目
    TreeItems_t _tree_items;

    /// テレメトリのログを保存するファイル
    std::shared_ptr<QFile> _log_file;

    /// テレメトリのログに含まれるフレーム番号
    uint32_t _frame_number = 0;

    /// サブスクライバ
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _battery_subscription;
    rclcpp::Subscription<phoenix_msgs::msg::StreamDataAdc2>::SharedPtr _adc2_subscription;
    rclcpp::Subscription<phoenix_msgs::msg::StreamDataMotion>::SharedPtr _motion_subscription;

    /// 受信した最後のメッセージ
    std::shared_ptr<sensor_msgs::msg::BatteryState> _battery_message;
    std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> _adc2_message;
    std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> _motion_message;

    /// telemetryTreeで値を表示する列
    static constexpr int VALUE_COLUMN = 1;
};
