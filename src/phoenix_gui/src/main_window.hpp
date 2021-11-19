#pragma once

#define NOGDI
#include "image_viewer.hpp"
#include "node_thread.hpp"
#include <QtCore/QFile>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTreeWidgetItem>
#include <phoenix_msgs/msg/stream_data_adc2.hpp>
#include <phoenix_msgs/msg/stream_data_motion.hpp>
#include <phoenix_msgs/srv/program_fpga.hpp>
#include <phoenix_msgs/srv/program_nios.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/srv/self_test.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>

class Ui_MainWindow;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

private:
    /**
     *  設定ファイルから状態を復元する
     */
    void restoreSettings(void);

    /*
     * 設定ファイルに状態を保存する
     */
    void saveSettings(void) const;

    // void closeEvent(QCloseEvent *event) override;

    Q_SLOT void reloadNamespaceList(void);

    Q_SLOT void connectToNodes(const QString &namespace_name);

    Q_SLOT void updateTelemertyTreeItems(void);

    void generateTelemetryTreeItems(void);

    Q_SLOT void startLogging(void);

    Q_SLOT void stopLogging(void);

    Q_SLOT void quitNodeThread(void);

    Q_SIGNAL void updateRequest(void);

    Q_SLOT void sendCommand(void);

    Q_SLOT void programNios(void);

    Q_SLOT void programFpga(void);

    Q_SLOT void runSelfTest(void);



    /// telemetryTreeに表示する項目の定義
    struct TreeItems_t {
        struct DiagnosticItems_t {
            QTreeWidgetItem *timestamp, *level, *message, *error, *fault;
        } diag;
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

    /**
     * @brief telemetryTreeの診断ステータスを更新する
     * @param items ツリーの項目
     * @param header ヘッダー
     * @param status 診断ステータス
     */
    static void updateDiagnosticsInformation(TreeItems_t::DiagnosticItems_t &items, const std_msgs::msg::Header &header,
                                             const diagnostic_msgs::msg::DiagnosticStatus &status);

    /**
     * @brief ROS2ノードを作成する
     * @return 作成したノード
     */
    static std::shared_ptr<rclcpp::Node> createNode(void);

    /// Qt Designerで作成したUI要素
    Ui_MainWindow *_Ui;

    ImageViewerWidget *_image_viewer;

    /// telemetryTreeに表示する項目
    TreeItems_t _TreeItems;

    /// テレメトリのログを保存するファイル
    std::shared_ptr<QFile> _LogFile;

    /// テレメトリのログに含まれるフレーム番号
    uint32_t _LogFrameNumber = 0;

    /// ROS2のネットワークを監視するためのノード
    std::shared_ptr<rclcpp::Node> _NetworkAwarenessNode;

    /// ノードの処理を行うスレッド
    NodeThread *_NodeThread = nullptr;

    /// 接続中のノードの属する名前空間
    std::string _namespace;

    // 作成したSubscriptionを保持する
    struct Subscriptions_t {
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery;
        rclcpp::Subscription<phoenix_msgs::msg::StreamDataAdc2>::SharedPtr adc2;
        rclcpp::Subscription<phoenix_msgs::msg::StreamDataMotion>::SharedPtr motion;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image;
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
    } _Subscribers;

    /// 作成したPublisherを保持する
    struct Publisher_t {
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
    } _publishers;

    /// 受信した最後のトピックを保持する
    struct LastMessages_t {
        std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray> diagnostics;
        std::shared_ptr<sensor_msgs::msg::BatteryState> battery;
        std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> adc2;
        std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> motion;
    } _LastMessages;

    /// 作成したサービスクライアントを保持する
    struct Clients_t {
        rclcpp::Client<phoenix_msgs::srv::ProgramNios>::SharedPtr program_nios;
        rclcpp::Client<phoenix_msgs::srv::ProgramFpga>::SharedPtr program_fpga;
        rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedPtr self_test;
    } _Clients;

    /// telemetryTreeで値を表示する列
    static constexpr int COL = 1;

    /// GUIのノード名の頭に付ける文字列
    static constexpr char GUI_NODE_NAME_PREFIX[] = "phoenix_gui_";

    /// 設定ファイルの組織名
    static constexpr char SETTINGS_ORGANIZATION[] = "RoboCup";

    /// 設定ファイルのアプリケーション名
    static constexpr char SETTINGS_APPLICATION[] = "PhoenixGUI";
};
