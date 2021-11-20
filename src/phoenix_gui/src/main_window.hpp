#pragma once

#define NOGDI
#include "image_viewer.hpp"
#include "node_thread.hpp"
#include <QtWidgets/QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

    Q_SLOT void reloadNamespaceList(void);

    Q_SLOT void connectToNodes(const QString &namespace_name);

    Q_SLOT void quitNodeThread(void);

    Q_SIGNAL void updateRequest(void);

    Q_SLOT void sendCommand(void);

    /**
     * @brief ROS2ノードを作成する
     * @return 作成したノード
     */
    static std::shared_ptr<rclcpp::Node> createNode(void);

    /// Qt Designerで作成したUI要素
    Ui_MainWindow *_Ui;

    ImageViewerWidget *_image_viewer;

    /// ROS2のネットワークを監視するためのノード
    std::shared_ptr<rclcpp::Node> _NetworkAwarenessNode;

    /// ノードの処理を行うスレッド
    NodeThread *_NodeThread = nullptr;

    /// 接続中のノードの属する名前空間
    std::string _namespace;

    // 作成したSubscriptionを保持する
    struct Subscriptions_t {
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image;
    } _Subscribers;

    /// 作成したPublisherを保持する
    struct Publisher_t {
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
    } _publishers;

    /// GUIのノード名の頭に付ける文字列
    static constexpr char GUI_NODE_NAME_PREFIX[] = "phoenix_gui_";

    /// 設定ファイルの組織名
    static constexpr char SETTINGS_ORGANIZATION[] = "RoboCup";

    /// 設定ファイルのアプリケーション名
    static constexpr char SETTINGS_APPLICATION[] = "PhoenixGUI";
};
