#pragma once

#include <QtWidgets/QGroupBox>
#include <QtWidgets/QTreeWidgetItem>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/srv/self_test.hpp>

class Ui_DiagnosticsViewer;

class DiagnosticsViewer : public QGroupBox {
    Q_OBJECT

public:
    DiagnosticsViewer(QWidget *parent = nullptr);

    ~DiagnosticsViewer();

    /**
     * @brief 診断メッセージの受信を開始する
     * @param node ローカルノード
     * @param remote_node_namespace リモートノードの名前空間
     */
    void startDiagnostics(rclcpp::Node::SharedPtr node, const std::string &remote_node_namespace);

    /**
     * @brief 診断メッセージの受信を停止する
     */
    void stopDiagnostics(void);

    /**
     * @brief 診断メッセージの表示をクリアする
     */
    Q_SLOT void clearDiagnostics(void);

private:
    /**
     * @brief セルフテストを実行する
     */
    Q_SLOT void runSelfTest(void);

    /**
     * @brief 最後に受信した診断メッセージでツリーを更新する
     */
    Q_SLOT void updateDiagnostics(void);

    /**
     * @brief telemetryTreeの診断ステータスを更新する
     * @param header メッセージのヘッダー
     * @param status 診断ステータス
     */
    void updateDiagnosticsInformation(const std_msgs::msg::Header &header, const diagnostic_msgs::msg::DiagnosticStatus &status);

    /// Qt Designerで作成したUI
    Ui_DiagnosticsViewer *_ui = nullptr;

    /// ツリーの項目
    struct {
        QTreeWidgetItem *timestamp, *level, *message, *error, *fault;
    } _treeItems;

    /// Diagnosticsメッセージのサブスクライバ
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _diagnosticsSubscription;

    /// self_testサービスクライアント
    rclcpp::Client<diagnostic_msgs::srv::SelfTest>::SharedPtr _selfTestService;

    /// Diagnosticsメッセージの受信対象の名前空間(ハードウェアID)
    std::string _namespace;

    /// 最後に受信したDiagnosticsメッセージ
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr _diagnosticsMessage;

    /// 値の列
    static constexpr int VALUE_COLUMN = 1;
};
