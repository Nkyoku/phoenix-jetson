#pragma once

#include <QtWidgets/QGroupBox>
#include <rclcpp/rclcpp.hpp>
#include <phoenix_msgs/srv/program_fpga.hpp>
#include <phoenix_msgs/srv/program_nios.hpp>

class Ui_SystemGroup;

class SystemGroup : public QGroupBox {
    Q_OBJECT

public:
    SystemGroup(QWidget *parent = nullptr);

    ~SystemGroup();

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
    Q_SLOT void programNios(void);

    Q_SLOT void programFpga(void);

    /// Qt Designerで作成したUI
    Ui_SystemGroup *_ui = nullptr;

    /// program_niosサービスクライアント
    rclcpp::Client<phoenix_msgs::srv::ProgramNios>::SharedPtr _program_nios_service;

    /// program_fpgaサービスクライアント
    rclcpp::Client<phoenix_msgs::srv::ProgramFpga>::SharedPtr _program_fpga_service;
};
