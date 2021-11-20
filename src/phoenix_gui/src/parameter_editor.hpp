#pragma once

#include <QtWidgets/QGroupBox>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <QtWidgets/QTreeWidgetItem>
#include <unordered_map>
#include <vector>

class Ui_ParameterEditor;

class ParameterEditor : public QGroupBox {
    Q_OBJECT

public:
    ParameterEditor(QWidget *parent = nullptr);

    ~ParameterEditor();

    /**
     * @brief 指定された名前のリモートノードのパラメータをツリーに追加する
     * @param node ローカルノード
     * @param remote_node_name リモートノード名
     */
    void addParameters(rclcpp::Node::SharedPtr node, const std::string &remote_node_name);

    /**
     * @brief ノードの終了処理を行う
     */
    void uninitializeNode(void);

private:
    /**
     * @brief リモートノードのパラメータの一覧を取得する
     * @param client パラメータクライアント
     * @return パラメータの一覧
     */
    std::vector<rclcpp::Parameter> listNodeParameters(rclcpp::AsyncParametersClient::SharedPtr client);

    /**
     * @brief パラメータツリーの項目を新しく作成する
     * @param parent 親となるQTreeWidgetItem
     * @param param 表示するパラメータ
     * @return 作成したQTreeWidgetItem
     */
    static QTreeWidgetItem *createTreeItem(QTreeWidgetItem *parent, const rclcpp::Parameter &param);

    /**
     * @brief パラメータツリーの項目を更新する
     * @param item 更新対象となるQTreeWidgetItem
     * @param param 表示するパラメータ
     * @return 更新に成功したらtrueを返す
     */
    static bool modifyTreeItem(QTreeWidgetItem *item, const rclcpp::Parameter &param);

    /**
     * @brief ユーザーによるパラメータの編集を開始する
     * @param item 操作対象のQTreeWidgetItem
     * @param column 操作対象の列
     */
    Q_SLOT void startItemEditing(QTreeWidgetItem *item, int column);

    /**
     * @brief パラメータの変更をノードに反映する
     * @param item 操作対象のQTreeWidgetItem
     * @param column 操作対象の列
     */
    Q_SLOT void changeParameterValue(QTreeWidgetItem *item, int column);

    /// Qt Designerで作成したUI
    Ui_ParameterEditor *_ui = nullptr;

    /// ノード
    rclcpp::Node::SharedPtr _node;

    /// パラメータクライアント
    std::unordered_map<QTreeWidgetItem *, rclcpp::AsyncParametersClient::SharedPtr> _parameter_clients;

    /// 名前の列
    static constexpr int NAME_COLUMN = 0;

    /// 値の列
    static constexpr int VALUE_COLUMN = 1;

    /// 型名の列
    static constexpr int TYPE_COLUMN = 2;
};
