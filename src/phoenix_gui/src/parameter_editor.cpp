#include "parameter_editor.hpp"
#include "ui_parameter_editor.h"
#include <QtWidgets/QMessageBox>

using namespace std::chrono_literals;

constexpr auto SERVICE_TIMEOUT = 1s;

ParameterEditor::ParameterEditor(QWidget *parent) : QGroupBox(parent) {
    // UIを生成する
    _ui = new Ui_ParameterEditor;
    _ui->setupUi(this);

    // カラム幅を文字に合わせてリサイズする
    _ui->treeWidget->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    _ui->treeWidget->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    _ui->treeWidget->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);

    connect(this, &QGroupBox::clicked, _ui->treeWidget, &QTreeWidget::setEnabled);
    connect(_ui->treeWidget, &QTreeWidget::itemDoubleClicked, this, &ParameterEditor::startItemEditing);
    connect(_ui->treeWidget, &QTreeWidget::itemChanged, this, &ParameterEditor::changeParameterValue);
}

ParameterEditor::~ParameterEditor() {}

void ParameterEditor::addParameters(rclcpp::Node::SharedPtr node, const std::string &remote_node_name) {
    // リモートノードのパラメータサービスに接続する
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, remote_node_name);
    if (client->wait_for_service(1s)) {
        // リモートノード名の'/'以後の文字列を切り出して表示名とする
        std::string top_name;
        if (size_t pos = remote_node_name.find_last_of('/'); pos != std::string::npos) {
            top_name = remote_node_name.substr(pos + 1);
        }
        else {
            top_name = remote_node_name;
        }

        // パラメータを列挙する
        auto param_list = listNodeParameters(client);
        auto top_item = new QTreeWidgetItem(_ui->treeWidget, {top_name.c_str()});
        for (auto &param : param_list) {
            createTreeItem(top_item, param);
        }

        _parameter_clients[top_item] = client;
        top_item->setExpanded(true);
    }
}

void ParameterEditor::uninitializeNode(void) {
    _ui->treeWidget->clear();
    _parameter_clients.clear();
}

std::vector<rclcpp::Parameter> ParameterEditor::listNodeParameters(rclcpp::AsyncParametersClient::SharedPtr client) {
    std::vector<rclcpp::Parameter> result;
    if (client->service_is_ready()) {
        constexpr uint64_t DEPTH = 10;
        auto future_params = client->list_parameters({}, DEPTH);
        future_params.wait();
        auto params = future_params.get();
        auto future_results = client->get_parameters(params.names);
        future_results.wait();
        result = future_results.get();
    }
    return result;
}

QTreeWidgetItem *ParameterEditor::createTreeItem(QTreeWidgetItem *parent, const rclcpp::Parameter &param) {
    QTreeWidgetItem *item = nullptr;
    switch (param.get_type()) {
    case rclcpp::PARAMETER_BOOL:
        [[fallthrough]];
    case rclcpp::PARAMETER_INTEGER:
        [[fallthrough]];
    case rclcpp::PARAMETER_DOUBLE:
        [[fallthrough]];
    case rclcpp::PARAMETER_STRING:
        item = new QTreeWidgetItem(parent, {param.get_name().c_str()});
        modifyTreeItem(item, param);
        break;
    }
    return item;
}

bool ParameterEditor::modifyTreeItem(QTreeWidgetItem *item, const rclcpp::Parameter &param) {
    bool result = false;
    if (param.get_type() == rclcpp::PARAMETER_BOOL) {
        item->setData(VALUE_COLUMN, Qt::CheckStateRole, param.as_bool() ? Qt::Checked : Qt::Unchecked);
        item->setText(VALUE_COLUMN, QString());
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        result = true;
    }
    else {
        item->setData(VALUE_COLUMN, Qt::CheckStateRole, QVariant());
        switch (param.get_type()) {
        case rclcpp::PARAMETER_INTEGER:
            [[fallthrough]];
        case rclcpp::PARAMETER_DOUBLE:
            [[fallthrough]];
        case rclcpp::PARAMETER_STRING:
            item->setFlags(item->flags() | Qt::ItemIsEditable);
            result = true;
            break;

        default:
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            break;
        }
        item->setText(VALUE_COLUMN, param.value_to_string().c_str());
    }
    item->setText(TYPE_COLUMN, param.get_type_name().c_str());
    item->setData(VALUE_COLUMN, Qt::UserRole, (int)param.get_type());
    return result;
}

void ParameterEditor::startItemEditing(QTreeWidgetItem *item, int column) {
    if ((column == VALUE_COLUMN) && (item->flags() & Qt::ItemIsEditable)) {
        _ui->treeWidget->editItem(item, column);
    }
}

void ParameterEditor::changeParameterValue(QTreeWidgetItem *item, int column) {
    if (column != VALUE_COLUMN) {
        return;
    }
    auto top = _parameter_clients.find(item->parent());
    if (top != _parameter_clients.end()) {
        bool ok;
        std::string name = item->text(0).toStdString();
        int type = item->data(VALUE_COLUMN, Qt::UserRole).toInt(&ok);
        if (ok) {
            rclcpp::Parameter new_param;
            switch (type) {
            case rclcpp::PARAMETER_BOOL:
                // qDebug("%s = %s : bool", item->text(0).toStdString().c_str(), item->data(VALUE_COLUMN, Qt::CheckStateRole).toBool() ? "true" : "false");
                new_param = rclcpp::Parameter(name, item->data(VALUE_COLUMN, Qt::CheckStateRole).toBool());
                break;

            case rclcpp::PARAMETER_INTEGER:
                // qDebug("%s = %s : int", item->text(0).toStdString().c_str(), item->text(VALUE_COLUMN).toStdString().c_str());
                if (int64_t value = item->text(VALUE_COLUMN).toLongLong(&ok); ok) {
                    new_param = rclcpp::Parameter(name, value);
                }
                break;

            case rclcpp::PARAMETER_DOUBLE:
                // qDebug("%s = %s : double", item->text(0).toStdString().c_str(), item->text(VALUE_COLUMN).toStdString().c_str());
                if (double value = item->text(VALUE_COLUMN).toDouble(&ok); ok) {
                    new_param = rclcpp::Parameter(name, value);
                }
                break;

            case rclcpp::PARAMETER_STRING:
                // qDebug("%s = '%s' : string", item->text(0).toStdString().c_str(), item->text(VALUE_COLUMN).toStdString().c_str());
                new_param = rclcpp::Parameter(name, item->text(VALUE_COLUMN).toStdString());
                break;
            }
            if (new_param.get_type() != rclcpp::PARAMETER_NOT_SET) {
                // qDebug("Valid value");
                auto future_results = top->second->set_parameters({new_param});
                while (future_results.wait_for(SERVICE_TIMEOUT) != std::future_status::ready) {
                    if (QMessageBox::warning(this, this->title(), "The node does not respond.", QMessageBox::Abort, QMessageBox::Retry) == QMessageBox::Abort) {
                        return;
                    }
                }
                auto results = future_results.get();
                ok = (results.size() != 0) && results[0].successful;
            }
            if (!ok) {
                // エラーが起きたので値を元に戻す
                auto future_params = top->second->get_parameters({name});
                if (future_params.wait_for(SERVICE_TIMEOUT) == std::future_status::ready) {
                    auto params = future_params.get();
                    if (params.size() != 0) {
                        _ui->treeWidget->blockSignals(true);
                        modifyTreeItem(item, params[0]);
                        _ui->treeWidget->blockSignals(false);
                    }
                }
            }
        }
    }
}
