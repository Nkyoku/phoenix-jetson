#include "diagnostics_viewer.hpp"
#include "ui_diagnostics_viewer.h"
#include <QtWidgets/QMessageBox>

using namespace std::chrono_literals;

constexpr auto SERVICE_TIMEOUT = 1s;

DiagnosticsViewer::DiagnosticsViewer(QWidget *parent) : QGroupBox(parent) {
    // UIを生成する
    _ui = new Ui_DiagnosticsViewer;
    _ui->setupUi(this);
    clearDiagnostics();

    connect(_ui->runSelfTestButton, &QPushButton::clicked, this, &DiagnosticsViewer::runSelfTest);
}

DiagnosticsViewer::~DiagnosticsViewer() {}

void DiagnosticsViewer::startDiagnostics(rclcpp::Node::SharedPtr node, const std::string &remote_node_namespace) {
    _namespace = remote_node_namespace;
    clearDiagnostics();

    // Diagnosticsメッセージを受信するSubscriptionを作成する
    _diagnosticsSubscription = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10, [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            for (const auto &diag : msg->status) {
                if (diag.hardware_id == _namespace) {
                    // 対象の診断メッセージを受信したらUIスレッドで更新を行う
                    std::atomic_store(&_diagnosticsMessage, msg);
                    QMetaObject::invokeMethod(this, &DiagnosticsViewer::updateDiagnostics, Qt::QueuedConnection);
                    break;
                }
            }
        });

    // self_testサービスと接続する
    std::string prefix = _namespace;
    if (prefix.back() != '/') {
        prefix += '/';
    }
    _selfTestService = node->create_client<diagnostic_msgs::srv::SelfTest>(prefix + "self_test");
    if (_selfTestService->wait_for_service(SERVICE_TIMEOUT)) {
        _ui->runSelfTestButton->setEnabled(true);
    }
    else {
        _selfTestService.reset();
        _ui->runSelfTestButton->setEnabled(false);
    }
}

void DiagnosticsViewer::stopDiagnostics(void) {
    _diagnosticsSubscription.reset();
    _selfTestService.reset();
}

void DiagnosticsViewer::clearDiagnostics(void) {
    QString empty;
    _ui->timestampLabel->setText(empty);
    _ui->levelLabel->setText(empty);
    _ui->messageLabel->setText(empty);
    _ui->errorLabel->setText(empty);
    _ui->faultLabel->setText(empty);
}

void DiagnosticsViewer::runSelfTest(void) {
    auto request = std::make_shared<diagnostic_msgs::srv::SelfTest::Request>();
    auto result = _selfTestService->async_send_request(request);
    while (result.wait_for(SERVICE_TIMEOUT) != std::future_status::ready) {
        if (QMessageBox::warning(this, title(), "The service does not respond.", QMessageBox::Abort, QMessageBox::Retry) == QMessageBox::Abort) {
            return;
        }
    }
    auto response = result.get();
    if (!response) {
        QMessageBox::critical(this, title(), "Service didn't respond");
        return;
    }

    // 結果を表示する
    std::stringstream ss;
    ss << (response->passed ? "Passed:" : "Failed:") << std::endl << std::endl;
    auto it = std::find_if(response->status.begin(), response->status.end(), [&](const diagnostic_msgs::msg::DiagnosticStatus &diag) {
        return diag.hardware_id == _namespace;
    });
    if (it != response->status.end()) {
        ss << "name: " << it->name << std::endl;
        ss << "message: " << it->message << std::endl;
        ss << "values:";
        if (!it->values.empty()) {
            for (auto &item : it->values) {
                ss << std::endl << "  " << item.key << ": '" << item.value << '\'';
            }
        }
    }
    else {
        ss << "Unknown detail";
    }
    if (response->passed) {
        QMessageBox::information(this, title(), QString::fromStdString(ss.str()));
    }
    else {
        QMessageBox::critical(this, title(), QString::fromStdString(ss.str()));
    }
}

void DiagnosticsViewer::updateDiagnostics(void) {
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg = std::atomic_load(&_diagnosticsMessage);
    if (!msg) {
        return;
    }

    for (const auto &status : msg->status) {
        if (status.hardware_id != _namespace) {
            continue;
        }

        // timestampをミリ秒単位で表示する
        _ui->timestampLabel->setText(QString("%1.%2").arg(msg->header.stamp.sec).arg(msg->header.stamp.nanosec / 1000000, 3, 10, QLatin1Char('0')));

        // messageを表示する
        _ui->messageLabel->setText(QString::fromStdString(status.message));

        // levelを表示する
        {
            static const std::unordered_map<decltype(status.level), QString> level_map = {
                {diagnostic_msgs::msg::DiagnosticStatus::OK, "OK"},
                {diagnostic_msgs::msg::DiagnosticStatus::WARN, "WARN"},
                {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "ERROR"},
                {diagnostic_msgs::msg::DiagnosticStatus::STALE, "STALE"},
            };
            auto it = level_map.find(status.level);
            if (it != level_map.end()) {
                _ui->levelLabel->setText(it->second);
            }
            else {
                _ui->levelLabel->setText(QString::number(status.level));
            }
            auto palette = _ui->levelLabel->palette();
            palette.setColor(QPalette::WindowText, (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) ? QColor(0, 0, 0) : QColor(255, 0, 0));
            _ui->levelLabel->setPalette(palette);
        }

        // エラーフラグとフォルトフラグを表示する
        static const QString none("none");
        bool error_occured = false, fault_occured = false;
        for (auto &item : status.values) {
            if (!error_occured && (item.key == "Error Causes")) {
                _ui->errorLabel->setText(QString::fromStdString(item.value));
                error_occured = true;
            }
            else if (!fault_occured && (item.key == "Fault Causes")) {
                _ui->faultLabel->setText(QString::fromStdString(item.value));
                fault_occured = true;
            }
        }
        if (!error_occured) {
            _ui->errorLabel->setText(none);
        }
        if (!fault_occured) {
            _ui->faultLabel->setText(none);
        }

        break;
    }
}
