#include "main_window.hpp"
#include "ui_main_window.h"
#include "../../phoenix/include/phoenix.hpp"
#include <QtCore/QDebug>
#include <QtCore/QSettings>
#include <QtCore/QSysInfo>
#include <algorithm>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

constexpr auto SERVICE_TIMEOUT = 1s;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    // UIを生成する
    _Ui = new Ui_MainWindow;
    _Ui->setupUi(this);
    _Ui->scanButton->setIcon(QApplication::style()->standardPixmap(QStyle::SP_BrowserReload));
    _Ui->namespaceComboBox->setSizeAdjustPolicy(QComboBox::AdjustToContents);

    // カメラ画像を表示するImageViewerを作成する
    auto image_layout = new QHBoxLayout;
    _Ui->cameraGroup->setLayout(image_layout);
    _image_viewer = new ImageViewerWidget(_Ui->cameraGroup);
    image_layout->insertWidget(0, _image_viewer);
    _image_viewer->convertBgrToRgb(true);
    _image_viewer->setMirror(true, true);

    // ウィンドウ位置とスプリッタの位置を戻す
    restoreSettings();

    // ROS2のネットワークを監視するためのノードを生成する
    _NetworkAwarenessNode = createNode();

    // Qtのシグナルを接続する
    connect(_Ui->scanButton, &QPushButton::clicked, this, &MainWindow::reloadNamespaceList);
    connect(_Ui->namespaceComboBox, &QComboBox::currentTextChanged, this, &MainWindow::connectToNodes);
    connect(this, &MainWindow::updateRequest, _image_viewer, qOverload<>(&ImageViewerWidget::update), Qt::QueuedConnection);
    connect(_Ui->controllerGroup, &ControllerControls::commandReady, this, &MainWindow::sendCommand);

    // リストを更新する
    reloadNamespaceList();
}

MainWindow::~MainWindow() {
    quitNodeThread();
    saveSettings();
}

void MainWindow::restoreSettings(void) {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, SETTINGS_ORGANIZATION, SETTINGS_APPLICATION);
    restoreGeometry(settings.value("WindowGeometry").toByteArray());
    restoreState(settings.value("WindowState").toByteArray());
    if (settings.value("WindowMaximized", false).toBool() == true) {
        setWindowState(Qt::WindowMaximized);
    }
    _Ui->splitter->restoreState(settings.value("Splitter1State").toByteArray());
    _Ui->splitter_2->restoreState(settings.value("Splitter2State").toByteArray());
}

void MainWindow::saveSettings(void) const {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, SETTINGS_ORGANIZATION, SETTINGS_APPLICATION);
    settings.setValue("WindowGeometry", saveGeometry());
    settings.setValue("WindowState", saveState());
    settings.setValue("WindowMaximized", isMaximized());
    settings.setValue("Splitter1State", _Ui->splitter->saveState());
    settings.setValue("Splitter2State", _Ui->splitter_2->saveState());
}

void MainWindow::reloadNamespaceList(void) {
    // TIMEOUTで指定した時間、spin_once()を実行しながら待つ
    // ここでGraphが更新されるのを待つ
    static constexpr auto TIMEOUT = std::chrono::milliseconds(100);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(_NetworkAwarenessNode);
    auto start_time = std::chrono::system_clock::now();
    do {
        exec.spin_once(std::chrono::milliseconds(1));
    } while ((std::chrono::system_clock::now() - start_time) < TIMEOUT);

    // ノード名と名前空間名のリストを取得しphoenix::NAMESPACE_NAMEのサブ名前空間を持つ名前空間のリストを抽出する
    const QString phoenix_sub_namespace = QString('/') + phoenix::NAMESPACE_NAME;
    QStringList namespace_list;
    namespace_list.append("");
    auto graph = _NetworkAwarenessNode->get_node_graph_interface();
    auto node_and_namespace_list = graph->get_node_names_and_namespaces();
    for (auto &item : node_and_namespace_list) {
        if (namespace_list.contains(item.second.c_str())) {
            continue;
        }
        QString namespace_name(item.second.c_str());
        if (namespace_name.endsWith(phoenix_sub_namespace)) {
            namespace_list.append(item.second.c_str());
        }
    }
    std::sort(namespace_list.begin(), namespace_list.end());

    // コンボボックスの内容を更新する
    // 現在選択中の項目が新しいリストに存在しないときのみcurrentTextChanged()が発生するようにする
    QString current_text = _Ui->namespaceComboBox->currentText();
    if (namespace_list.contains(current_text)) {
        _Ui->namespaceComboBox->blockSignals(true);
        _Ui->namespaceComboBox->clear();
        _Ui->namespaceComboBox->addItems(namespace_list);
        _Ui->namespaceComboBox->setCurrentText(current_text);
        _Ui->namespaceComboBox->blockSignals(false);
    }
    else {
        _Ui->namespaceComboBox->clear();
        _Ui->namespaceComboBox->blockSignals(true);
        _Ui->namespaceComboBox->addItems(namespace_list);
        _Ui->namespaceComboBox->blockSignals(false);
    }
}

void MainWindow::connectToNodes(const QString &namespace_name) {
    using namespace std::placeholders;

    // 既存のノードとスレッドを終了する
    quitNodeThread();

    if (namespace_name.isEmpty()) {
        _namespace.clear();

        // 画面を初期化する
        _image_viewer->setImage(nullptr);
        emit updateRequest();
        return;
    }
    _namespace = namespace_name.toStdString();

    // '/'を付与する
    std::string prefix = _namespace;
    if (prefix.back() != '/') {
        prefix += '/';
    }

    // ノードとスレッドを作成する
    _NodeThread = new NodeThread(this, createNode());
    connect(_NodeThread, &NodeThread::finished, _NodeThread, &QObject::deleteLater);

    // 各UI部品のノードの機能を初期化する
    _Ui->diagnosticsViewer->initializeNode(_NodeThread->node(), _namespace);
    _Ui->telemetryViewer->initializeNode(_NodeThread->node(), _namespace);
    _Ui->systemGroup->initializeNode(_NodeThread->node(), _namespace);

    // センサーデータを購読するときのQoSの設定
    const rclcpp::SensorDataQoS qos_sensor;

    // imageトピックを受信するSubscriptionを作成する
    _Subscribers.image = _NodeThread->node()->create_subscription<sensor_msgs::msg::Image>("/video_source/raw", qos_sensor,
                                                                                           [this](const std::shared_ptr<sensor_msgs::msg::Image> msg) {
                                                                                               _image_viewer->setImage(msg);
                                                                                               emit updateRequest();
                                                                                           });

    // cmd_velトピックを配信するPublisherを作成する
    _publishers.velocity = _NodeThread->node()->create_publisher<geometry_msgs::msg::Twist>(prefix + phoenix::TOPIC_NAME_COMMAND_VELOCITY, 1);

    // スレッドを実行
    _NodeThread->start();

    // パラメータの一覧をツリーに表示する
    _Ui->parameterGroup->addParameters(_NodeThread->node(), prefix + phoenix::command::NODE_NAME);
    //_Ui->parameterGroup->addParameters(_NodeThread->node(), prefix + phoenix::stream::NODE_NAME);
}

void MainWindow::quitNodeThread(void) {
    if (_NodeThread != nullptr) {
        // Subscriptionを破棄する
        _Subscribers.image.reset();

        // Publisherを破棄する
        _publishers.velocity.reset();

        // 各UI部品が受け持つノードの機能を終了する
        _Ui->diagnosticsViewer->uninitializeNode();
        _Ui->telemetryViewer->uninitializeNode();
        _Ui->systemGroup->uninitializeNode();
        _Ui->parameterGroup->uninitializeNode();

        // スレッドを終了する
        // deleteはdeleteLater()スロットにより行われるのでここでする必要はない
        _NodeThread->requestInterruption();
        _NodeThread->quit();
        _NodeThread->wait(NodeThread::QUIT_TIMEOUT);
        _NodeThread = nullptr;
    }
}

void MainWindow::sendCommand(void) {
    if (_publishers.velocity) {
        geometry_msgs::msg::Twist msg = _Ui->controllerGroup->targetVelocity();
        _publishers.velocity->publish(msg);
    }
}

std::shared_ptr<rclcpp::Node> MainWindow::createNode(void) {
    // ノードを作成する
    // ノード名の被りを防止するため末尾に乱数を付与する
    QString node_name = QString("%1%2").arg(GUI_NODE_NAME_PREFIX).arg(std::random_device()(), 8, 16, QLatin1Char('0'));
    QString namespace_name = QSysInfo::machineHostName();
    return std::make_shared<rclcpp::Node>(node_name.toStdString(), namespace_name.toStdString());
}
