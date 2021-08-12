﻿#include "main_window.hpp"
#include "cintelhex/cintelhex.h"
#include "format_value.hpp"
#include "ui_main_window.h"
#include "../../phoenix/include/phoenix.hpp"
#include <QtCore/QDebug>
#include <QtCore/QSettings>
#include <QtCore/QSysInfo>
#include <QtCore/QTimer>
#include <QtGui/QCloseEvent>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGestureEvent>
#include <QtWidgets/QMessageBox>
#include <algorithm>
#include <chrono>
#include <random>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    // UIを生成する
    _Ui = new Ui_MainWindow;
    _Ui->setupUi(this);
    _Ui->reloadButton->setIcon(QApplication::style()->standardPixmap(QStyle::SP_BrowserReload));
    _Ui->namespaceComboBox->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    _Ui->gamepadComboBox->addItem("None");
    generateTelemetryTreeItems();
    _ImageViewer = new ImageViewerWidget(_Ui->splitter_2);
    _Ui->splitter_2->insertWidget(0, _ImageViewer);
    _ImageViewer->convertBgrToRgb(true);
    _ImageViewer->setMirror(true, true);

    _Pad.scene = new QGraphicsScene(this);
    _Ui->controllerPadGraphics->setScene(_Pad.scene);
    _Ui->controllerPadGraphics->viewport()->grabGesture(Qt::PanGesture);
    _Ui->controllerPadGraphics->viewport()->grabGesture(Qt::PinchGesture);
    _Ui->controllerPadGraphics->viewport()->installEventFilter(this);
    _Pad.cross_h = _Pad.scene->addLine(0, 0, 1, 0, QPen(Qt::black));
    _Pad.cross_v = _Pad.scene->addLine(0, 0, 0, 1, QPen(Qt::black));

    // ウィンドウ位置とスプリッタの位置を戻す
    restoreSettings();

    // ゲームパッド入力スレッドを作成する
    _GamepadThread = new GamepadThread();
    connect(_GamepadThread, &GamepadThread::finished, _GamepadThread, &QObject::deleteLater);
    connect(
        _GamepadThread, &GamepadThread::gamepadConnected, this,
        [this](int deviceId) {
            _Ui->gamepadComboBox->addItem(QString("XInput %1").arg(deviceId), deviceId);
        },
        Qt::QueuedConnection);
    connect(
        _GamepadThread, &GamepadThread::gamepadDisconnected, this,
        [this](int deviceId) {
            int index = _Ui->gamepadComboBox->findData(deviceId);
            if (index == _Ui->gamepadComboBox->currentIndex()) {
                _Ui->gamepadComboBox->setCurrentIndex(0);
            }
            if (0 <= index) {
                _Ui->gamepadComboBox->removeItem(index);
            }
        },
        Qt::QueuedConnection);

    // ROS2のネットワークを監視するためのノードを生成する
    _NetworkAwarenessNode = createNode();

    // テレメトリ更新用のタイマーを生成する
    QTimer *timer = new QTimer(this);
    timer->start(100);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateTelemertyTreeItems);

    // コマンド送信用のタイマーを生成する
    QTimer *timer2 = new QTimer(this);
    timer2->start(100);
    connect(timer2, &QTimer::timeout, this, &MainWindow::sendCommand);

    // Qtのシグナルを接続する
    connect(_Ui->reloadButton, &QPushButton::clicked, this, &MainWindow::reloadNamespaceList);
    connect(_Ui->namespaceComboBox, &QComboBox::currentTextChanged, this, &MainWindow::connectToNodes);
    connect(this, &MainWindow::updateRequest, _ImageViewer, qOverload<>(&ImageViewerWidget::update), Qt::QueuedConnection);
    connect(_Ui->gamepadComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::connectToGamepad);
    connect(_Ui->saveLogButton, &QPushButton::clicked, this, &MainWindow::startLogging);
    connect(_Ui->stopLogButton, &QPushButton::clicked, this, &MainWindow::stopLogging);
    connect(_Ui->programNiosButton, &QPushButton::clicked, this, &MainWindow::programNios);
    connect(_Ui->programFpgaButton, &QPushButton::clicked, this, &MainWindow::programFpga);
    connect(_Ui->selfTestButton, &QPushButton::clicked, this, &MainWindow::runSelfTest);

    // リストを更新する
    reloadNamespaceList();

    // ゲームパッド入力スレッドを開始する
    _GamepadThread->start();
}

MainWindow::~MainWindow() {
    quitNodeThread();
    quitGamepadThread();
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
    _Ui->tabWidget->setCurrentIndex(settings.value("TabIndex", _Ui->tabWidget->currentIndex()).toInt());
}

void MainWindow::saveSettings(void) const {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, SETTINGS_ORGANIZATION, SETTINGS_APPLICATION);
    settings.setValue("WindowGeometry", saveGeometry());
    settings.setValue("WindowState", saveState());
    settings.setValue("WindowMaximized", isMaximized());
    settings.setValue("Splitter1State", _Ui->splitter->saveState());
    settings.setValue("Splitter2State", _Ui->splitter_2->saveState());
    settings.setValue("TabIndex", _Ui->tabWidget->currentIndex());
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
    const QWidget *viewport = _Ui->controllerPadGraphics->viewport();
    if (obj == viewport) {
        if (event->type() == QEvent::Gesture) {
            QGestureEvent *gesture_event = static_cast<QGestureEvent *>(event);
            if (QGesture *gesture = gesture_event->gesture(Qt::PanGesture)) {
                QPanGesture *pan = static_cast<QPanGesture *>(gesture);
                QSize size = viewport->size();
                _Pad.velocity_scale_x = pan->offset().x() / (size.width() * 0.5);
                _Pad.velocity_scale_y = -pan->offset().y() / (size.height() * 0.5);
                if ((pan->state() == Qt::GestureFinished) || (pan->state() == Qt::GestureCanceled)) {
                    _Pad.velocity_scale_x = 0.0;
                    _Pad.velocity_scale_y = 0.0;
                }
            }
            if (QGesture *gesture = gesture_event->gesture(Qt::PinchGesture)) {
                QPinchGesture *pinch = static_cast<QPinchGesture *>(gesture);
                QPinchGesture::ChangeFlags change_flags = pinch->changeFlags();
                if (change_flags & QPinchGesture::RotationAngleChanged) {
                    double d = pinch->rotationAngle() - pinch->lastRotationAngle();
                    if (d <= -180.0) {
                        d += 360.0;
                    }
                    else if (180.0 <= d) {
                        d -= 360.0;
                    }
                    _Pad.velocity_scale_omega -= d;
                }
                if ((pinch->state() == Qt::GestureFinished) || (pinch->state() == Qt::GestureCanceled)) {
                    _Pad.velocity_scale_omega = 0.0;
                }
            }
            return true;
        }
        else if (event->type() == QEvent::Resize) {
            int width = viewport->width();
            int height = viewport->height();
            _Pad.scene->setSceneRect(0, 0, width, height);
            _Pad.cross_h->setLine(0, height / 2, width, height / 2);
            _Pad.cross_v->setLine(width / 2, 0, width / 2, height);
        }
    }
    return false;
}

void MainWindow::connectToGamepad(int index) {
    qDebug() << "Selection changed " << index;
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
        _ImageViewer->setImage(nullptr);
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

    // /diagnosticsトピックを受信するSubscriptionを作成する
    _Subscribers.diagnostics = _NodeThread->node()->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10, [this](const std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray> msg) {
            for (const auto &diag : msg->status) {
                if (diag.hardware_id == _namespace) {
                    std::atomic_store(&_LastMessages.diagnostics, msg);
                    break;
                }
            }
        });

    // センサーデータを購読するときのQoSの設定
    const rclcpp::SensorDataQoS qos_sensor;

    // batteryトピックを受信するSubscriptionを作成する
    _Subscribers.battery = _NodeThread->node()->create_subscription<sensor_msgs::msg::BatteryState>(
        prefix + phoenix::TOPIC_NAME_BATTERY, qos_sensor, [this](const std::shared_ptr<sensor_msgs::msg::BatteryState> msg) {
            std::atomic_store(&_LastMessages.battery, msg);
        });

    // stream_data_adc2トピックを受信するSubscriptionを作成する
    _Subscribers.adc2 = _NodeThread->node()->create_subscription<phoenix_msgs::msg::StreamDataAdc2>(
        prefix + phoenix::TOPIC_NAME_ADC2, qos_sensor, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> msg) {
            if (!_LastMessages.adc2) {
                std::atomic_store(&_LastMessages.adc2, msg);
            }
            else {
                /*std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> last_msg = _LastMessages.adc2;
                msg->dc48v_voltage = std::max(msg->dc48v_voltage, last_msg->dc48v_voltage);
                msg->dribble_current = std::max(msg->dribble_current, last_msg->dribble_current);*/
                std::atomic_store(&_LastMessages.adc2, msg);
            }
        });

    // stream_data_motionトピックを受信するSubscriptionを作成する
    _Subscribers.motion = _NodeThread->node()->create_subscription<phoenix_msgs::msg::StreamDataMotion>(
        prefix + phoenix::TOPIC_NAME_MOTION, qos_sensor, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> msg) {
            std::atomic_store(&_LastMessages.motion, msg);
            std::shared_ptr<QFile> file = _LogFile;
            if (file) {
                QTextStream stream(file.get());
                QChar sep(',');
                std::shared_ptr<sensor_msgs::msg::BatteryState> battery_msg = _LastMessages.battery;
                std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> adc2_msg = _LastMessages.adc2;
                float battery_voltage = battery_msg ? battery_msg->voltage : 0.0f;
                float battery_current = battery_msg ? -battery_msg->current : 0.0f;
                float dc48v_voltage = adc2_msg ? adc2_msg->dc48v_voltage : 0.0f;
                stream << (0.001 * _LogFrameNumber) << sep;
                stream << msg->wheel_velocity_meas[0] << sep;
                stream << msg->wheel_velocity_meas[1] << sep;
                stream << msg->wheel_velocity_meas[2] << sep;
                stream << msg->wheel_velocity_meas[3] << sep;
                stream << msg->wheel_current_meas_q[0] << sep;
                stream << msg->wheel_current_meas_q[1] << sep;
                stream << msg->wheel_current_meas_q[2] << sep;
                stream << msg->wheel_current_meas_q[3] << sep;
                stream << msg->wheel_velocity_ref[0] << sep;
                stream << msg->wheel_velocity_ref[1] << sep;
                stream << msg->wheel_velocity_ref[2] << sep;
                stream << msg->wheel_velocity_ref[3] << sep;
                stream << msg->wheel_current_ref[0] << sep;
                stream << msg->wheel_current_ref[1] << sep;
                stream << msg->wheel_current_ref[2] << sep;
                stream << msg->wheel_current_ref[3] << sep;
                stream << msg->wheel_current_limit[0] << sep;
                stream << msg->wheel_current_limit[1] << sep;
                stream << msg->wheel_current_limit[2] << sep;
                stream << msg->wheel_current_limit[3] << sep;
                stream << msg->machine_velocity[0] << sep;
                stream << msg->machine_velocity[1] << sep;
                stream << msg->machine_velocity[2] << sep;
                stream << msg->slip_flags << sep;
                stream << msg->accelerometer[0] << sep;
                stream << msg->accelerometer[1] << sep;
                stream << msg->accelerometer[2] << sep;
                stream << msg->gyroscope[0] << sep;
                stream << msg->gyroscope[1] << sep;
                stream << msg->gyroscope[2] << sep;
                stream << dc48v_voltage << sep;
                stream << battery_voltage << sep;
                stream << battery_current << Qt::endl;
                _LogFrameNumber++;
            }
        });

    // imageトピックを受信するSubscriptionを作成する
    _Subscribers.image = _NodeThread->node()->create_subscription<sensor_msgs::msg::Image>("/video_source/raw", qos_sensor,
                                                                                           [this](const std::shared_ptr<sensor_msgs::msg::Image> msg) {
                                                                                               _ImageViewer->setImage(msg);
                                                                                               emit updateRequest();
                                                                                           });

    // cmd_velトピックを配信するPublisherを作成する
    _publishers.velocity = _NodeThread->node()->create_publisher<geometry_msgs::msg::Twist>(prefix + phoenix::TOPIC_NAME_COMMAND_VELOCITY, 1);

    // NiosII書き換えサービスを見つける
    _Clients.program_nios = _NodeThread->node()->create_client<phoenix_msgs::srv::ProgramNios>(prefix + phoenix::SERVICE_NAME_PROGRAM_NIOS);
    if (_Clients.program_nios->wait_for_service(std::chrono::milliseconds(1000)) == false) {
        _Clients.program_nios.reset();
        _Ui->programNiosButton->setEnabled(false);
    }
    else {
        _Ui->programNiosButton->setEnabled(true);
    }

    // FPGA書き換えサービスを見つける
    _Clients.program_fpga = _NodeThread->node()->create_client<phoenix_msgs::srv::ProgramFpga>(prefix + phoenix::SERVICE_NAME_PROGRAM_FPGA);
    if (_Clients.program_fpga->wait_for_service(std::chrono::milliseconds(1000)) == false) {
        _Clients.program_fpga.reset();
        _Ui->programFpgaButton->setEnabled(false);
    }
    else {
        _Ui->programFpgaButton->setEnabled(true);
    }

    // セルフテストサービスを見つける
    _Clients.self_test = _NodeThread->node()->create_client<diagnostic_msgs::srv::SelfTest>(prefix + "self_test");
    if (_Clients.self_test->wait_for_service(std::chrono::milliseconds(1000)) == false) {
        _Clients.self_test.reset();
        _Ui->selfTestButton->setEnabled(false);
    }
    else {
        _Ui->selfTestButton->setEnabled(true);
    }

    // スレッドを実行
    _NodeThread->start();
}

void MainWindow::updateTelemertyTreeItems(void) {
    if (_LastMessages.diagnostics) {
        auto msg = std::atomic_exchange(&_LastMessages.diagnostics, std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray>());
        for (const auto &diag : msg->status) {
            if (diag.hardware_id == _namespace) {
                updateDiagnosticsInformation(_TreeItems.diag, msg->header, diag);
                break;
            }
        }
    }
    if (_LastMessages.battery) {
        // auto msg = std::atomic_exchange(&_LastMessages.battery, std::shared_ptr<sensor_msgs::msg::BatteryState>());
        auto &msg = _LastMessages.battery;
        _TreeItems.battery.present->setText(COL, boolToString(msg->present));
        _TreeItems.battery.voltage->setText(COL, QString::number(msg->voltage, 'f', 3));
        _TreeItems.battery.current->setText(COL, QString::number(msg->current, 'f', 3));
        _TreeItems.battery.temperature->setText(COL, QString::number(msg->temperature, 'f', 3));
    }
    if (_LastMessages.adc2) {
        // auto msg = std::atomic_exchange(&_LastMessages.adc2, std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2>());
        auto &msg = _LastMessages.adc2;
        _TreeItems.adc2.dc48v_voltage->setText(COL, QString::number(msg->dc48v_voltage, 'f', 3));
        _TreeItems.adc2.dribble_voltage->setText(COL, QString::number(msg->dribble_voltage, 'f', 3));
        _TreeItems.adc2.dribble_current->setText(COL, QString::number(msg->dribble_current, 'f', 3));
    }
    if (_LastMessages.motion) {
        // auto msg = std::atomic_exchange(&_LastMessages.motion, std::shared_ptr<phoenix_msgs::msg::StreamDataMotion>());
        auto &msg = _LastMessages.motion;
        for (int index = 0; index < 3; index++) {
            _TreeItems.motion.accelerometer[index]->setText(COL, QString::number(msg->accelerometer[index], 'f', 3));
            _TreeItems.motion.gyroscope[index]->setText(COL, QString::number(msg->gyroscope[index], 'f', 3));
        }
        for (int index = 0; index < 4; index++) {
            _TreeItems.motion.wheel_velocity[index]->setText(COL, QString::number(msg->wheel_velocity_meas[index], 'f', 3));
            _TreeItems.motion.wheel_current_d[index]->setText(COL, QString::number(msg->wheel_current_meas_d[index], 'f', 3));
            _TreeItems.motion.wheel_current_q[index]->setText(COL, QString::number(msg->wheel_current_meas_q[index], 'f', 3));
        }
        for (int index = 0; index < 4; index++) {
            _TreeItems.control.wheel_velocity_ref[index]->setText(COL, QString::number(msg->wheel_velocity_ref[index], 'f', 3));
            _TreeItems.control.wheel_current_ref[index]->setText(COL, QString::number(msg->wheel_current_ref[index], 'f', 3));
            _TreeItems.control.wheel_current_limit[index]->setText(COL, QString::number(msg->wheel_current_limit[index], 'f', 4));
        }
        for (int index = 0; index < 3; index++) {
            _TreeItems.control.machine_velocity[index]->setText(COL, QString::number(msg->machine_velocity[index], 'f', 3));
        }
        _TreeItems.control.slip_flags->setText(COL, QString::number(msg->slip_flags, 'f', 3));
        _TreeItems.control.perf_counter->setText(COL, QString::number(msg->performance_counter));
    }
}

void MainWindow::generateTelemetryTreeItems(void) {
    // diagnosticsの内容を表示するアイテムを作成する
    auto diag_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Diagnostics"});
    _TreeItems.diag.timestamp = new QTreeWidgetItem(diag_top, {"Timestamp", "", "s"});
    _TreeItems.diag.level = new QTreeWidgetItem(diag_top, {"Level"});
    _TreeItems.diag.message = new QTreeWidgetItem(diag_top, {"Message"});
    _TreeItems.diag.error = new QTreeWidgetItem(diag_top, {"Error"});
    _TreeItems.diag.fault = new QTreeWidgetItem(diag_top, {"Fault"});

    // batteryの内容を表示するアイテムを作成する
    auto battery_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Battery"});
    _TreeItems.battery.present = new QTreeWidgetItem(battery_top, {"Battery Present"});
    _TreeItems.battery.voltage = new QTreeWidgetItem(battery_top, {"Battery Voltage", "", "V"});
    _TreeItems.battery.current = new QTreeWidgetItem(battery_top, {"Battery Current", "", "A"});
    _TreeItems.battery.temperature = new QTreeWidgetItem(battery_top, {"Board Temperature", "", u8"\u2103"});

    // stream_data_adc2の内容を表示するアイテムを作成する
    auto adc2_top = new QTreeWidgetItem(_Ui->telemetryTree, {"ADC2"});
    _TreeItems.adc2.dc48v_voltage = new QTreeWidgetItem(adc2_top, {"DC48V Voltage", "", "V"});
    _TreeItems.adc2.dribble_voltage = new QTreeWidgetItem(adc2_top, {"Dribble Voltage", "", "V"});
    _TreeItems.adc2.dribble_current = new QTreeWidgetItem(adc2_top, {"Dribble Current", "", "A"});

    // stream_data_motionの内容を表示するアイテムを作成する
    auto motion_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Motion"});
    _TreeItems.motion.accelerometer[0] = new QTreeWidgetItem(motion_top, {"Accelerometer X", "", u8"m/s\u00B2"});
    _TreeItems.motion.accelerometer[1] = new QTreeWidgetItem(motion_top, {"Accelerometer Y", "", u8"m/s\u00B2"});
    _TreeItems.motion.accelerometer[2] = new QTreeWidgetItem(motion_top, {"Accelerometer Z", "", u8"m/s\u00B2"});
    _TreeItems.motion.gyroscope[0] = new QTreeWidgetItem(motion_top, {"Gyroscope X", "", "rad/s"});
    _TreeItems.motion.gyroscope[1] = new QTreeWidgetItem(motion_top, {"Gyroscope Y", "", "rad/s"});
    _TreeItems.motion.gyroscope[2] = new QTreeWidgetItem(motion_top, {"Gyroscope Z", "", "rad/s"});
    _TreeItems.motion.wheel_velocity[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_velocity[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_velocity[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_velocity[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_current_d[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Current Q", "", "A"});
    _TreeItems.motion.wheel_current_d[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Current Q", "", "A"});
    _TreeItems.motion.wheel_current_d[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Current Q", "", "A"});
    _TreeItems.motion.wheel_current_d[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Current Q", "", "A"});

    // stream_data_controlの内容を表示するアイテムを作成する
    auto control_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Control"});
    _TreeItems.control.perf_counter = new QTreeWidgetItem(control_top, {"Performance Counter", "", "Cycles"});
    _TreeItems.control.wheel_velocity_ref[0] = new QTreeWidgetItem(control_top, {"Wheel 1 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_velocity_ref[1] = new QTreeWidgetItem(control_top, {"Wheel 2 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_velocity_ref[2] = new QTreeWidgetItem(control_top, {"Wheel 3 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_velocity_ref[3] = new QTreeWidgetItem(control_top, {"Wheel 4 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_current_ref[0] = new QTreeWidgetItem(control_top, {"Wheel 1 Current Ref", "", "A"});
    _TreeItems.control.wheel_current_ref[1] = new QTreeWidgetItem(control_top, {"Wheel 2 Current Ref", "", "A"});
    _TreeItems.control.wheel_current_ref[2] = new QTreeWidgetItem(control_top, {"Wheel 3 Current Ref", "", "A"});
    _TreeItems.control.wheel_current_ref[3] = new QTreeWidgetItem(control_top, {"Wheel 4 Current Ref", "", "A"});
    _TreeItems.control.wheel_current_limit[0] = new QTreeWidgetItem(control_top, {"Wheel 1 Current Limit", "", "A"});
    _TreeItems.control.wheel_current_limit[1] = new QTreeWidgetItem(control_top, {"Wheel 2 Current Limit", "", "A"});
    _TreeItems.control.wheel_current_limit[2] = new QTreeWidgetItem(control_top, {"Wheel 3 Current Limit", "", "A"});
    _TreeItems.control.wheel_current_limit[3] = new QTreeWidgetItem(control_top, {"Wheel 4 Current Limit", "", "A"});
    _TreeItems.control.machine_velocity[0] = new QTreeWidgetItem(control_top, {"Machine Velocity X", "", "m/s"});
    _TreeItems.control.machine_velocity[1] = new QTreeWidgetItem(control_top, {"Machine Velocity Y", "", "m/s"});
    _TreeItems.control.machine_velocity[2] = new QTreeWidgetItem(control_top, {u8"Machine Velocity \u03C9", "", "rad/s"});
    _TreeItems.control.slip_flags = new QTreeWidgetItem(control_top, {"Slip Flags", "", ""});

    // カラム幅を文字に合わせてリサイズする
    _Ui->telemetryTree->expandAll();
    _Ui->telemetryTree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    _Ui->telemetryTree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    _Ui->telemetryTree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
}

void MainWindow::startLogging(void) {
    QString path = QFileDialog::getSaveFileName(this, "Save Log File", "", "CSV (*.csv)");
    if (path.isEmpty()) {
        return;
    }
    auto file = std::make_shared<QFile>(path);
    if (file->open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream stream(file.get());
        stream << "Time";
        for (int index = 1; index <= 4; index++)
            stream << ",Velocity " << index << " Meas";
        for (int index = 1; index <= 4; index++)
            stream << ",Current " << index << " Meas";
        for (int index = 1; index <= 4; index++)
            stream << ",Velocity " << index << " Ref";
        for (int index = 1; index <= 4; index++)
            stream << ",Current " << index << " Ref";
        for (int index = 1; index <= 4; index++)
            stream << ",Energy " << index;
        stream << ",Machine Vx,Machine Vy,Machine Omega";
        stream << ",Slip Flags";
        stream << ",Accel X,Accel Y,Accel Z";
        stream << ",Gyro X,Gyro Y,Gyro Z";
        stream << ",DC48V,Battery Voltage,Battery Current\n";
        stream.flush();
        _LogFrameNumber = 0;
        _LogFile = file;
    }
    _Ui->saveLogButton->setEnabled(false);
    _Ui->stopLogButton->setEnabled(true);
}

void MainWindow::stopLogging(void) {
    _LogFile.reset();
    _Ui->saveLogButton->setEnabled(true);
    _Ui->stopLogButton->setEnabled(false);
}

void MainWindow::quitNodeThread(void) {
    if (_NodeThread != nullptr) {
        // Subscriptionを破棄する
        _Subscribers.battery.reset();
        _Subscribers.adc2.reset();
        _Subscribers.motion.reset();
        _Subscribers.image.reset();
        _Subscribers.diagnostics.reset();

        // Publisherを破棄する
        _publishers.velocity.reset();

        // Clientsを破棄する
        _Ui->programNiosButton->setEnabled(false);
        _Ui->programFpgaButton->setEnabled(false);
        _Ui->selfTestButton->setEnabled(false);
        _Clients.program_nios.reset();
        _Clients.program_fpga.reset();
        _Clients.self_test.reset();

        // スレッドを終了する
        // deleteはdeleteLater()スロットにより行われるのでここでする必要はない
        _NodeThread->requestInterruption();
        _NodeThread->quit();
        _NodeThread->wait(std::chrono::milliseconds(NodeThread::QUIT_TIMEOUT));
        _NodeThread = nullptr;
    }
}

void MainWindow::sendCommand(void) {
    if (_publishers.velocity && _Ui->enableControllerCheckBox->isChecked()) {
        geometry_msgs::msg::Twist msg;
        bool gamepad_selected;
        int gamepad_device_id = _Ui->gamepadComboBox->currentData().toInt(&gamepad_selected);
        if (gamepad_selected) {
            auto input_state = _GamepadThread->inputState(gamepad_device_id);
            if (input_state) {
                msg.linear.x = 4.0f * input_state->leftStickX;
                msg.linear.y = 4.0f * input_state->leftStickY;
                msg.linear.z = -input_state->rightTrigger;
                msg.angular.z = -10.0f * input_state->rightStickX;
                _publishers.velocity->publish(msg);
            }
        }
        else {
            msg.linear.x = _Pad.velocity_scale_x;
            msg.linear.y = _Pad.velocity_scale_y;
            msg.angular.z = _Pad.velocity_scale_omega * 0.1;
            _publishers.velocity->publish(msg);
        }
    }
}

void MainWindow::programNios(void) {
    static const QString TITLE("Program NIOS");

    QString path = QFileDialog::getOpenFileName(this, "Open HEX File", "", "Intel HEX (*.hex)");
    if (path.isEmpty()) {
        return;
    }

    // Hexファイルを読み込む
    auto request = std::make_shared<phoenix_msgs::srv::ProgramNios::Request>();
    ihex_recordset_t *recored_set = ihex_rs_from_file(path.toStdString().c_str());
    size_t copied_bytes = 0;
    uint_t i = 0;
    ihex_record_t *record;
    int err;
    do {
        uint32_t offset;
        err = ihex_rs_iterate_data(recored_set, &i, &record, &offset);
        if (err || record == 0)
            break;
        uint32_t address = offset + record->ihr_address;
        uint32_t length = record->ihr_length;
        if ((length % 4) != 0) {
            err = 1;
            break;
        }
        if ((0 <= address * 4) && ((address * 4 + length) <= request->program.size())) {
            const uint32_t *src = reinterpret_cast<const uint32_t *>(record->ihr_data);
            uint32_t *dst = reinterpret_cast<uint32_t *>(request->program.data() + address * 4);
            for (size_t index = 0; index < (length / 4); index++) {
                uint32_t data = *src++;
                *dst++ = ((data >> 24) & 0xFFu) | ((data >> 8) & 0xFF00u) | ((data << 8) & 0xFF0000u) | (data << 24);
            }
            copied_bytes += length;
        }
    } while (0 < i);
    ihex_rs_free(recored_set);
    if (err || (copied_bytes != request->program.size())) {
        QMessageBox::critical(this, TITLE, "An error occured while loading HEX file");
        return;
    }

    // プログラムを転送する
    auto result = _Clients.program_nios->async_send_request(request);
    result.wait();
    auto response = result.get();
    if (!response) {
        QMessageBox::critical(this, TITLE, "Service didn't respond");
    }
    else if (!response->succeeded) {
        QMessageBox::critical(this, TITLE, "An error occured while programming memory");
    }
    else {
        QMessageBox::information(this, TITLE, "Finished");
    }
}

void MainWindow::programFpga(void) {
    static const QString TITLE("Program FPGA");

    QString path = QFileDialog::getOpenFileName(this, "Open RPD File", "", "Raw Programming Data File (*.rpd)");
    if (path.isEmpty()) {
        return;
    }

    // RPDファイルを読み込む
    auto request = std::make_shared<phoenix_msgs::srv::ProgramFpga::Request>();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, TITLE, "Failed to load file");
        return;
    }
    QByteArray binary = file.readAll();
    file.close();
    if (request->bitstream.size() < (size_t)binary.size()) {
        QMessageBox::critical(this, TITLE, QString("Size of the RPD file must be less than %1 bytes").arg(request->bitstream.size()));
        return;
    }
    memcpy(request->bitstream.data(), binary.constData(), binary.size());
    memset(request->bitstream.data() + binary.size(), 0xFF, request->bitstream.size() - (size_t)binary.size());

    // ビットストリームを転送する
    auto result = _Clients.program_fpga->async_send_request(request);
    result.wait();
    auto response = result.get();
    if (!response) {
        QMessageBox::critical(this, TITLE, "Service didn't respond");
    }
    else if (!response->succeeded) {
        QMessageBox::critical(this, TITLE, "An error occured while programming bitstream");
    }
    else {
        QMessageBox::information(this, TITLE, "Finished");
    }
}

void MainWindow::runSelfTest(void) {
    static const QString TITLE("Self Test");
    auto request = std::make_shared<diagnostic_msgs::srv::SelfTest::Request>();
    auto result = _Clients.self_test->async_send_request(request);
    result.wait();
    auto response = result.get();
    if (!response) {
        QMessageBox::critical(this, TITLE, "Service didn't respond");
    }

    // 結果を表示する。
    std::stringstream ss;
    ss << (response->passed ? "passed:" : "failed:");
    auto it = std::find_if(response->status.begin(), response->status.end(), [&](const diagnostic_msgs::msg::DiagnosticStatus &diag) {
        return diag.hardware_id == _namespace;
    });
    if (it != response->status.end()) {
        ss << std::endl << "  name: " << it->name;
        ss << std::endl << "  message: " << it->message;
        if (!it->values.empty()) {
            ss << std::endl << "  values:";
            for (auto &item : it->values) {
                ss << std::endl << "    " << item.key << ": '" << item.value << '\'';
            }
        }
    }
    else {
        ss << " Unknown detail";
    }
    if (response->passed) {
        QMessageBox::information(this, TITLE, QString::fromStdString(ss.str()));
    }
    else {
        QMessageBox::critical(this, TITLE, QString::fromStdString(ss.str()));
    }
}

void MainWindow::updateDiagnosticsInformation(TreeItems_t::DiagnosticItems_t &items, const std_msgs::msg::Header &header,
                                              const diagnostic_msgs::msg::DiagnosticStatus &status) {
    // timestampをミリ秒単位で表示する
    items.timestamp->setText(COL, QString("%1.%2").arg(header.stamp.sec).arg(header.stamp.nanosec / 1000000, 3, 10, QLatin1Char('0')));

    // messageを表示する
    items.message->setText(COL, QString::fromStdString(status.message));

    // levelを表示する
    static const std::unordered_map<decltype(status.level), QString> level_map = {
        {diagnostic_msgs::msg::DiagnosticStatus::OK, "OK"},
        {diagnostic_msgs::msg::DiagnosticStatus::WARN, "WARN"},
        {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "ERROR"},
        {diagnostic_msgs::msg::DiagnosticStatus::STALE, "STALE"},
    };
    auto it = level_map.find(status.level);
    if (it != level_map.end()) {
        items.level->setText(COL, it->second);
    }
    else {
        items.level->setText(COL, QString::number(status.level));
    }

    // エラーフラグとフォルトフラグを表示する
    static const QString none("none");
    bool error_occured = false, fault_occured = false;
    for (auto &item : status.values) {
        if (!error_occured && (item.key == "Error Flags")) {
            items.error->setText(COL, QString::fromStdString(item.value));
            error_occured = true;
        }
        else if (!fault_occured && (item.key == "Fault Flags")) {
            items.fault->setText(COL, QString::fromStdString(item.value));
            fault_occured = true;
        }
    }
    if (!error_occured) {
        items.error->setText(COL, none);
    }
    if (!fault_occured) {
        items.fault->setText(COL, none);
    }
}

std::shared_ptr<rclcpp::Node> MainWindow::createNode(void) {
    // ノードを作成する
    // ノード名の被りを防止するため末尾に乱数を付与する
    QString node_name = QString("%1%2").arg(GUI_NODE_NAME_PREFIX).arg(std::random_device()(), 8, 16, QLatin1Char('0'));
    QString namespace_name = QSysInfo::machineHostName();
    return std::make_shared<rclcpp::Node>(node_name.toStdString(), namespace_name.toStdString());
}

void MainWindow::quitGamepadThread(void) {
    _GamepadThread->requestInterruption();
    _GamepadThread->quit();
    _GamepadThread->wait(std::chrono::milliseconds(NodeThread::QUIT_TIMEOUT));
    _GamepadThread = nullptr;
}
