#include "telemetry_viewer.hpp"
#include "ui_telemetry_viewer.h"
#include "../../phoenix/include/phoenix.hpp"
#include "format_value.hpp"
#include "common.hpp"
#include <QtCore/QTimer>
#include <QtCore/QTextStream>
#include <QtWidgets/QFileDialog>

using namespace std::chrono_literals;

constexpr auto SERVICE_TIMEOUT = 1s;

TelemetryViewer::TelemetryViewer(QWidget *parent) : QGroupBox(parent) {
    // UIを生成する
    _ui = new Ui_TelemetryViewer;
    _ui->setupUi(this);
    generateTelemetryTreeItems();

    connect(_ui->saveLogButton, &QPushButton::clicked, this, &TelemetryViewer::startLogging);
    connect(_ui->stopLogButton, &QPushButton::clicked, this, &TelemetryViewer::stopLogging);

    // テレメトリ更新用のタイマーを生成する
    QTimer *timer = new QTimer(this);
    timer->start(100);
    connect(timer, &QTimer::timeout, this, &TelemetryViewer::updateTelemertyTreeItems);
}

TelemetryViewer::~TelemetryViewer() {}

void TelemetryViewer::initializeNode(rclcpp::Node::SharedPtr node, const std::string &remote_node_namespace) {
    const rclcpp::SensorDataQoS qos_sensor;

    // batteryトピックを受信するSubscriptionを作成する
    _battery_subscription = node->create_subscription<sensor_msgs::msg::BatteryState>(
        constructName(remote_node_namespace, phoenix::TOPIC_NAME_BATTERY), qos_sensor, [this](const std::shared_ptr<sensor_msgs::msg::BatteryState> msg) {
            std::atomic_store(&_battery_message, msg);
        });

    // stream_data_adc2トピックを受信するSubscriptionを作成する
    _adc2_subscription = node->create_subscription<phoenix_msgs::msg::StreamDataAdc2>(
        constructName(remote_node_namespace, phoenix::TOPIC_NAME_ADC2), qos_sensor, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> msg) {
            if (!_adc2_message) {
                std::atomic_store(&_adc2_message, msg);
            }
            else {
                /*std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> last_msg = _adc2_message;
                msg->dc48v_voltage = std::max(msg->dc48v_voltage, last_msg->dc48v_voltage);
                msg->dribble_current = std::max(msg->dribble_current, last_msg->dribble_current);*/
                std::atomic_store(&_adc2_message, msg);
            }
        });

    // stream_data_motionトピックを受信するSubscriptionを作成する
    _motion_subscription = node->create_subscription<phoenix_msgs::msg::StreamDataMotion>(
        constructName(remote_node_namespace, phoenix::TOPIC_NAME_MOTION), qos_sensor, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> msg) {
            std::atomic_store(&_motion_message, msg);
            std::shared_ptr<QFile> file = _log_file;
            if (file) {
                QTextStream stream(file.get());
                QChar sep(',');
                std::shared_ptr<sensor_msgs::msg::BatteryState> battery_msg = _battery_message;
                std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> adc2_msg = _adc2_message;
                float battery_voltage = battery_msg ? battery_msg->voltage : 0.0f;
                float battery_current = battery_msg ? -battery_msg->current : 0.0f;
                float dc48v_voltage = adc2_msg ? adc2_msg->dc48v_voltage : 0.0f;
                stream << (0.001 * _frame_number) << sep;
                stream << msg->accelerometer[0] << sep;
                stream << msg->accelerometer[1] << sep;
                stream << msg->accelerometer[2] << sep;
                stream << msg->gyroscope[0] << sep;
                stream << msg->gyroscope[1] << sep;
                stream << msg->gyroscope[2] << sep;
                stream << msg->gravity[0] << sep;
                stream << msg->gravity[1] << sep;
                stream << msg->gravity[2] << sep;
                stream << msg->body_acceleration[0] << sep;
                stream << msg->body_acceleration[1] << sep;
                stream << msg->body_acceleration[2] << sep;
                stream << msg->body_velocity[0] << sep;
                stream << msg->body_velocity[1] << sep;
                stream << msg->body_velocity[2] << sep;
                stream << msg->wheel_velocity_meas[0] << sep;
                stream << msg->wheel_velocity_meas[1] << sep;
                stream << msg->wheel_velocity_meas[2] << sep;
                stream << msg->wheel_velocity_meas[3] << sep;
                stream << msg->wheel_current_meas_q[0] << sep;
                stream << msg->wheel_current_meas_q[1] << sep;
                stream << msg->wheel_current_meas_q[2] << sep;
                stream << msg->wheel_current_meas_q[3] << sep;
                stream << msg->wheel_current_ref[0] << sep;
                stream << msg->wheel_current_ref[1] << sep;
                stream << msg->wheel_current_ref[2] << sep;
                stream << msg->wheel_current_ref[3] << sep;
                stream << msg->body_ref_accel[0] << sep;
                stream << msg->body_ref_accel[1] << sep;
                stream << msg->body_ref_accel[2] << sep;
                stream << msg->body_ref_accel[3] << sep;
                stream << dc48v_voltage << sep;
                stream << battery_voltage << sep;
                stream << battery_current << Qt::endl;
                _frame_number++;
            }
        });
}

void TelemetryViewer::uninitializeNode(void) {
    _battery_subscription.reset();
    _adc2_subscription.reset();
    _motion_subscription.reset();
}

void TelemetryViewer::updateTelemertyTreeItems(void) {
    if (_battery_message) {
        // auto msg = std::atomic_exchange(&_battery_message, std::shared_ptr<sensor_msgs::msg::BatteryState>());
        auto msg = _battery_message;
        _tree_items.battery.present->setText(VALUE_COLUMN, boolToString(msg->present));
        _tree_items.battery.voltage->setText(VALUE_COLUMN, QString::number(msg->voltage, 'f', 3));
        _tree_items.battery.current->setText(VALUE_COLUMN, QString::number(msg->current, 'f', 3));
        _tree_items.battery.temperature->setText(VALUE_COLUMN, QString::number(msg->temperature, 'f', 2));
    }
    if (_adc2_message) {
        // auto msg = std::atomic_exchange(&_adc2_message, std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2>());
        auto msg = _adc2_message;
        _tree_items.adc2.dc48v_voltage->setText(VALUE_COLUMN, QString::number(msg->dc48v_voltage, 'f', 3));
        _tree_items.adc2.dribble_voltage->setText(VALUE_COLUMN, QString::number(msg->dribble_voltage, 'f', 3));
        _tree_items.adc2.dribble_current->setText(VALUE_COLUMN, QString::number(msg->dribble_current, 'f', 3));
    }
    if (_motion_message) {
        // auto msg = std::atomic_exchange(&_motion_message, std::shared_ptr<phoenix_msgs::msg::StreamDataMotion>());
        auto msg = _motion_message;
        for (int index = 0; index < 3; index++) {
            _tree_items.motion.accelerometer[index]->setText(VALUE_COLUMN, QString::number(msg->accelerometer[index], 'f', 3));
            _tree_items.motion.gyroscope[index]->setText(VALUE_COLUMN, QString::number(msg->gyroscope[index], 'f', 3));
            _tree_items.motion.gravity[index]->setText(VALUE_COLUMN, QString::number(msg->gravity[index], 'f', 3));
            _tree_items.motion.body_acceleration[index]->setText(VALUE_COLUMN, QString::number(msg->body_acceleration[index], 'f', 3));
            _tree_items.motion.body_velocity[index]->setText(VALUE_COLUMN, QString::number(msg->body_velocity[index], 'f', 3));
        }
        for (int index = 0; index < 4; index++) {
            _tree_items.motion.wheel_velocity[index]->setText(VALUE_COLUMN, QString::number(msg->wheel_velocity_meas[index], 'f', 3));
            _tree_items.motion.wheel_current_d[index]->setText(VALUE_COLUMN, QString::number(msg->wheel_current_meas_d[index], 'f', 3));
            _tree_items.motion.wheel_current_q[index]->setText(VALUE_COLUMN, QString::number(msg->wheel_current_meas_q[index], 'f', 3));
        }
        for (int index = 0; index < 4; index++) {
            _tree_items.control.wheel_current_ref[index]->setText(VALUE_COLUMN, QString::number(msg->wheel_current_ref[index], 'f', 3));
            _tree_items.control.body_ref_accel[index]->setText(VALUE_COLUMN, QString::number(msg->body_ref_accel[index], 'f', 3));
        }
        _tree_items.control.perf_counter->setText(VALUE_COLUMN, QString::number(msg->performance_counter));
    }
}

void TelemetryViewer::startLogging(void) {
    QString path = QFileDialog::getSaveFileName(this, "Save Log File", "", "CSV (*.csv)");
    if (path.isEmpty()) {
        return;
    }
    auto file = std::make_shared<QFile>(path);
    if (file->open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream stream(file.get());
        stream << "Time";
        stream << ",AccelX,AccelY,AccelZ";
        stream << ",GyroX,GyroY,GyroZ";
        stream << ",GravityX,GravityY,GravityZ";
        stream << ",BodyAccelX,BodyAccelY,BodyAccelZ";
        stream << ",BodyVelocityX,BodyVelocityY,BodyVelocityW";
        for (int index = 1; index <= 4; index++)
            stream << ",Wheel" << index << "Velocity";
        for (int index = 1; index <= 4; index++)
            stream << ",Wheel" << index << "CurrentMeas";
        for (int index = 1; index <= 4; index++)
            stream << ",Wheel" << index << "CurrentRef";
        stream << ",BodyRefAccelX,BodyRefAccelY,BodyRefAccelW,BodyRefAccelC";
        stream << ",DC48V,Battery Voltage,Battery Current\n";
        stream.flush();
        _frame_number = 0;
        _log_file = file;
    }
    _ui->saveLogButton->setEnabled(false);
    _ui->stopLogButton->setEnabled(true);
}

void TelemetryViewer::stopLogging(void) {
    _log_file.reset();
    _ui->saveLogButton->setEnabled(true);
    _ui->stopLogButton->setEnabled(false);
}

void TelemetryViewer::generateTelemetryTreeItems(void) {
    // batteryの内容を表示するアイテムを作成する
    auto battery_top = new QTreeWidgetItem(_ui->telemetryTree, {"Battery"});
    _tree_items.battery.present = new QTreeWidgetItem(battery_top, {"Battery Present"});
    _tree_items.battery.voltage = new QTreeWidgetItem(battery_top, {"Battery Voltage", "", "V"});
    _tree_items.battery.current = new QTreeWidgetItem(battery_top, {"Battery Current", "", "A"});
    _tree_items.battery.temperature = new QTreeWidgetItem(battery_top, {"Board Temperature", "", u8"\u2103"});

    // stream_data_adc2の内容を表示するアイテムを作成する
    auto adc2_top = new QTreeWidgetItem(_ui->telemetryTree, {"ADC2"});
    _tree_items.adc2.dc48v_voltage = new QTreeWidgetItem(adc2_top, {"DC48V Voltage", "", "V"});
    _tree_items.adc2.dribble_voltage = new QTreeWidgetItem(adc2_top, {"Dribble Voltage", "", "V"});
    _tree_items.adc2.dribble_current = new QTreeWidgetItem(adc2_top, {"Dribble Current", "", "A"});

    // stream_data_motionの内容を表示するアイテムを作成する
    auto motion_top = new QTreeWidgetItem(_ui->telemetryTree, {"Motion"});
    _tree_items.motion.accelerometer[0] = new QTreeWidgetItem(motion_top, {"Accelerometer X", "", u8"m/s\u00B2"});
    _tree_items.motion.accelerometer[1] = new QTreeWidgetItem(motion_top, {"Accelerometer Y", "", u8"m/s\u00B2"});
    _tree_items.motion.accelerometer[2] = new QTreeWidgetItem(motion_top, {"Accelerometer Z", "", u8"m/s\u00B2"});
    _tree_items.motion.gyroscope[0] = new QTreeWidgetItem(motion_top, {"Gyroscope X", "", "rad/s"});
    _tree_items.motion.gyroscope[1] = new QTreeWidgetItem(motion_top, {"Gyroscope Y", "", "rad/s"});
    _tree_items.motion.gyroscope[2] = new QTreeWidgetItem(motion_top, {"Gyroscope Z", "", "rad/s"});
    _tree_items.motion.gravity[0] = new QTreeWidgetItem(motion_top, {"Gravity X", "", u8"m/s\u00B2"});
    _tree_items.motion.gravity[1] = new QTreeWidgetItem(motion_top, {"Gravity Y", "", u8"m/s\u00B2"});
    _tree_items.motion.gravity[2] = new QTreeWidgetItem(motion_top, {"Gravity Z", "", u8"m/s\u00B2"});
    _tree_items.motion.body_acceleration[0] = new QTreeWidgetItem(motion_top, {"Body Acceleration X", "", u8"m/s\u00B2"});
    _tree_items.motion.body_acceleration[1] = new QTreeWidgetItem(motion_top, {"Body Acceleration Y", "", u8"m/s\u00B2"});
    _tree_items.motion.body_acceleration[2] = new QTreeWidgetItem(motion_top, {"Body Acceleration Z", "", u8"m/s\u00B2"});
    _tree_items.motion.body_velocity[0] = new QTreeWidgetItem(motion_top, {"Body Velocity X", "", "m/s"});
    _tree_items.motion.body_velocity[1] = new QTreeWidgetItem(motion_top, {"Body Velocity Y", "", "m/s"});
    _tree_items.motion.body_velocity[2] = new QTreeWidgetItem(motion_top, {u8"Body Velocity \u03C9", "", "rad/s"});
    _tree_items.motion.wheel_velocity[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Velocity", "", "m/s"});
    _tree_items.motion.wheel_velocity[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Velocity", "", "m/s"});
    _tree_items.motion.wheel_velocity[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Velocity", "", "m/s"});
    _tree_items.motion.wheel_velocity[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Velocity", "", "m/s"});
    _tree_items.motion.wheel_current_d[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Current D", "", "A"});
    _tree_items.motion.wheel_current_q[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Current Q", "", "A"});
    _tree_items.motion.wheel_current_d[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Current D", "", "A"});
    _tree_items.motion.wheel_current_q[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Current Q", "", "A"});
    _tree_items.motion.wheel_current_d[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Current D", "", "A"});
    _tree_items.motion.wheel_current_q[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Current Q", "", "A"});
    _tree_items.motion.wheel_current_d[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Current D", "", "A"});
    _tree_items.motion.wheel_current_q[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Current Q", "", "A"});

    // stream_data_controlの内容を表示するアイテムを作成する
    auto control_top = new QTreeWidgetItem(_ui->telemetryTree, {"Control"});
    _tree_items.control.perf_counter = new QTreeWidgetItem(control_top, {"Performance Counter", "", "Cycles"});
    _tree_items.control.wheel_current_ref[0] = new QTreeWidgetItem(control_top, {"Wheel 1 Current Ref", "", "A"});
    _tree_items.control.wheel_current_ref[1] = new QTreeWidgetItem(control_top, {"Wheel 2 Current Ref", "", "A"});
    _tree_items.control.wheel_current_ref[2] = new QTreeWidgetItem(control_top, {"Wheel 3 Current Ref", "", "A"});
    _tree_items.control.wheel_current_ref[3] = new QTreeWidgetItem(control_top, {"Wheel 4 Current Ref", "", "A"});
    _tree_items.control.body_ref_accel[0] = new QTreeWidgetItem(control_top, {"Ref Acceleration X", "", u8"m/s\u00B2"});
    _tree_items.control.body_ref_accel[1] = new QTreeWidgetItem(control_top, {"Ref Acceleration Y", "", u8"m/s\u00B2"});
    _tree_items.control.body_ref_accel[2] = new QTreeWidgetItem(control_top, {u8"Ref Acceleration \u03C9", "", u8"rad/s\u00B2"});
    _tree_items.control.body_ref_accel[3] = new QTreeWidgetItem(control_top, {"Ref Acceleration C", "", u8"m/s\u00B2"});

    // カラム幅を文字に合わせてリサイズする
    _ui->telemetryTree->expandAll();
    _ui->telemetryTree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    _ui->telemetryTree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    _ui->telemetryTree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
}
