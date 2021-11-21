#include "telemetry_viewer.hpp"
#include "ui_telemetry_viewer.h"
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <QtCore/QTimer>
#include <QtCore/QTextStream>
#include <QtWidgets/QFileDialog>
#include <QtCore/QtDebug>

TelemetryViewer::TelemetryViewer(QWidget *parent) : QGroupBox(parent) {
    // UIを生成する
    _ui = new Ui_TelemetryViewer;
    _ui->setupUi(this);

    // カラム幅を文字に合わせてリサイズする
    _ui->telemetryTree->expandAll();
    _ui->telemetryTree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    _ui->telemetryTree->header()->setSectionResizeMode(1, QHeaderView::Stretch);

    connect(_ui->saveLogButton, &QPushButton::clicked, this, &TelemetryViewer::startLogging);
    connect(_ui->stopLogButton, &QPushButton::clicked, this, &TelemetryViewer::stopLogging);

    // テレメトリ更新用のタイマーを生成する
    QTimer *timer = new QTimer(this);
    timer->start(100);
    connect(timer, &QTimer::timeout, this, &TelemetryViewer::updateTelemertyTreeItems);
}

TelemetryViewer::~TelemetryViewer() {}

void TelemetryViewer::uninitializeNode(void) {
    _subscribers.clear();
}

void TelemetryViewer::updateTelemertyTreeItems(void) {
    for (auto &subscriber : _subscribers) {
        subscriber->update();
    }
}

void TelemetryViewer::startLogging(void) {
    /*QString path = QFileDialog::getSaveFileName(this, "Save Log File", "", "CSV (*.csv)");
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
    _ui->stopLogButton->setEnabled(true);*/
}

void TelemetryViewer::stopLogging(void) {
    /*_log_file.reset();
    _ui->saveLogButton->setEnabled(true);
    _ui->stopLogButton->setEnabled(false);*/
}

void TelemetryViewer::SubscriberBase::updateImpl(Field_t &parent_field, const void *ptr, const rosidl_message_type_support_t *type_support) {
    auto members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);
    size_t member_count = std::min(static_cast<size_t>(members->member_count_), parent_field.children.size());
    for (size_t index = 0; index < member_count; index++) {
        Field_t &child_field = parent_field.children[index];
        const rosidl_typesupport_introspection_cpp::MessageMember *member = members->members_ + index;
        const void *child_ptr = reinterpret_cast<const uint8_t *>(ptr) + member->offset_;
        if (member->is_array_) {
            if (member->array_size_ != 0) {
                // 固定長配列
                if (member->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                    // プリミティブ型の固定長配列
                    size_t element_count = std::min(member->array_size_, child_field.elements.size());
                    for (size_t array_index = 0; array_index < element_count; array_index++) {
                        const void *value = member->get_const_function(child_ptr, array_index);
                        updatePrimitive(child_field.elements[array_index], member->type_id_, value);
                    }
                }
                else {
                    // メッセージの固定長配列
                    size_t element_count = std::min(member->array_size_, child_field.children.size());
                    for (size_t array_index = 0; array_index < element_count; array_index++) {
                        Field_t &grandchild_field = child_field.children[array_index];
                        const void *grandchild_ptr = member->get_const_function(child_ptr, array_index);
                        updateImpl(grandchild_field, grandchild_ptr, member->members_);
                    }
                }
            }
            else {
                // 可変長配列は非対応
            }
        }
        else {
            // 単独
            if (member->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                // プリミティブ型
                updatePrimitive(child_field.item, member->type_id_, child_ptr);
            }
            else {
                // メッセージ
                updateImpl(child_field, child_ptr, member->members_);
            }
        }
    }
}

void TelemetryViewer::SubscriberBase::updatePrimitive(QTreeWidgetItem *item, uint8_t type, const void *ptr) {
    static const QString true_text("true");
    static const QString false_text("false");
    switch (type) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const float *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const double *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        item->setText(VALUE_COLUMN, QChar(*reinterpret_cast<const char *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
        item->setText(VALUE_COLUMN, QChar(*reinterpret_cast<const wchar_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
        item->setText(VALUE_COLUMN, *reinterpret_cast<const bool *>(ptr) ? true_text : false_text);
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
        [[fallthrough]];
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const uint8_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const int8_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const uint16_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const int16_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const uint32_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const int32_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const uint64_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        item->setText(VALUE_COLUMN, QString::number(*reinterpret_cast<const int64_t *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        item->setText(VALUE_COLUMN, QString::fromStdString(*reinterpret_cast<const std::string *>(ptr)));
        break;

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        item->setText(VALUE_COLUMN, QString::fromStdWString(*reinterpret_cast<const std::wstring *>(ptr)));
        break;

    default:
        item->setText(VALUE_COLUMN, QString());
        break;
    }
}

void TelemetryViewer::addSubscriptionImpl(const std::string &topic_name, std::shared_ptr<SubscriberBase> subscriber) {
    // ツリーにトップレベルアイテムを作成する
    // qDebug("QTreeWidgetItem(%s)", topic_name.c_str());
    subscriber->top.item = new QTreeWidgetItem(_ui->telemetryTree, {QString::fromStdString(topic_name)});
    subscriber->top.item->setExpanded(true);

    // ツリーに各フィールドの項目を作成する
    // auto typesupport = subscriber->handle();
    // Field_t &parent = subscriber->top;
    // std::string base_name("");

    createTreeItems(subscriber->top, "", subscriber->handle());

    _subscribers.push_back(subscriber);
}

void TelemetryViewer::createTreeItems(Field_t &parent_field, const QString &base_name, const rosidl_message_type_support_t *type_support) {
    auto members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);
    // qDebug("members = %p", members);
    // qDebug("members.namespace = '%s'", members->message_namespace_);
    // qDebug("members.name = '%s'", members->message_name_);
    // qDebug("members.count = %u", members->member_count_);

    // qDebug("members : ns = %p, name = %p, count = %p", members->message_namespace_, members->message_name_, &members->member_count_);
    size_t member_count = members->member_count_;
    parent_field.children.resize(member_count);
    for (size_t index = 0; index < member_count; index++) {
        Field_t &child_field = parent_field.children[index];
        const rosidl_typesupport_introspection_cpp::MessageMember *member = members->members_ + index;
        // qDebug("member[%zu] : name = %s, type = %d, is_array = %d, size = %zu", index, member->name_, member->type_id_, member->is_array_,
        // member->array_size_);
        QString field_name = member->name_;
        if (member->is_array_) {
            if (member->array_size_ != 0) {
                // 固定長配列
                size_t element_count = std::min(member->array_size_, MAX_ARRAY_ELEMENTS);
                if (member->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                    // プリミティブ型の固定長配列
                    child_field.elements.resize(element_count);
                    for (size_t array_index = 0; array_index < element_count; array_index++) {
                        QString text = QString("%1%2[%3]").arg(base_name).arg(field_name).arg(array_index);
                        // qDebug("QTreeWidgetItem(%s)", text.toStdString().c_str());
                        child_field.elements[array_index] = new QTreeWidgetItem(parent_field.item, {text});
                    }
                }
                else {
                    // メッセージの固定長配列
                    child_field.children.resize(element_count);
                    for (size_t array_index = 0; array_index < element_count; array_index++) {
                        QString text = QString("%1%2[%3]").arg(base_name).arg(field_name).arg(array_index);
                        // qDebug("QTreeWidgetItem(%s)", text.toStdString().c_str());
                        child_field.children[array_index].item = new QTreeWidgetItem(parent_field.item, {text});
                        child_field.children[array_index].item->setExpanded(true);
                        text.append('.');
                        createTreeItems(child_field.children[array_index], text, member->members_);
                    }
                }
            }
            else {
                // 可変長配列は非対応
            }
        }
        else {
            // 単独
            QString text = QString("%1%2").arg(base_name).arg(field_name);
            // qDebug("QTreeWidgetItem(%s)", text.toStdString().c_str());
            child_field.item = new QTreeWidgetItem(parent_field.item, {text});
            if (member->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                // メッセージ
                child_field.item->setExpanded(true);
                text.append('.');
                createTreeItems(child_field, text, member->members_);
            }
        }
    }
}
