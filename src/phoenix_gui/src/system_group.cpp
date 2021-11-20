#include "system_group.hpp"
#include "ui_system_group.h"
#include "common.hpp"
#include "cintelhex/cintelhex.h"
#include "../../phoenix/include/phoenix.hpp"
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

using namespace std::chrono_literals;

constexpr auto SERVICE_TIMEOUT = 1s;

SystemGroup::SystemGroup(QWidget *parent) : QGroupBox(parent) {
    // UIを生成する
    _ui = new Ui_SystemGroup;
    _ui->setupUi(this);

    connect(_ui->programNiosButton, &QPushButton::clicked, this, &SystemGroup::programNios);
    connect(_ui->programFpgaButton, &QPushButton::clicked, this, &SystemGroup::programFpga);
}

SystemGroup::~SystemGroup() {}

void SystemGroup::initializeNode(rclcpp::Node::SharedPtr node, const std::string &remote_node_namespace) {
    // NiosII書き換えサービスと接続する
    _program_nios_service = node->create_client<phoenix_msgs::srv::ProgramNios>(constructName(remote_node_namespace, phoenix::SERVICE_NAME_PROGRAM_NIOS));
    if (_program_nios_service->wait_for_service(SERVICE_TIMEOUT) == false) {
        _program_nios_service.reset();
        _ui->programNiosButton->setEnabled(false);
    }
    else {
        _ui->programNiosButton->setEnabled(true);
    }

    // FPGA書き換えサービスと接続する
    _program_fpga_service = node->create_client<phoenix_msgs::srv::ProgramFpga>(constructName(remote_node_namespace, phoenix::SERVICE_NAME_PROGRAM_FPGA));
    if (_program_fpga_service->wait_for_service(SERVICE_TIMEOUT) == false) {
        _program_fpga_service.reset();
        _ui->programFpgaButton->setEnabled(false);
    }
    else {
        _ui->programFpgaButton->setEnabled(true);
    }
}

void SystemGroup::uninitializeNode(void) {
    _ui->programNiosButton->setEnabled(false);
    _ui->programFpgaButton->setEnabled(false);
    _program_nios_service.reset();
    _program_fpga_service.reset();
}

void SystemGroup::programNios(void) {
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
        QMessageBox::critical(this, title(), "An error occured while loading HEX file");
        return;
    }

    // プログラムを転送する
    auto result = _program_nios_service->async_send_request(request);
    result.wait();
    auto response = result.get();
    if (!response) {
        QMessageBox::critical(this, title(), "Service didn't respond");
    }
    else if (!response->succeeded) {
        QMessageBox::critical(this, title(), "An error occured while programming memory");
    }
    else {
        QMessageBox::information(this, title(), "Finished");
    }
}

void SystemGroup::programFpga(void) {
    QString path = QFileDialog::getOpenFileName(this, "Open RPD File", "", "Raw Programming Data File (*.rpd)");
    if (path.isEmpty()) {
        return;
    }

    // RPDファイルを読み込む
    auto request = std::make_shared<phoenix_msgs::srv::ProgramFpga::Request>();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, title(), "Failed to load file");
        return;
    }
    QByteArray binary = file.readAll();
    file.close();
    if (request->bitstream.size() < (size_t)binary.size()) {
        QMessageBox::critical(this, title(), QString("Size of the RPD file must be less than %1 bytes").arg(request->bitstream.size()));
        return;
    }
    memcpy(request->bitstream.data(), binary.constData(), binary.size());
    memset(request->bitstream.data() + binary.size(), 0xFF, request->bitstream.size() - (size_t)binary.size());

    // ビットストリームを転送する
    auto result = _program_fpga_service->async_send_request(request);
    result.wait();
    auto response = result.get();
    if (!response) {
        QMessageBox::critical(this, title(), "Service didn't respond");
    }
    else if (!response->succeeded) {
        QMessageBox::critical(this, title(), "An error occured while programming bitstream");
    }
    else {
        QMessageBox::information(this, title(), "Finished");
    }
}
