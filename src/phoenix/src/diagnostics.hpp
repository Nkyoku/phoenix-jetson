#pragma once

#include <stream_data.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace phoenix {

/**
 * @brief デフォルトのハードウェアIDを設定する
 * @param hardware_id ハードウェアID
 */
void setDefaultHardwareId(const std::string &hardware_id);

/**
 * @brief FPGAの診断ステータスを作成する。
 * 最後に受信したステータスを元に診断ステータスを作成する。
 * @param diag 診断ステータス
 */
void createFpgaDiagnostics(const StreamDataStatus &status, diagnostic_msgs::msg::DiagnosticStatus &diag);

/**
 * @brief ホスト名を取得する
 * @return std::string ホスト名
 */
std::string getHostName(void);

/**
 * @brief ROS2のノード名や名前空間名として使用可能な文字列のみを含むホスト名を取得する
 * @return std::string ホスト名
 */
std::string getRegularHostName(void);

}
