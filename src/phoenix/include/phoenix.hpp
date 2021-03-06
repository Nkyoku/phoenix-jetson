#pragma once

namespace phoenix {



/// 名前空間名
static constexpr char NAMESPACE_NAME[] = "phoenix";

/// 速度指令値を購読するトピック名
static constexpr char TOPIC_NAME_COMMAND_VELOCITY[] = "cmd_vel";

/// バッテリ状態を配信するトピック名
static constexpr char TOPIC_NAME_BATTERY[] = "battery_state";

/// IMU測定値を配信するトピック名
static constexpr char TOPIC_NAME_IMU[] = "imu";

/// オドメトリを配信するトピック名
static constexpr char TOPIC_NAME_ODOMETRY[] = "odom";

/// ADC2の測定値を配信するトピック名
static constexpr char TOPIC_NAME_ADC2[] = "adc2";

/// モーションに関する情報を配信するトピック名
static constexpr char TOPIC_NAME_MOTION[] = "motion";

/// Nios IIのプログラムを書き換えるサービス名
static constexpr char SERVICE_NAME_PROGRAM_NIOS[] = "program_nios";

/// FPGAを書き換えるサービス名
static constexpr char SERVICE_NAME_PROGRAM_FPGA[] = "program_fpga";

/// FPGAの診断ステータスを配信するときにコンポーネント名の後ろに付ける名前
static constexpr char DIAGNOSTICS_NAME_SUFFIX_FPGA[] = "-FPGA";

/// オドメトリ座標系のフレームID
static constexpr char FRAME_ID_ODOMETRY[] = "/odom";

/// ベース座標系のフレームID
static constexpr char FRAME_ID_BASE[] = "/base_link";

/// IMU座標系のフレームID
static constexpr char FRAME_ID_IMU[] = "/imu_link";



namespace test {

/// テストのためにエラーフラグに故障注入するトピック名
static constexpr char TOPIC_NAME_INJECTED_ERROR_FLAGS[] = "injected_error_flags";

/// テストのためにフォルトフラグに故障注入するトピック名
static constexpr char TOPIC_NAME_INJECTED_FAULT_FLAGS[] = "injected_fault_flags";

}



namespace battery {

/// ノード名
static constexpr char NODE_NAME[] = "battery";

/// I2Cのデバイスパスのパラメータ名
static constexpr char PARAM_NAME_DEVICE_PATH[] = "device_path";

/// デバイスのI2Cアドレスを指定するパラメータ名
static constexpr char PARAM_NAME_DEVICE_ADDRESS[] = "device_address";

} // namespace battery



namespace command {

/// ノード名
static constexpr char NODE_NAME[] = "command";

/// SPIのデバイスパスのパラメータ名
static constexpr char PARAM_NAME_DEVICE_PATH[] = "device_path";

/// 速度制御の比例ゲインのパラメータ名
static constexpr char PARAM_NAME_SPEED_X_KP[] = "speed_x_kp";
static constexpr char PARAM_NAME_SPEED_Y_KP[] = "speed_y_kp";
static constexpr char PARAM_NAME_SPEED_W_KP[] = "speed_w_kp";
static constexpr char PARAM_NAME_SPEED_C_KP[] = "speed_c_kp";

/// 速度制御の積分ゲインのパラメータ名
static constexpr char PARAM_NAME_SPEED_X_KI[] = "speed_x_ki";
static constexpr char PARAM_NAME_SPEED_Y_KI[] = "speed_y_ki";
static constexpr char PARAM_NAME_SPEED_W_KI[] = "speed_w_ki";
static constexpr char PARAM_NAME_SPEED_C_KI[] = "speed_c_ki";

} // namespace command



namespace stream {

/// ノード名
static constexpr char NODE_NAME[] = "stream";

/// UARTのデバイスパスのパラメータ名
static constexpr char PARAM_NAME_DEVICE_PATH[] = "device_path";

/// オドメトリとIMUの間引き回数のパラメータ名
static constexpr char PARAM_NAME_DECIMATION[] = "decimation";

} // namespace stream



} // namespace phoenix
