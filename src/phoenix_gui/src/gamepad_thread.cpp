#include "gamepad_thread.hpp"

#ifdef _MSC_VER
#define ENABLE_GAMEPAD_SUPPORT 1
#else
#define ENABLE_GAMEPAD_SUPPORT 0
#endif

#if ENABLE_GAMEPAD_SUPPORT
#include <Windows.h>
#include <Xinput.h>
#include <string.h>
#undef min
#undef max
#include <algorithm>
#pragma comment(lib, "xinput.lib")
#endif

GamepadThread::GamepadThread(QObject *parent) : QThread(parent) {}

GamepadThread::~GamepadThread() {}

bool GamepadThread::isSupported(void) {
    return ENABLE_GAMEPAD_SUPPORT;
}

std::shared_ptr<GamepadThread::InputState> GamepadThread::inputState(int device_id) {
    if ((0 <= device_id) && (device_id <= MAX_DEVICE_COUNT)) {
        return _input_states[device_id];
    }
    return std::shared_ptr<GamepadThread::InputState>();
}

void GamepadThread::vibrate(int device_id, float power) {
    if ((0 <= device_id) && (device_id <= MAX_DEVICE_COUNT)) {
        _vibration_powers[device_id] = static_cast<int>(std::min(std::max(0.0f, power), 1.0f) * 65535);
    }
}

void GamepadThread::run(void) {
#if ENABLE_GAMEPAD_SUPPORT
    while (!isInterruptionRequested()) {
        for (int index = 0; index < MAX_DEVICE_COUNT; index++) {
            XINPUT_STATE state;
            memset(&state, 0, sizeof(state));
            if (XInputGetState(index, &state) == ERROR_SUCCESS) {
                if (!_is_device_connected[index] || (_last_packet_numbers[index] != state.dwPacketNumber)) {
                    _last_packet_numbers[index] = state.dwPacketNumber;
                    if (_is_device_connected[index] == false) {
                        _vibration_powers[index] = -1;
                    }
                    // 入力を読み取って格納する
                    auto input_state = std::make_shared<InputState>();
                    input_state->buttons = state.Gamepad.wButtons;
                    input_state->left_trigger = state.Gamepad.bLeftTrigger / 255.0f;
                    input_state->right_trigger = state.Gamepad.bRightTrigger / 255.0f;
                    input_state->left_stick_x = state.Gamepad.sThumbLX / 32768.0f;
                    input_state->left_stick_y = state.Gamepad.sThumbLY / 32768.0f;
                    input_state->right_stick_x = state.Gamepad.sThumbRX / 32768.0f;
                    input_state->right_stick_y = state.Gamepad.sThumbRY / 32768.0f;
                    applyTriggerDeadZone(input_state->left_trigger, (float)XINPUT_GAMEPAD_TRIGGER_THRESHOLD / 255.0f);
                    applyTriggerDeadZone(input_state->right_trigger, (float)XINPUT_GAMEPAD_TRIGGER_THRESHOLD / 255.0f);
                    applyStickDeadZone(input_state->left_stick_x, input_state->left_stick_y, (float)XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE / 32768.0f);
                    applyStickDeadZone(input_state->right_stick_x, input_state->right_stick_y, (float)XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE / 32768.0f);
                    _input_states[index] = input_state;
                }
                if (_is_device_connected[index] == false) {
                    _is_device_connected[index] = true;
                    emit gamepadConnected(index);
                }

                // バイブレーションの値が変更されていれば反映する
                XINPUT_VIBRATION vibration_state;
                int vibration_power = _vibration_powers[index].exchange(-1);
                if (0 <= vibration_power) {
                    vibration_state.wLeftMotorSpeed = vibration_power;
                    vibration_state.wRightMotorSpeed = vibration_power;
                    XInputSetState(index, &vibration_state);
                }
            }
            else {
                if (_is_device_connected[index] == true) {
                    _input_states[index].reset();
                    _is_device_connected[index] = false;
                    emit gamepadDisconnected(index);
                }
            }
        }
        msleep(10);
    }
#endif
}

void GamepadThread::applyTriggerDeadZone(float &value, float dead_zone) {
    if (value < dead_zone) {
        value = 0.0f;
    }
    else {
        value = (value - dead_zone) / (1.0f - dead_zone);
    }
}

void GamepadThread::applyStickDeadZone(float &x_value, float &y_value, float dead_zone) {
    float mag = sqrtf(x_value * x_value + y_value * y_value);
    if (mag < dead_zone) {
        x_value = 0.0f;
        y_value = 0.0f;
    }
    else {
        float norm_mag = (std::min(mag, 1.0f) - dead_zone) / (1.0f - dead_zone);
        x_value = x_value * norm_mag / mag;
        y_value = y_value * norm_mag / mag;
    }
}
