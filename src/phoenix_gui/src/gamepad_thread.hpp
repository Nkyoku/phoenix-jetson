#pragma once

#include <QtCore/QThread>
#include <array>
#include <atomic>
#include <memory>
#include <stdint.h>

class GamepadThread : public QThread {
    Q_OBJECT

public:
    struct InputState {
        uint16_t buttons;
        float left_trigger;
        float right_trigger;
        float left_stick_x;
        float left_stick_y;
        float right_stick_x;
        float right_stick_y;
    };

    GamepadThread(QObject *parent = nullptr);

    virtual ~GamepadThread();

    static bool isSupported(void);

    std::shared_ptr<InputState> inputState(int device_id);

    Q_SLOT void vibrate(int device_id, float power);

    Q_SIGNAL void gamepadConnected(int device_id);

    Q_SIGNAL void gamepadDisconnected(int device_id);

private:
    void run(void) override;

    static void applyTriggerDeadZone(float &value, float dead_zone);

    static void applyStickDeadZone(float &x_value, float &y_value, float dead_zone);

    static constexpr int MAX_DEVICE_COUNT = 4;

    std::array<bool, MAX_DEVICE_COUNT> _is_device_connected = {};

    std::array<uint32_t, MAX_DEVICE_COUNT> _last_packet_numbers = {};

    std::array<std::shared_ptr<InputState>, MAX_DEVICE_COUNT> _input_states;

    std::array<std::atomic<int>, MAX_DEVICE_COUNT> _vibration_powers = {};
};
