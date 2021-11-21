#pragma once

#include <QtWidgets/QGroupBox>
#include <QtWidgets/QTreeWidgetItem>
#include <QtCore/QFile>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>
#include <list>

class Ui_TelemetryViewer;

class TelemetryViewer : public QGroupBox {
    Q_OBJECT

public:
    TelemetryViewer(QWidget *parent = nullptr);

    ~TelemetryViewer();

    /**
     * @brief 指定したトピックをサブスクライブしてツリーに表示する
     * @tparam T メッセージの型
     * @param node ローカルノード
     * @param topic_name サブスクライブするトピック名
     * @param qos QoSの設定
     */
    template<class T>
    void addSubscription(rclcpp::Node::SharedPtr node, const std::string &topic_name, const rclcpp::QoS &qos);

    /**
     * @brief ノードの終了処理を行う
     */
    void uninitializeNode(void);

private:
    /**
     * @brief ツリーを更新する
     */
    Q_SLOT void updateTelemertyTreeItems(void);

    /**
     * @brief ロギングを開始する
     */
    Q_SLOT void startLogging(void);

    /**
     * @brief ロギングを停止する
     */
    Q_SLOT void stopLogging(void);

    /// Qt Designerで作成したUI
    Ui_TelemetryViewer *_ui = nullptr;

    /// テレメトリのログを保存するファイル
    std::shared_ptr<QFile> _log_file;

    /// テレメトリのログに含まれるフレーム番号
    uint32_t _frame_number = 0;

    /// ツリーに表示するデータ構造を保持する構造体
    struct Field_t {
        QTreeWidgetItem *item = nullptr;
        std::vector<QTreeWidgetItem *> elements;
        std::vector<Field_t> children;
    };

    /// サブスクリプション情報を保持するクラス
    class SubscriberBase {
    public:
        Field_t top;
        virtual ~SubscriberBase() {}
        virtual void subscribe(rclcpp::Node::SharedPtr node, const std::string &topic_name, const rclcpp::QoS &qos) = 0;
        virtual const rosidl_message_type_support_t *handle(void) const = 0;
        virtual void update(void) = 0;

    protected:
        static void updateImpl(Field_t &parent_field, const void *ptr, const rosidl_message_type_support_t *type_support);
        static void updatePrimitive(QTreeWidgetItem *item, uint8_t type, const void *ptr);
    };

    /// 各メッセージのサブスクリプション情報を保持するクラス
    template<class T>
    class Subscriber : public SubscriberBase {
    public:
        std::shared_ptr<T> last_message;
        std::shared_ptr<rclcpp::Subscription<T>> subscription;

        void subscribe(rclcpp::Node::SharedPtr node, const std::string &topic_name, const rclcpp::QoS &qos) {
            subscription = node->create_subscription<T>(topic_name, qos, [this](const std::shared_ptr<T> msg) {
                std::atomic_store(&last_message, msg);
            });
        }

        const rosidl_message_type_support_t *handle(void) const {
            return rosidl_typesupport_introspection_cpp::get_message_type_support_handle<T>();
        }

        void update(void) {
            auto msg = std::atomic_load(&last_message);
            updateImpl(top, msg.get(), handle());
        }
    };

    /// addSubscription<T>()で作成したサブスクリプションのリスト
    std::list<std::shared_ptr<SubscriberBase>> _subscribers;

    /**
     * @brief addSubscription<T>()の共通処理
     * @param topic_name トピック名
     * @param subscriber サブスクライバ
     */
    void addSubscriptionImpl(const std::string &topic_name, std::shared_ptr<SubscriberBase> subscriber);

    /**
     * @brief typesupport情報に従ってツリー項目を作成する
     * @param parent_field ツリー上で親となるフィールド
     * @param base_name 項目名の先頭につける文字列
     * @param type_support メッセージのtypesupport情報
     */
    void createTreeItems(Field_t &parent_field, const QString &base_name, const rosidl_message_type_support_t *type_support);

    /// telemetryTreeでフィールド名を表示する列
    static constexpr int FIELD_COLUMN = 0;

    /// telemetryTreeで値を表示する列
    static constexpr int VALUE_COLUMN = 1;

    /// 配列を表示する最大数
    static constexpr size_t MAX_ARRAY_ELEMENTS = 16;
};

template<class T>
inline void TelemetryViewer::addSubscription(rclcpp::Node::SharedPtr node, const std::string &topic_name, const rclcpp::QoS &qos) {
    auto subscriber = std::make_shared<Subscriber<T>>();
    subscriber->subscribe(node, topic_name, qos);
    addSubscriptionImpl(topic_name, subscriber);
}
