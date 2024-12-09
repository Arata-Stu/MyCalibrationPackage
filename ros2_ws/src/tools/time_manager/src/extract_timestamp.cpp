#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/header.hpp>
#include <fstream>
#include <iostream>
#include <optional>

class BagReaderNode : public rclcpp::Node
{
public:
    BagReaderNode() : Node("bag_reader_node")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("bag_file", "");
        this->declare_parameter<std::string>("topic_name", "");
        this->declare_parameter<std::string>("output_file", "");

        // パラメータの取得
        bag_file_ = this->get_parameter("bag_file").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        output_file_ = this->get_parameter("output_file").as_string();

        if (bag_file_.empty() || topic_name_.empty() || output_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "bag_file, topic_name, または output_file パラメータが設定されていません。");
            rclcpp::shutdown();
            return;
        }

        // Bagファイルを読み取る
        process_bag_file();
        // ノードの終了
        rclcpp::shutdown();  // Bagファイル処理後にシャットダウン
    }

private:
    std::string bag_file_;
    std::string topic_name_;
    std::string output_file_;

    void process_bag_file()
    {
        auto reader = std::make_shared<rosbag2_cpp::Reader>();
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_;
        storage_options.storage_id = "sqlite3";

        try {
            reader->open(storage_options);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Bagファイルの読み込みに失敗しました: %s", e.what());
            return;
        }

        // 出力ファイルを開く
        std::ofstream output_file(output_file_, std::ios::out);
        if (!output_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "出力ファイルのオープンに失敗しました: %s", output_file_.c_str());
            return;
        }

        // オフセットの初期化
        std::optional<long long> time_offset_microseconds;

        // トピックのデータを取得
        while (reader->has_next()) {
            auto bag_message = reader->read_next();

            if (bag_message->topic_name == topic_name_) {
                // ヘッダーのタイムスタンプを取得
                auto serialized_data = bag_message->serialized_data;
                rclcpp::SerializedMessage serialized_msg(*serialized_data);

                auto header = std::make_shared<std_msgs::msg::Header>();
                rclcpp::Serialization<std_msgs::msg::Header> serialization;
                serialization.deserialize_message(&serialized_msg, header.get());

                auto sec = header->stamp.sec;
                auto nanosec = header->stamp.nanosec;

                // マイクロ秒に変換
                long long timestamp_microseconds = static_cast<long long>(sec) * 1'000'000 + nanosec / 1'000;

                // オフセットを初期化
                if (!time_offset_microseconds.has_value()) {
                    time_offset_microseconds = timestamp_microseconds;
                }

                // オフセットを引いた相対時間を計算
                long long relative_time_microseconds = timestamp_microseconds - time_offset_microseconds.value();

                // 結果をファイルに書き出し
                output_file << relative_time_microseconds << std::endl;

                // ログに表示
                RCLCPP_INFO(this->get_logger(), "Relative Timestamp (microseconds): %lld", relative_time_microseconds);
            }
        }

        output_file.close();
        RCLCPP_INFO(this->get_logger(), "すべてのタイムスタンプを %s に書き出しました。", output_file_.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BagReaderNode>();

    // ノードの処理（Bagファイルの処理が終われば自動的に終了）
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
    }

    return 0;
}
