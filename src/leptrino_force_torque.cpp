/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 * Notice: Modified & copied from Leptrino CD example source code
 * Refactored and improved by YusakuNakajima on 2025-06-20 for better performance
 */

#include <mutex>
#include <string>
#include <chrono>
#include <thread>

// Leptrino SDK Headers
#include <leptrino/pCommon.h>
#include <leptrino/pComResInternal.h>
#include <leptrino/rs_comm.h>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace leptrino_constants {
const double SENSOR_INIT_TIMEOUT_SEC = 2.0;
const double SENSOR_INIT_POLL_DURATION_SEC = 0.01;
const int SENSOR_READ_RATE_HZ = 1200;
} // namespace leptrino_constants

// ========== [修正点 1] 削除されていた SystemInfo の型定義を再追加 ==========
/**
 * @struct ST_SystemInfo
 * @brief Leptrinoライブラリが使用するシステム情報構造体
 */
typedef struct ST_SystemInfo
{
  int com_ok;
} SystemInfo;


/**
 * @class LeptrinoSensor
 * @brief Leptrino 6軸力覚センサーとのシリアル通信を管理するクラス
 */
class LeptrinoSensor
{
private:
    SystemInfo gSys;
    UCHAR CommRcvBuff[256];
    UCHAR CommSendBuff[1024];
    UCHAR SendBuff[512];
    double conversion_factor[FN_Num];
    std::string g_com_port;
    bool is_initialized_;

public:
    /**
     * @brief コンストラクタ
     */
    LeptrinoSensor() : is_initialized_(false)
    {
        memset(conversion_factor, 0, sizeof(conversion_factor));
        gSys.com_ok = NG;
    }

    /**
     * @brief デストラクタ. 通信ポートを確実にクローズします
     */
    ~LeptrinoSensor()
    {
        close();
    }

    /**
     * @brief シリアルポートを開き、通信を準備します
     * @param port シリアルポートのパス (例: "/dev/ttyUSB0")
     * @return 初期化が成功した場合は true
     */
    bool init(const std::string& port)
    {
        g_com_port = port;
        if (Comm_Open(g_com_port.c_str()) != OK)
        {
            return false;
        }
        Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
        gSys.com_ok = OK;
        is_initialized_ = true;
        return true;
    }

    /**
     * @brief センサーの製品情報と定格値を取得し、センサーを初期化します
     * @return 初期化が成功した場合は true
     */
    bool initializeSensor()
    {
        if (!getProductInformation())
        {
            RCLCPP_ERROR(rclcpp::get_logger("leptrino_sensor"), "Failed to get sensor product info.");
            return false;
        }

        if (!getSensorLimit())
        {
            RCLCPP_ERROR(rclcpp::get_logger("leptrino_sensor"), "Failed to get sensor limit.");
            return false;
        }
        return true;
    }

    /**
     * @brief センサーからのデータストリーミングを開始します
     */
    void serialStart()
    {
        RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "Starting sensor data stream.");
        USHORT len = 0x04;
        SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_DATA_START; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    /**
     * @brief センサーからのデータストリーミングを停止します
     */
    void serialStop()
    {
        RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "Stopping sensor data stream.");
        USHORT len = 0x04;
        SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_DATA_STOP; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    /**
     * @brief センサーからデータを1フレーム読み取ります
     * @param[out] wrench 読み取ったデータが格納されるWrenchメッセージ
     * @return データの読み取りに成功した場合は true
     */
    bool read(geometry_msgs::msg::Wrench& wrench)
    {
        Comm_Rcv();
        if (Comm_CheckRcv() != 0)
        {
            memset(CommRcvBuff, 0, sizeof(CommRcvBuff));
            if (Comm_GetRcvData(CommRcvBuff) > 0)
            {
                ST_R_DATA_GET_F *stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
                wrench.force.x = stForce->ssForce[0] * conversion_factor[0];
                wrench.force.y = stForce->ssForce[1] * conversion_factor[1];
                wrench.force.z = stForce->ssForce[2] * conversion_factor[2];
                wrench.torque.x = stForce->ssForce[3] * conversion_factor[3];
                wrench.torque.y = stForce->ssForce[4] * conversion_factor[4];
                wrench.torque.z = stForce->ssForce[5] * conversion_factor[5];
                return true;
            }
        }
        return false;
    }

private:
    /**
     * @brief シリアルポートをクローズします
     */
    void close()
    {
        if (is_initialized_ && gSys.com_ok == OK)
        {
            Comm_Close();
            gSys.com_ok = NG;
            is_initialized_ = false;
        }
    }

    /**
     * @brief データをDLE-STX形式で送信します
     */
    void sendData(UCHAR *pucInput, USHORT usSize)
    {
        USHORT usCnt; UCHAR ucWork; UCHAR ucBCC = 0; UCHAR *pucWrite = &CommSendBuff[0]; USHORT usRealSize;
        *pucWrite = CHR_DLE; pucWrite++; *pucWrite = CHR_STX; pucWrite++; usRealSize = 2;
        for (usCnt = 0; usCnt < usSize; usCnt++){
            ucWork = pucInput[usCnt];
            if (ucWork == CHR_DLE){ *pucWrite = CHR_DLE; pucWrite++; usRealSize++; }
            *pucWrite = ucWork; ucBCC ^= ucWork; pucWrite++; usRealSize++;
        }
        *pucWrite = CHR_DLE; pucWrite++; *pucWrite = CHR_ETX; ucBCC ^= CHR_ETX; pucWrite++; *pucWrite = ucBCC; usRealSize += 3;
        Comm_SendData(&CommSendBuff[0], usRealSize);
    }

    /**
     * @brief センサーの製品情報を要求します
     */
    void getProductInfo()
    {
        RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "Requesting sensor product information...");
        USHORT len = 0x04; SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_GET_INF; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    /**
     * @brief センサーの定格値を要求します
     */
    void getLimit()
    {
        RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "Requesting sensor limit values...");
        USHORT len = 0x04; SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_GET_LIMIT; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    /**
     * @brief センサーの製品情報を取得します
     * @return 成功した場合は true
     */
    bool getProductInformation()
    {
        getProductInfo();
        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok())
        {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(current_time - start_time).count();
            if (elapsed >= leptrino_constants::SENSOR_INIT_TIMEOUT_SEC)
                break;

            Comm_Rcv();
            if (Comm_CheckRcv() != 0 && Comm_GetRcvData(CommRcvBuff) > 0)
            {
                ST_R_GET_INF *stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
                stGetInfo->scFVer[F_VER_SIZE-1] = '\0';
                RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "  - Version: %s", stGetInfo->scFVer);
                stGetInfo->scSerial[SERIAL_SIZE-1] = '\0';
                RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "  - SerialNo: %s", stGetInfo->scSerial);
                stGetInfo->scPName[P_NAME_SIZE-1] = '\0';
                RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "  - Type: %s", stGetInfo->scPName);
                return true;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(leptrino_constants::SENSOR_INIT_POLL_DURATION_SEC));
        }
        return false;
    }

    /**
     * @brief センサーの定格値を取得し、変換係数を計算します
     * @return 成功した場合は true
     */
    bool getSensorLimit()
    {
        getLimit();
        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok())
        {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(current_time - start_time).count();
            if (elapsed >= leptrino_constants::SENSOR_INIT_TIMEOUT_SEC)
                break;

            Comm_Rcv();
            if (Comm_CheckRcv() != 0 && Comm_GetRcvData(CommRcvBuff) > 0)
            {
                ST_R_LEP_GET_LIMIT* stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
                for (int i = 0; i < FN_Num; i++)
                {
                    conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
                }
                RCLCPP_INFO(rclcpp::get_logger("leptrino_sensor"), "Sensor limit values received successfully.");
                return true;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(leptrino_constants::SENSOR_INIT_POLL_DURATION_SEC));
        }
        return false;
    }
};


/**
 * @class LeptrinoNode
 * @brief センサーからのデータ読み取りとROSトピックへの公開を管理するROS2ノードクラス
 */
class LeptrinoNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    LeptrinoSensor sensor_;

    geometry_msgs::msg::Wrench latest_wrench_;
    std::mutex wrench_mutex_;
    std::string frame_id_;
    bool new_data_available_;
    bool is_initialized_ok_;
    
    // スレッド関連
    std::thread sensor_thread_;
    std::atomic<bool> shutdown_requested_;

public:
    /**
     * @brief コンストラクタ
     */
    LeptrinoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("leptrino_force_torque_node", options), 
          new_data_available_(false), 
          is_initialized_ok_(false),
          shutdown_requested_(false)
    {
        // Declare parameters
        this->declare_parameter("com_port", "/dev/ttyUSB0");
        this->declare_parameter("frame_id", "leptrino_link");
        this->declare_parameter("rate", 100.0);

        // Get parameters
        std::string port = this->get_parameter("com_port").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("rate").as_double();

        if(rate_hz <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "Parameter 'rate' must be positive. Defaulting to 1.0 Hz.");
            rate_hz = 1.0;
        }

        if (!sensor_.init(port))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize sensor on port %s. Shutting down.", port.c_str());
            rclcpp::shutdown();
            return;
        }
        if (!sensor_.initializeSensor())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to configure sensor (get info and limit). Shutting down.");
            rclcpp::shutdown();
            return;
        }

        wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 10);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&LeptrinoNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Leptrino node started. Publishing at %.1f Hz.", rate_hz);
        is_initialized_ok_ = true;
        
        // センサースレッドを開始
        startSensorThread();
    }

    /**
     * @brief デストラクタ
     */
    ~LeptrinoNode()
    {
        shutdown_requested_ = true;
        
        if(is_initialized_ok_)
        {
            sensor_.serialStop();
        }
        
        if (sensor_thread_.joinable())
        {
            sensor_thread_.join();
        }
    }

    /**
     * @brief センサーデータ読み取りスレッドを開始します
     */
    void startSensorThread()
    {
        if (!is_initialized_ok_)
        {
            RCLCPP_ERROR(this->get_logger(), "Node was not initialized correctly. Aborting sensor thread start.");
            return;
        }

        sensor_.serialStart();
        
        // センサー読み取り用のスレッドを開始
        sensor_thread_ = std::thread([this]() {
            rclcpp::Rate read_rate(leptrino_constants::SENSOR_READ_RATE_HZ);
            auto last_read_time = std::chrono::steady_clock::now();

            while (rclcpp::ok() && !shutdown_requested_)
            {
                geometry_msgs::msg::Wrench temp_wrench;
                if (sensor_.read(temp_wrench))
                {
                    {
                        std::lock_guard<std::mutex> lock(wrench_mutex_);
                        latest_wrench_ = temp_wrench;
                        new_data_available_ = true;
                    }
                }
                
                auto current_time = std::chrono::steady_clock::now();
                double actual_period = std::chrono::duration<double>(current_time - last_read_time).count();
                last_read_time = current_time;
                RCLCPP_DEBUG(this->get_logger(), "Sensor read loop period: %.6f s (%.1f Hz)", actual_period, 1.0 / actual_period);

                read_rate.sleep();
            }
        });
    }

private:
    /**
     * @brief タイマーによって定期的に呼び出され、Wrenchデータを公開するコールバック関数
     */
    void timerCallback()
    {
        if (!new_data_available_)
        {
            static rclcpp::Time last_warn_time = this->now();
            if ((this->now() - last_warn_time).seconds() >= 1.0)
            {
                RCLCPP_WARN(this->get_logger(), "No new data is available from the sensor to publish.");
                last_warn_time = this->now();
            }
            return;
        }

        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;

        {
            std::lock_guard<std::mutex> lock(wrench_mutex_);
            msg.wrench = latest_wrench_;
        }

        wrench_pub_->publish(msg);
    }
};


// ROS2コンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(LeptrinoNode)

// standaloneノードとしての実行も可能にする
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LeptrinoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}