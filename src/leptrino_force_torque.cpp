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

// Leptrino SDK Headers
#include <leptrino/pCommon.h>
#include <leptrino/pComResInternal.h>
#include <leptrino/rs_comm.h>

// ROS Headers
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

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
            ROS_ERROR("Failed to get sensor product info.");
            return false;
        }

        if (!getSensorLimit())
        {
            ROS_ERROR("Failed to get sensor limit.");
            return false;
        }
        return true;
    }

    /**
     * @brief センサーからのデータストリーミングを開始します
     */
    void serialStart()
    {
        ROS_INFO("Starting sensor data stream.");
        USHORT len = 0x04;
        SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_DATA_START; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    /**
     * @brief センサーからのデータストリーミングを停止します
     */
    void serialStop()
    {
        ROS_INFO("Stopping sensor data stream.");
        USHORT len = 0x04;
        SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_DATA_STOP; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    /**
     * @brief センサーからデータを1フレーム読み取ります
     * @param[out] wrench 読み取ったデータが格納されるWrenchメッセージ
     * @return データの読み取りに成功した場合は true
     */
    bool read(geometry_msgs::Wrench& wrench)
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
        ROS_INFO("Requesting sensor product information...");
        USHORT len = 0x04; SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_GET_INF; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    /**
     * @brief センサーの定格値を要求します
     */
    void getLimit()
    {
        ROS_INFO("Requesting sensor limit values...");
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
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < leptrino_constants::SENSOR_INIT_TIMEOUT_SEC)
        {
            Comm_Rcv();
            if (Comm_CheckRcv() != 0 && Comm_GetRcvData(CommRcvBuff) > 0)
            {
                ST_R_GET_INF *stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
                stGetInfo->scFVer[F_VER_SIZE] = '\0';
                ROS_INFO("  - Version: %s", stGetInfo->scFVer);
                stGetInfo->scSerial[SERIAL_SIZE] = '\0';
                ROS_INFO("  - SerialNo: %s", stGetInfo->scSerial);
                stGetInfo->scPName[P_NAME_SIZE] = '\0';
                ROS_INFO("  - Type: %s", stGetInfo->scPName);
                return true;
            }
            ros::Duration(leptrino_constants::SENSOR_INIT_POLL_DURATION_SEC).sleep();
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
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < leptrino_constants::SENSOR_INIT_TIMEOUT_SEC)
        {
            Comm_Rcv();
            if (Comm_CheckRcv() != 0 && Comm_GetRcvData(CommRcvBuff) > 0)
            {
                ST_R_LEP_GET_LIMIT* stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
                for (int i = 0; i < FN_Num; i++)
                {
                    conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
                }
                ROS_INFO("Sensor limit values received successfully.");
                return true;
            }
            ros::Duration(leptrino_constants::SENSOR_INIT_POLL_DURATION_SEC).sleep();
        }
        return false;
    }
};


/**
 * @class LeptrinoNode
 * @brief センサーからのデータ読み取りとROSトピックへの公開を管理するROSノードクラス
 */
class LeptrinoNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher wrench_pub_;
    ros::Timer publish_timer_;
    LeptrinoSensor sensor_;

    geometry_msgs::Wrench latest_wrench_;
    std::mutex wrench_mutex_;
    std::string frame_id_;
    bool new_data_available_;
    bool is_initialized_ok_;

public:
    /**
     * @brief コンストラクタ
     */
    LeptrinoNode() : nh_private_("~"), new_data_available_(false), is_initialized_ok_(false)
    {
        std::string port;
        double rate_hz;
        nh_private_.param<std::string>("com_port", port, "/dev/ttyUSB0");
        nh_private_.param<std::string>("frame_id", frame_id_, "leptrino_link");
        nh_private_.param<double>("rate", rate_hz, 100.0);

        if(rate_hz <= 0)
        {
            ROS_WARN("Parameter 'rate' must be positive. Defaulting to 1.0 Hz.");
            rate_hz = 1.0;
        }

        if (!sensor_.init(port))
        {
            ROS_FATAL("Failed to initialize sensor on port %s. Shutting down.", port.c_str());
            ros::shutdown();
            return;
        }
        if (!sensor_.initializeSensor())
        {
            ROS_FATAL("Failed to configure sensor (get info and limit). Shutting down.");
            ros::shutdown();
            return;
        }

        wrench_pub_ = nh_private_.advertise<geometry_msgs::WrenchStamped>("wrench", 10);
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz), &LeptrinoNode::timerCallback, this);

        ROS_INFO("Leptrino node started. Publishing at %.1f Hz.", rate_hz);
        is_initialized_ok_ = true;
    }

    /**
     * @brief デストラクタ
     */
    ~LeptrinoNode()
    {
        if(is_initialized_ok_)
        {
            sensor_.serialStop();
        }
    }

    /**
     * @brief センサー読み取りループを実行します
     */
    void run()
    {
        if (!is_initialized_ok_)
        {
            ROS_ERROR("Node was not initialized correctly. Aborting run().");
            return;
        }

        sensor_.serialStart();
        ros::Rate read_rate(leptrino_constants::SENSOR_READ_RATE_HZ);
        ros::Time last_read_time = ros::Time::now();

        while (ros::ok())
        {
            geometry_msgs::Wrench temp_wrench;
            if (sensor_.read(temp_wrench))
            {
                {
                    std::lock_guard<std::mutex> lock(wrench_mutex_);
                    latest_wrench_ = temp_wrench;
                    new_data_available_ = true;
                }
            }
            
            ros::Time current_time = ros::Time::now();
            double actual_period = (current_time - last_read_time).toSec();
            last_read_time = current_time;
            ROS_DEBUG("Sensor read loop period: %.6f s (%.1f Hz)", actual_period, 1.0 / actual_period);

            ros::spinOnce();
            read_rate.sleep();
        }
    }

private:
    /**
     * @brief タイマーによって定期的に呼び出され、Wrenchデータを公開するコールバック関数
     */
    void timerCallback(const ros::TimerEvent& event)
    {
        // ========== [修正点 2] ROS 1 の正しいAPIを使用して周期を計算 ==========
        ros::Duration expected_duration = event.current_expected - event.last_expected;
        ros::Duration actual_duration = event.current_real - event.last_real;
        ROS_DEBUG("Publish callback period: Expected: %.6f s, Actual: %.6f s",
                  expected_duration.toSec(), actual_duration.toSec());

        if (!new_data_available_)
        {
            ROS_WARN_THROTTLE(1.0, "No new data is available from the sensor to publish.");
            return;
        }

        geometry_msgs::WrenchStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;

        {
            std::lock_guard<std::mutex> lock(wrench_mutex_);
            msg.wrench = latest_wrench_;
        }

        wrench_pub_.publish(msg);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "leptrino_force_torque_node");
    LeptrinoNode node;
    node.run();
    return 0;
}