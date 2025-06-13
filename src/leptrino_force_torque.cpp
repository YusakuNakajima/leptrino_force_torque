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
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <mutex>

#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>
#include <leptrino/pComResInternal.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

typedef struct ST_SystemInfo
{
  int com_ok;
} SystemInfo;


// Leptrinoセンサーとの通信を行うクラス
class LeptrinoSensor
{
private:
    SystemInfo gSys;
    UCHAR CommRcvBuff[256];
    UCHAR CommSendBuff[1024];
    UCHAR SendBuff[512];
    double conversion_factor[FN_Num];
    std::string g_com_port;

public:
    LeptrinoSensor() {
        memset(conversion_factor, 0, sizeof(conversion_factor));
    }

    bool init(const std::string& port)
    {
        g_com_port = port;
        gSys.com_ok = NG;
        int rt = Comm_Open(g_com_port.c_str());
        if (rt == OK)
        {
            Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
            gSys.com_ok = OK;
        }
        return gSys.com_ok == OK;
    }

    void close()
    {
        if (gSys.com_ok == OK)
        {
            Comm_Close();
        }
    }

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

    void getProductInfo()
    {
        ROS_INFO("Get sensor information");
        USHORT len = 0x04; SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_GET_INF; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    void getLimit()
    {
        ROS_INFO("Get sensor limit");
        USHORT len = 0x04; SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_GET_LIMIT; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    void serialStart()
    {
        ROS_INFO("Start sensor");
        USHORT len = 0x04; SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_DATA_START; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    void serialStop()
    {
        printf("Stop sensor\n");
        USHORT len = 0x04; SendBuff[0] = len; SendBuff[1] = 0xFF; SendBuff[2] = CMD_DATA_STOP; SendBuff[3] = 0;
        sendData(SendBuff, len);
    }

    // データ受信と処理を行う関数
    bool read(geometry_msgs::Wrench& wrench)
    {
        Comm_Rcv();
        if (Comm_CheckRcv() != 0)
        {
            memset(CommRcvBuff, 0, sizeof(CommRcvBuff));
            int rt = Comm_GetRcvData(CommRcvBuff);
            if (rt > 0)
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

    bool initializeSensor() {
        getProductInfo();
        ros::Time start_time = ros::Time::now();
        while(ros::ok() && (ros::Time::now() - start_time).toSec() < 2.0) {
            Comm_Rcv();
            if(Comm_CheckRcv()!=0) {
                int rt = Comm_GetRcvData(CommRcvBuff);
                if(rt > 0) {
                    ST_R_GET_INF *stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
                    stGetInfo->scFVer[F_VER_SIZE] = 0;
                    ROS_INFO("Version: %s", stGetInfo->scFVer);
                    stGetInfo->scSerial[SERIAL_SIZE] = 0;
                    ROS_INFO("SerialNo: %s", stGetInfo->scSerial);
                    stGetInfo->scPName[P_NAME_SIZE] = 0;
                    ROS_INFO("Type: %s", stGetInfo->scPName);
                    goto GET_LIMIT;
                }
            }
            ros::Duration(0.01).sleep();
        }
        ROS_ERROR("Failed to get sensor product info.");
        return false;

    GET_LIMIT:
        getLimit();
        start_time = ros::Time::now();
        while(ros::ok() && (ros::Time::now() - start_time).toSec() < 2.0) {
            Comm_Rcv();
            if(Comm_CheckRcv()!=0) {
                int rt = Comm_GetRcvData(CommRcvBuff);
                if(rt > 0) {
                    ST_R_LEP_GET_LIMIT* stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
                    for(int i = 0; i < FN_Num; i++) {
                        conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
                    }
                    ROS_INFO("Sensor limit values received successfully.");
                    return true;
                }
            }
            ros::Duration(0.01).sleep();
        }
        ROS_ERROR("Failed to get sensor limit.");
        return false;
    }
};


// ROSノード全体を管理するクラス
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

public:
    LeptrinoNode() : nh_private_("~"), new_data_available_(false)
    {
        std::string port;
        int rate;
        nh_private_.param<std::string>("com_port", port, "/dev/ttyUSB0");
        nh_private_.param<std::string>("frame_id", frame_id_, "leptrino");
        nh_private_.param<int>("rate", rate, 100);

        if (!sensor_.init(port)) {
            ROS_FATAL("Failed to initialize sensor on port %s. Shutting down.", port.c_str());
            ros::shutdown();
            return;
        }
        if (!sensor_.initializeSensor()) {
            ROS_FATAL("Failed to get sensor info and limit. Shutting down.");
            ros::shutdown();
            return;
        }

        wrench_pub_ = nh_private_.advertise<geometry_msgs::WrenchStamped>("wrench", 10);
        
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &LeptrinoNode::timerCallback, this);

        ROS_INFO("Leptrino node started. Publishing at %d Hz.", rate);
    }

    ~LeptrinoNode() {
        sensor_.serialStop();
        sensor_.close();
    }

    void timerCallback(const ros::TimerEvent& event)
    {
        if(!new_data_available_) {
            ROS_WARN_THROTTLE(1.0, "Timer callback triggered, but no new data is available from the sensor.");
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

    void run()
    {
        if(ros::isShuttingDown()) return; // Abort if initialization failed

        sensor_.serialStart();
        ros::Rate read_rate(1200); 

        while(ros::ok())
        {
            geometry_msgs::Wrench temp_wrench;
            if(sensor_.read(temp_wrench))
            {
                {
                    std::lock_guard<std::mutex> lock(wrench_mutex_);
                    latest_wrench_ = temp_wrench;
                    new_data_available_ = true;
                }
            }
            ros::spinOnce();
            read_rate.sleep();
        }
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "leptrino_force_torque");
  LeptrinoNode node;
  node.run();
  return 0;
}