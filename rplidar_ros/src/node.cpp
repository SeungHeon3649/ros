/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ros/ros.h" //ros 라이브러리
#include "sensor_msgs/LaserScan.h" //lidar 라이브러리
#include "std_srvs/Empty.h" //서비스 라이브러리
#include "rplidar.h" //rplidar sdk

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0])) //배열크기 정의
#endif

#define DEG2RAD(x) ((x)*M_PI/180.) //라디안변환 정의

using namespace rp::standalone::rplidar; //네임스페이스 정의

RPlidarDriver * drv = NULL; //전역변수 선언

void publish_scan(ros::Publisher *pub,
                  rplidar_response_measurement_node_t *nodes,
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  std::string frame_id)
{ //scan publish 정의
    static int scan_count = 0; //스캔횟수 선언
    sensor_msgs::LaserScan scan_msg; //스캔메시지 선언

    scan_msg.header.stamp = start; //시작시간 선언
    scan_msg.header.frame_id = frame_id; //프레임아이디 선언
    scan_count++; //

    bool reversed = (angle_max > angle_min); // 최대값 최소값 바뀌었을때
    if ( reversed ) { // 반대로 선언
      scan_msg.angle_min =  M_PI - angle_max;
      scan_msg.angle_max =  M_PI - angle_min;
    } else {
      scan_msg.angle_min =  M_PI - angle_min;
      scan_msg.angle_max =  M_PI - angle_max;
    }
    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);
 //각도증가량 (1도)
    scan_msg.scan_time = scan_time; //한 바퀴 돌리는 데 걸린 시간
    scan_msg.time_increment = scan_time / (double)(node_count-1); //1도 스캔하는데 걸리는 시간
    scan_msg.range_min = 0.15; //최소 범위
    scan_msg.range_max = 8.0; //최대 범위

    scan_msg.intensities.resize(node_count); //강도 정규화(?)
    scan_msg.ranges.resize(node_count); //범위 정규화(?)
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed); //라이다가 뒤집, 최대 최소 반대
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) { //1도마다
            float read_value = (float) nodes[i].distance_q2/4.0f/1000; //값 측정
            if (read_value == 0.0) //값이 0이면
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity(); //무한대입력
            else
                scan_msg.ranges[i] = read_value; //값 스캔메시지에 입력
            scan_msg.intensities[i] = (float) (nodes[i].sync_quality >> 2); //(??)
        }
    } else { //뒤집히거나 최대최소 반대일때
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].distance_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[node_count-1-i] = read_value;
            scan_msg.intensities[node_count-1-i] = (float) (nodes[i].sync_quality >> 2);
        }
    }

    pub->publish(scan_msg); //퍼블리쉬
}

bool getRPLIDARDeviceInfo(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            fprintf(stderr, "Error, operation time out.\n");
        } else {
            fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
           , devinfo.firmware_version>>8
           , devinfo.firmware_version & 0xFF
           , (int)devinfo.hardware_version);
    return true;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { 
        printf("RPLidar health status : %d\n", healthinfo.status);
        
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected."
                            "Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve rplidar health code: %x\n", 
                        op_result);
        return false;
    }
}

bool stop_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res) //모터 정지 서비스
{
  if(!drv) //드라이버 생성 x
       return false;

  ROS_DEBUG("Stop motor"); 
  drv->stop();
  drv->stopMotor();
  return true;
}

bool start_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;
  ROS_DEBUG("Start motor");
  drv->startMotor();
  drv->startScan();;
  return true;
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "rplidar_node"); //노드명 초기화

    std::string serial_port; //시리얼 포트
    int serial_baudrate = 115200; //보드레이트
    std::string frame_id; //프레임 아이디
    bool inverted = false; //반대인지 아닌지
    bool angle_compensate = true; //각도 보상

    ros::NodeHandle nh; //노드 핸들
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000); //퍼블리셔 선언
    ros::NodeHandle nh_private("~"); //
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); //파라미터 값 저장
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);  
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, false);
    nh_private.param<bool>("angle_compensate", angle_compensate, true);

    printf("RPLIDAR running on ROS package rplidar_ros\n"
           "SDK Version: "RPLIDAR_SDK_VERSION"\n");

    u_result     op_result;

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT); //드라이버 생성
    
    if (!drv) { //생성 안되었을 시 오류
        fprintf(stderr, "Create Driver fail, exit\n");
        return -2;
    }

    // make connection...
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) { //드라이버 연결 안됐을 시 오류
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , serial_port.c_str());
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    // get rplidar device info
    if (!getRPLIDARDeviceInfo(drv)) { //드라이버 정보 얻지 못하면 못했을 시
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) { //라이다 상태 확인
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor); //모터 정지 서비스
    ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor); //모터 시작 서비스

    drv->startMotor(); //모터시작
    drv->startScan(); //스캔 시작

    ros::Time start_scan_time; //시간 측정
    ros::Time end_scan_time;
    double scan_duration; //걸린 시간
    while (ros::ok()) { //반복

        rplidar_response_measurement_node_t nodes[360*2]; //측정값 저장 변수
        size_t   count = _countof(nodes); //배열 크기 저장

        start_scan_time = ros::Time::now();
        op_result = drv->grabScanData(nodes, count); //데이터 저장 후 결과 확인
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3; //걸린 시간 측정

        if (op_result == RESULT_OK) { //결과 완료
            op_result = drv->ascendScanData(nodes, count); //(???)

            float angle_min = DEG2RAD(0.0f); //최소각
            float angle_max = DEG2RAD(359.0f); //최대각
            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    const int angle_compensate_nodes_count = 360;
                    const int angle_compensate_multiple = 1;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_t));
                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].distance_q2 != 0) {
                            float angle = (float)((nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {
                                angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                            }
                        }
                    }
  
                    publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max,
                             frame_id);
                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].distance_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].distance_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

                    publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max,
                             frame_id);
               }
            } else if (op_result == RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);

                publish_scan(&scan_pub, nodes, count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max,
                             frame_id);
            }
        }

        ros::spinOnce();
    }

    // done!
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}
/*
왜 배열 크기를 360*2를 하였는가
ascendScanData ?
scan_msg.intensities[i] = (float) (nodes[i].sync_quality >> 2); 쉬프트의 의미
*/