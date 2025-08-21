/**
 * @file vlp16.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <atomic>
#include <functional>
#include <chrono>
#include <mutex>

struct VLP16Packet {
    // 원시 UDP 페이로드(보통 1206 bytes)
    std::vector<uint8_t> data;
    // 수신 시각(모노토닉)
    std::chrono::steady_clock::time_point recv_time;
};

struct VLP16Params {
    std::string device_ip = "";        // 특정 라이다 IP로 필터링할 경우(옵션)
    std::string bind_ip = "0.0.0.0";   // 수신 NIC 바인드 IP
    uint16_t data_port = 2368;         // 기본 2368
    int recv_timeout_ms = 100;         // 소켓 수신 타임아웃
    bool use_device_ip_filter = false; // 송신자 IP 필터링 여부
    size_t max_packet_per_cycle = 400; // 1사이클당 최대 패킷(안전 장치)
    bool use_azimuth_cycle = false;    // true면 아지무스 wrap으로 사이클 경계
    int azimuth_field_offset = 100;    // 패킷 내 아지무스 바이트 오프셋(예시)
    bool verbose = false;
};

class VLP16Reader {
public:
    VLP16Reader();
    ~VLP16Reader();

    // 파라미터를 받아 초기화(소켓 오픈, 옵션 셋)
    bool on_init(const VLP16Params& params);

    // 실시간 루프: stop() 호출 전까지 패킷 수신
    void on_loop();

    // 종료 플래그 설정
    void stop();

    // 사이클이 완성될 때 호출되는 콜백 등록
    // 콜백은 1사이클의 패킷 벡터를 전달받음
    void set_cycle_callback(std::function<void(const std::vector<VLP16Packet>&)> cb);

private:
    // 내부 함수
    bool open_socket();
    void close_socket();
    bool receive_one_packet(VLP16Packet& out_pkt, std::string& src_ip);
    bool is_new_cycle(const VLP16Packet& pkt);

    // 패킷에서 아지무스(deg x100 등) 추출 예시 함수(원시 예제)
    int extract_azimuth(const VLP16Packet& pkt) const;

private:
    VLP16Params params_;
    int sock_fd_ = -1;
    std::atomic<bool> running_{false};

    // 사이클 누적 버퍼
    std::vector<VLP16Packet> current_cycle_;
    std::mutex cycle_mtx_;

    // 사이클 판단용 상태
    int last_azimuth_ = -1;

    // 콜백
    std::function<void(const std::vector<VLP16Packet>&)> cycle_cb_;
};
