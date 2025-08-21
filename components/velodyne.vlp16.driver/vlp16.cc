#include "vlp16.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <cstring>
#include <iostream>

VLP16Reader::VLP16Reader() {}
VLP16Reader::~VLP16Reader() { close_socket(); }

bool VLP16Reader::on_init(const VLP16Params& params) {
    params_ = params;
    return open_socket();
}

bool VLP16Reader::open_socket() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::perror("socket");
        return false;
    }

    // 바인드
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(params_.data_port);
    addr.sin_addr.s_addr = inet_addr(params_.bind_ip.c_str());

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        close_socket();
        return false;
    }

    // 타임아웃
    timeval tv{};
    tv.tv_sec = params_.recv_timeout_ms / 1000;
    tv.tv_usec = (params_.recv_timeout_ms % 1000) * 1000;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::perror("setsockopt SO_RCVTIMEO");
        // 계속 진행은 가능
    }

    // 큰 버퍼 권장
    int rcvbuf = 4 * 1024 * 1024;
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    if (params_.verbose) {
        std::cout << "[VLP16Reader] Listening on " << params_.bind_ip
                  << ":" << params_.data_port << std::endl;
    }

    return true;
}

void VLP16Reader::close_socket() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void VLP16Reader::stop() {
    running_.store(false);
}

void VLP16Reader::set_cycle_callback(std::function<void(const std::vector<VLP16Packet>&)> cb) {
    cycle_cb_ = std::move(cb);
}

bool VLP16Reader::receive_one_packet(VLP16Packet& out_pkt, std::string& src_ip) {
    if (sock_fd_ < 0) return false;

    // VLP-16 데이터 패킷은 보통 1206 bytes
    uint8_t buf[2048];
    sockaddr_in src_addr{};
    socklen_t addrlen = sizeof(src_addr);
    ssize_t n = ::recvfrom(sock_fd_, buf, sizeof(buf), 0,
                           reinterpret_cast<sockaddr*>(&src_addr), &addrlen);
    if (n <= 0) {
        return false; // timeout or error
    }

    char ipbuf[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(src_addr.sin_addr), ipbuf, INET_ADDRSTRLEN);
    src_ip = ipbuf;

    // 디바이스 IP 필터링(옵션)
    if (params_.use_device_ip_filter && !params_.device_ip.empty()) {
        if (src_ip != params_.device_ip) {
            return false;
        }
    }

    out_pkt.data.assign(buf, buf + n);
    out_pkt.recv_time = std::chrono::steady_clock::now();
    return true;
}

int VLP16Reader::extract_azimuth(const VLP16Packet& pkt) const {
    // 주의: 실제 VLP-16 패킷 포맷을 참고하여 아지무스 위치와 엔디안을 정확히 처리해야 합니다.
    // 일반적으로 블록 헤더마다 azimuth(2 bytes, 0~35999) 존재. 여기서는 예시 오프셋 사용.
    // 안전 점검
    const int off = params_.azimuth_field_offset;
    if (pkt.data.size() < static_cast<size_t>(off + 2)) return -1;

    // 리틀엔디안 가정
    uint16_t az = static_cast<uint16_t>(pkt.data[off] | (pkt.data[off + 1] << 8));
    return static_cast<int>(az); // 0~35999 (0.01 deg)
}

bool VLP16Reader::is_new_cycle(const VLP16Packet& pkt) {
    if (!params_.use_azimuth_cycle) {
        return false;
    }
    int az = extract_azimuth(pkt);
    if (az < 0) return false;

    bool new_cycle = false;
    if (last_azimuth_ >= 0) {
        // wrap-around 감지: 예를 들어 35000 -> 1000 처럼 큰 감소
        if (az + 2000 < last_azimuth_) { // 임계값은 상황에 맞게 조정
            new_cycle = true;
        }
    }
    last_azimuth_ = az;
    return new_cycle;
}

void VLP16Reader::on_loop() {
    running_.store(true);
    current_cycle_.clear();
    last_azimuth_ = -1;

    while (running_.load()) {
        VLP16Packet pkt;
        std::string src_ip;
        bool ok = receive_one_packet(pkt, src_ip);
        if (!ok) {
            // timeout 등: 필요시 continue
            continue;
        }

        // 1) 사이클 경계 체크(옵션)
        bool cycle_boundary = is_new_cycle(pkt);

        // 2) 현재 패킷을 사이클 벡터에 추가
        {
            std::lock_guard<std::mutex> lk(cycle_mtx_);
            current_cycle_.push_back(std::move(pkt));

            // 안전: 과도한 누적 방지
            if (current_cycle_.size() >= params_.max_packet_per_cycle) {
                cycle_boundary = true;
            }
        }

        // 3) 사이클 종료 시 콜백 호출 및 벡터 스왑/비움
        if (cycle_boundary) {
            std::vector<VLP16Packet> finished_cycle;
            {
                std::lock_guard<std::mutex> lk(cycle_mtx_);
                finished_cycle.swap(current_cycle_);
                current_cycle_.clear();
            }
            if (cycle_cb_) {
                cycle_cb_(finished_cycle);
            }
        }
    }
}
