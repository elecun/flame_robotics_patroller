#!/usr/bin/env python3
"""Receive and inspect Velodyne VLP-16 data packets without extra packages.

Example: python3 test/vlp16_udp_test.py --bind 192.168.100.2 --duration 10
"""

import argparse
import socket
import struct
import sys
import time
from collections import Counter


PACKET_SIZE = 1206
BLOCK_COUNT = 12
BLOCK_SIZE = 100
BLOCK_MARKER = b"\xff\xee"


def inspect_packet(data):
    """Return (first azimuth, nonzero distance count, timestamp, factory) or None."""
    if len(data) != PACKET_SIZE:
        return None

    for block in range(BLOCK_COUNT):
        offset = block * BLOCK_SIZE
        if data[offset : offset + 2] != BLOCK_MARKER:
            return None
        if struct.unpack_from("<H", data, offset + 2)[0] >= 36000:
            return None

    distances = (
        struct.unpack_from("<H", data, block * BLOCK_SIZE + 4 + channel * 3)[0]
        for block in range(BLOCK_COUNT)
        for channel in range(32)
    )
    azimuth = struct.unpack_from("<H", data, 2)[0] / 100.0
    timestamp = struct.unpack_from("<I", data, 1200)[0]
    return azimuth, sum(value > 0 for value in distances), timestamp, data[1204:1206].hex()


def main():
    parser = argparse.ArgumentParser(description="VLP-16 UDP 2368 수신 및 패킷 형식 확인")
    parser.add_argument("--bind", default="192.168.100.2", help="수신할 로컬 IP (기본값: 192.168.100.2)")
    parser.add_argument("--port", type=int, default=2368, help="수신 UDP 포트 (기본값: 2368)")
    parser.add_argument("--source", help="기대하는 라이다 송신 IP; 지정 시 별도로 집계")
    parser.add_argument("--duration", type=float, default=10.0, help="측정 시간, 초 (기본값: 10)")
    args = parser.parse_args()
    if not 0 <= args.port <= 65535 or args.duration <= 0:
        parser.error("--port는 0~65535, --duration은 양수여야 합니다")

    counts = Counter()
    sources = Counter()
    first_valid = None
    last_valid = None
    start = time.monotonic()
    deadline = start + args.duration

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((args.bind, args.port))
            print(f"수신 대기: {args.bind}:{sock.getsockname()[1]} / {args.duration:g}초", flush=True)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                sock.settimeout(min(remaining, 1.0))
                try:
                    data, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                counts["udp"] += 1
                sources[address[0]] += 1
                result = inspect_packet(data)
                if result is None:
                    counts["invalid"] += 1
                    continue
                counts["valid"] += 1
                if args.source and address[0] != args.source:
                    counts["other_source"] += 1
                    continue
                counts["matched"] += 1
                if first_valid is None:
                    first_valid = (address, result)
                last_valid = (address, result)
    except KeyboardInterrupt:
        print("\n측정 중단")
    except OSError as exc:
        print(f"UDP 소켓 오류: {exc}", file=sys.stderr)
        return 2

    elapsed = max(time.monotonic() - start, 0.001)
    print(f"수신 UDP: {counts['udp']}개 ({counts['udp'] / elapsed:.1f}개/초)")
    print(f"VLP-16 형식 일치: {counts['valid']}개, 형식 불일치: {counts['invalid']}개")
    print("송신 IP: " + (", ".join(f"{ip}={n}개" for ip, n in sources.most_common()) or "없음"))
    if args.source:
        print(f"지정 송신 IP {args.source}: {counts['matched']}개, 다른 IP: {counts['other_source']}개")
    if first_valid:
        address, (azimuth, nonzero, timestamp, factory) = first_valid
        print(f"첫 유효 패킷: {address[0]}:{address[1]}, 방위각 {azimuth:.2f}°, "
              f"거리값 {nonzero}/384개, 타임스탬프 {timestamp}, factory 0x{factory}")
        address, (azimuth, nonzero, timestamp, factory) = last_valid
        print(f"마지막 유효 패킷: 방위각 {azimuth:.2f}°, 거리값 {nonzero}/384개")

    if counts["matched"]:
        print("결과: VLP-16 형식의 UDP 패킷 수신 확인")
        return 0
    if counts["udp"]:
        print("결과: UDP 수신됨, 기대한 송신 IP/패킷 형식은 확인되지 않음")
    else:
        print("결과: 측정 시간 동안 UDP 패킷 수신 없음")
    return 1


if __name__ == "__main__":
    sys.exit(main())
