import argparse
import os
from ouster.sdk import pcap
# from ouster.sdk import client # Deprecated
import time

def get_pcap_info(pcap_path, metadata_path=None):
    """
    Analyzes an Ouster pcap file and prints summary information.
    """
    if not os.path.exists(pcap_path):
        print(f"Error: File not found: {pcap_path}")
        return

    print(f"Analyzing pcap file: {pcap_path}")
    if metadata_path:
        print(f"Using metadata: {metadata_path}")
    print("-" * 40)

    try:
        # Try to infer metadata if possible or just check packet types
        # Using PcapPacketSource from ouster.sdk.pcap
        # ouster.sdk.pcap.PcapPacketSource(file, metadata=...)
        if metadata_path:
             from ouster.sdk.sensor import SensorInfo
             with open(metadata_path, 'r') as f:
                 meta_json = f.read()
             info_obj = SensorInfo(meta_json)
             source = pcap.PcapPacketSource(pcap_path, metadata=info_obj)
        else:
             source = pcap.PcapPacketSource(pcap_path)
        
        info = {
            'has_lidar': False,
            'has_imu': False,
            'packet_count': 0,
            'lidar_packet_count': 0,
            'imu_packet_count': 0,
            'first_ts': None,
            'last_ts': None,
            'lidar_payload_size': set(),
            'imu_payload_size': set()
        }

        # Iterate through packets
        for packet in source:
            info['packet_count'] += 1
            
            # Packet object might be LidarPacket or ImuPacket, or generic Packet
            # Check for attributes
            ts = None
            if hasattr(packet, 'capture_timestamp'):
                ts = packet.capture_timestamp
            elif hasattr(packet, 'host_timestamp'):
                ts = packet.host_timestamp
            
            if ts:
                if info['first_ts'] is None:
                    info['first_ts'] = ts
                info['last_ts'] = ts

            # Inspect payload size or type
            # In SDK, packet might have .buf attribute for raw data
            # or we can check isinstance
            
            # Simple size check on the raw buffer if available
            buf_len = 0
            if hasattr(packet, 'buf'): # older sdk
                 buf_len = len(packet.buf)
            elif hasattr(packet, '_data'): # internal maybe?
                 buf_len = len(packet._data)
            else:
                 # Try len(packet) if it supports buffer protocol
                 try:
                     buf_len = len(packet)
                 except:
                     pass

            # Ouster IMU packets are usually small (48 bytes), Lidar are large
            # But the SDK might wrap them.
            # Let's check type name
            type_name = type(packet).__name__
            
            if 'Imu' in type_name or buf_len == 48:
                info['has_imu'] = True
                info['imu_packet_count'] += 1
                if buf_len > 0: info['imu_payload_size'].add(buf_len)
            else:
                info['has_lidar'] = True
                info['lidar_packet_count'] += 1
                if buf_len > 0: info['lidar_payload_size'].add(buf_len)
        
        # Summary Output
        print(f"Total Packets: {info['packet_count']}")
        print(f"Lidar Packets: {info['lidar_packet_count']}")
        print(f"IMU Packets:   {info['imu_packet_count']}")
        
        if info['first_ts'] and info['last_ts']:
            # Timestamp is usually nanoseconds in Ouster SDK
            duration_ns = info['last_ts'] - info['first_ts']
            duration_sec = duration_ns / 1e9
            print(f"Duration:      {duration_sec:.2f} seconds")
            # print(f"Start Time:    {info['first_ts']}") 
            # print(f"End Time:      {info['last_ts']}")
        
        print("-" * 40)
        print("Detailed Info:")
        
        if info['has_imu']:
            print("- IMU Data: Included")
        else:
            print("- IMU Data: Not Found")
            
        if info['has_lidar']:
             print(f"- Lidar Data: Included (Payload sizes: {list(info['lidar_payload_size'])})")
        else:
            print("- Lidar Data: Not Found")

        print("-" * 40)
        print("Note: accurate channel/profile decoding requires a metadata JSON file.")

    except Exception as e:
        print(f"Error reading pcap: {e}")
        print("Tip: If the pcap does not contain embedded metadata, please provide a metadata JSON file.")
        print("     Usage: --pcap <file> --metadata <json>")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check content of Ouster pcap file.")
    parser.add_argument("--pcap", type=str, required=True, help="Path to the pcap file")
    parser.add_argument("--metadata", type=str, default=None, help="Path to the metadata JSON file")
    args = parser.parse_args()
    
    get_pcap_info(args.pcap, args.metadata)

