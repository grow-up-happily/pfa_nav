#!/usr/bin/env python3
import struct
import time
import os

# 定义数据结构
# 对应节点中的: payload = struct.pack('<B8f', ...) + CRC
# Payload: 1 byte (uint8) + 8 floats (32 bytes) = 33 bytes
# CRC: 2 bytes (uint16)
# Total: 35 bytes
PACKET_SIZE = 35
PAYLOAD_FORMAT = '<B8f'
CRC_FORMAT = '<H'

def main():
    serial_path = '/tmp/virtual_gimbal'
    print(f"Waiting for data on {serial_path}...")
    
    # 循环等待文件被 socat 创建
    while not os.path.exists(serial_path):
        time.sleep(1)
        
    try:
        # 打开虚拟串口文件进行读取
        with open(serial_path, 'rb') as f:
            while True:
                # 读取一个完整的数据包
                data = f.read(PACKET_SIZE)
                if not data or len(data) < PACKET_SIZE:
                    time.sleep(0.01)
                    continue
                
                # 解析数据
                payload = data[:-2]
                crc_bytes = data[-2:]
                
                try:
                    unpacked = struct.unpack(PAYLOAD_FORMAT, payload)
                    crc = struct.unpack(CRC_FORMAT, crc_bytes)[0]
                    
                    flag = unpacked[0]
                    floats = unpacked[1:]
                    
                    # 打印解析结果 (这里重点关注vx, vy，也是代码里最后两个float)
                    # 代码逻辑: struct.pack(..., 0.0, ..., vx, vy)
                    vx = floats[6]
                    vy = floats[7]
                    
                    print("-" * 30)
                    print(f"Received Packet (Hex): {data.hex().upper()}")
                    print(f"Flag: {flag}")
                    print(f"VX:   {vx:.4f}")
                    print(f"VY:   {vy:.4f}")
                    print(f"CRC:  {crc:04X}")
                    
                except struct.error as e:
                    print(f"Parse error: {e}")

    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
