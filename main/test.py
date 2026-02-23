import socket
import struct

# 模拟卫星 (1,2) 的监听端口
# 代码中逻辑是 8000 + j，所以 (1,2) 对应 8002
UDP_IP = "127.0.0.1"
UDP_PORT = 8002

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"=======================================")
print(f"   仿真卫星 (1,2) 接收端已启动 ")
print(f"   监听地址: {UDP_IP}:{UDP_PORT}")
print(f"=======================================")

while True:
    print("等待接收数据...")
    data, addr = sock.recvfrom(1024) # 阻塞等待
    
    print(f"\n[RX] 收到来自 {addr} 的 UDP 包!")
    print(f"     原始数据(HEX): {data.hex().upper()}")
    
    # 解析 SSP 头部 (前11个字节)
    # 格式对应 C结构体: B(start) B(si) B(sj) B(di) B(dj) B(type) H(path_len) I(data_len)
    if len(data) >= 12:
        # 解析前 12 个字节
        header = struct.unpack("<BBBBBBHI", data[:12])
        
        start_byte = header[0]
        src_node   = (header[1], header[2])
        dst_node   = (header[3], header[4])
        req_type   = header[5]
        path_len   = header[6]
        
        # 提取 Payload (从第 12 字节开始)
        payload_path = data[12:].decode('utf-8', errors='ignore')
        
        print(f"     --- SSP 协议解析 ---")
        print(f"     起始符: 0x{start_byte:02X} (OK)" if start_byte == 0x5A else "起始符错误")
        print(f"     源卫星: {src_node}")
        print(f"     目卫星: {dst_node}")
        print(f"     类型:   {req_type} (1=READ)")
        print(f"     路径:   {payload_path}")
        print(f"     --------------------")
    else:
        print("数据包太短，无法解析")