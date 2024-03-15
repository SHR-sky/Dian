import socket

# 获取ipv4地址
ip_address = socket.gethostbyname(socket.gethostname())

HOST, PORT = ip_address.split(':')

# 由于未知原因，网络接口一直显示0.0.0.0，所以需要手动设置主机和端口，从电脑属性查看，或者手机热点查看热点
# 例如
# HOST = '10.19.187.65'  # 网络接口
# PORT = 12345  # 端口号

# 创建套接字对象
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定主机和端口
server_socket.bind((HOST, PORT))

# 开始监听连接
server_socket.listen(5)

print(f"Server is listening on {HOST}:{PORT}")

while True:
    # 等待客户端连接
    client_socket, addr = server_socket.accept()
    print(f"Connection from {addr}")

    # 接收数据
    data = client_socket.recv(1024).decode('utf-8')
    print(f"Received data: {data}")
    # 此处有待实现写入csv数据

    # 回复客户端
    response = "Data received successfully"
    client_socket.sendall(response.encode('utf-8'))

    # 关闭客户端连接
    client_socket.close()