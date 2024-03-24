# CODE: CREATE SOCKET SERVER, CONNECT TO PI SLAVE AND GET SENSOR DATA

# importing libraries ||
import socket
import pickle

# Global Variables ||


# Creating Functions ||
def Setup_Server(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(1)
    s.settimeout(30)  # timeout for socket
    return s

def Recv_Data(client, buffer_size):
    data = client.recv(buffer_size).decode('utf-8')
    return data

def Send_Data(client, msg):
    client.send(msg.encode('utf-8'))

def Recv_Pickle(client, buffer_size):
    Pickle = client.recv(buffer_size)
    if Pickle == 0:  # in case buffer is empty, return an empty list
        data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        return data
    data = pickle.loads(Pickle)
    return data

def Send_Pickle(client, Pickle):
    data = pickle.dumps(Pickle)
    client.send(data)


# Code Start ||


# For Testing ||
if __name__ == '__main__':

    try:
        host1 = socket.gethostname()
        port1 = 4444
        s = Setup_Server(host1, port1)
        client, address = s.accept()  # wait for client to connect
        data = Recv_Data(client, 1024)
        print(data)
        msg = "Hi how are you ?"
        Send_Data(client, msg)
        msg2 = [2, 4, 6, 8, 9, 3, 5, 0, 1, 2]
        My_Pickle = Recv_Pickle(client, 1024)
        print(My_Pickle)
        Send_Pickle(client, msg2)
    finally:
        print("FINISHING PROGRAM")
        s.close()