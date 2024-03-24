# CODE: ESTABLISH A SOCKET CONNECTION WITH THE SERVER

# importing libraries ||
import socket

# Global Variables ||


# Creating Functions ||
def Setup_Connection():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(30)  # timeout for connection attempt in seconds
    return s

def Send_Data(msg, s):
    msg = str(msg)
    s.send(msg.encode('utf-8'))

def Recv_Data(buffer_size, s):
    data = s.recv(buffer_size).decode('utf-8')
    return data

def Send_Pickle(data, s):
    s.send(data)


# Code Start ||


# For Testing ||
if __name__ == "__main__":

    host1 = socket.gethostname()
    port1 = 4444
    message = 'Hello World !!'
    try:
        s = Setup_Connection()
        s.connect((host1, port1))
        print("Created socket connection")
        Send_Data(message, s)
        print("Sent... Waiting for response")
        response = Recv_Data(1024, s)
        print("Response: {}".format(response))
    finally:
        print("FINISHING PROGRAM")
        s.close()
