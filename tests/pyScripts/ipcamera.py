import socket
import sys

host = "192.168.0.43"
port = 8000
def main():
	print("hello world")
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	except socket.error:
		print("Failed to create socket")
		sys.exit()

	print("# Getting remote IP address")
	try:
		remote_ip = socket.gethostbyname(host)
	except socket.gaierror:
		print("Hostname could not be resolved. Exiting")
		sys.exit()

	# Connect to remote server 
	print('# Connecting to server, ' + host + '(' + remote_ip + ')')
	s.connect((remote_ip, port))

	# Send data to remote server
	print("# Sending data to server")
	request ="GET /stream.mjpg HTTP/1.1\r\n\r\n"

	try:
		s.sendall(request.encode())
	except socket.error:
		print("Send Failed")
		sys.exit

	# Receive data
	print("# Recevied data from server")
	reply = s.recv(4096)
	print(reply)
if __name__ == '__main__':
	main()
# EOF