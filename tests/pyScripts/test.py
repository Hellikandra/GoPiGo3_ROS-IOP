from mjpeg.client import MJPEGClient
import time
#url='http://example.com:8080/?action=stream'
url='http://192.168.0.43:8000/stream.mjpg'

# Create a new client thread
client = MJPEGClient(url)

# Allocate memory buffers for frames
bufs = client.request_buffers(65536, 50)
for b in bufs:
    client.enqueue_buffer(b)

# Start the client in a background thread
client.start()
i = 0
while True:
    print("in 1 :", i)
    buf = client.dequeue_buffer()
    # <do some work>
    print("in 2 :", i)
    data = memoryview(buf.data)[:buf.used]
    print("in 3 :", i)
    client.enqueue_buffer(buf)
    print("in 4 :", i)
    time.sleep(0.333)
    i += 1
print("End")
