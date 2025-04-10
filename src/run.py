# start a http server
import http.server
import socketserver
import os

PORT = 8004
Handler = http.server.SimpleHTTPRequestHandler
os.chdir(os.path.dirname(__file__))
httpd = socketserver.TCPServer(("", PORT), Handler)
print(f"Serving at port {PORT}")
# create functionaly to stop the server
def stop_server():
    print("Stopping server...")
    httpd.shutdown()
    print("Server stopped.")
# create a function to start the server
def start_server():
    print("Starting server...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        stop_server()
    except SystemExit:
        stop_server()
    except Exception as e:
        print(f"Error: {e}")
# start the server  
start_server()