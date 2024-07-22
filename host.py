import http.server
import socketserver
import os
import mimetypes

# Set the directory you want to serve files from
web_dir = os.path.join(os.path.dirname(__file__), '.')
os.chdir(web_dir)

PORT = 8000

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def guess_type(self, path):
        base, ext = os.path.splitext(path)
        if ext == '.cpp' or ext == '.h':
            return 'text/plain'
        return super().guess_type(path)

with socketserver.TCPServer(("", PORT), CustomHTTPRequestHandler) as httpd:
    print("Serving at port", PORT)
    httpd.serve_forever()
