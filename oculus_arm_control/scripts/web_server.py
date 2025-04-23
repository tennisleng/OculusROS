#!/usr/bin/env python3

import rospy
import argparse
import os
import signal
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import socket

class RosHTTPServer:
    def __init__(self, webroot, port=8080):
        self.webroot = os.path.abspath(webroot)
        self.port = port
        self.server = None
        self.server_thread = None
        
        # Verify the webroot exists
        if not os.path.isdir(self.webroot):
            rospy.logerr(f"Web root directory not found: {self.webroot}")
            sys.exit(1)
            
        rospy.loginfo(f"Using web root directory: {self.webroot}")
            
        # Create a custom request handler with the specified webroot
        class CustomHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=webroot, **kwargs)
                
            def log_message(self, format, *args):
                # Send log messages to ROS log
                rospy.logdebug(f"HTTP: {self.address_string()} - {format % args}")
                
            def end_headers(self):
                # Add CORS headers to allow cross-origin requests
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                super().end_headers()
        
        self.handler_class = CustomHandler
    
    def start(self):
        """Start the HTTP server in a background thread"""
        try:
            # Bind to all interfaces by default
            self.server = HTTPServer(('0.0.0.0', self.port), self.handler_class)
            
            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.server.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            # Get actual server address
            host, port = self.server.socket.getsockname()
            local_ip = self.get_local_ip()
            
            rospy.loginfo(f"Web server started on:")
            rospy.loginfo(f" - Local:   http://localhost:{port}")
            rospy.loginfo(f" - Network: http://{local_ip}:{port}")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to start web server: {e}")
            return False
    
    def stop(self):
        """Stop the HTTP server"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
            rospy.loginfo("Web server stopped")
    
    def get_local_ip(self):
        """Get the local machine's IP address"""
        try:
            # Create a dummy connection to determine the local IP address
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"


def signal_handler(sig, frame):
    """Handle shutdown signals"""
    rospy.loginfo("Shutting down web server...")
    rospy.signal_shutdown("Received shutdown signal")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ROS HTTP Server for Latency Monitoring Interface")
    parser.add_argument("--port", type=int, default=8080,
                        help="HTTP server port (default: 8080)")
    parser.add_argument("--webroot", type=str, default="web",
                        help="Web root directory (default: web)")
    
    args = parser.parse_args()
    
    # Initialize ROS node
    rospy.init_node('web_server', anonymous=True)
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and start the server
    server = RosHTTPServer(args.webroot, args.port)
    if not server.start():
        sys.exit(1)
    
    # Keep the node running until shutdown
    rospy.spin()
    
    # Stop the server when node is shutting down
    server.stop()


if __name__ == "__main__":
    main() 