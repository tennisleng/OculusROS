#!/usr/bin/env python3

import rospy
import random
import time
import argparse
import threading
from std_msgs.msg import String, Float64, Bool, UInt32

class LatencyResponder:
    def __init__(self, args):
        # Parse arguments
        self.fixed_delay = args.fixed_delay
        self.variable_delay = args.variable_delay
        self.packet_loss = args.packet_loss
        self.network_jitter = args.jitter
        self.burst_loss = args.burst_loss
        self.burst_length = args.burst_length
        self.burst_interval = args.burst_interval
        
        # Initialize node
        rospy.init_node('latency_responder')
        
        # Create subscribers and publishers
        self.ping_sub = rospy.Subscriber('/latency/ping', String, self.ping_callback)
        self.pong_pub = rospy.Publisher('/latency/pong', String, queue_size=10)
        self.responder_status_pub = rospy.Publisher('/latency/responder_status', Bool, queue_size=10)
        self.simulated_delay_pub = rospy.Publisher('/latency/simulated_delay', Float64, queue_size=10)
        
        # Status tracking
        self.response_count = 0
        self.drop_count = 0
        self.in_burst_mode = False
        self.burst_timer = None
        self.sequence_numbers = []  # Track processed sequence numbers to detect duplicates
        
        # Register shutdown handler
        rospy.on_shutdown(self.shutdown_handler)
        
        # Start status publisher
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        # Schedule the first burst if burst mode is enabled
        if self.burst_loss > 0 and self.burst_interval > 0:
            self.schedule_burst()
        
        rospy.loginfo(f"Latency responder initialized with parameters:")
        rospy.loginfo(f"- Fixed delay: {self.fixed_delay} ms")
        rospy.loginfo(f"- Variable delay: {self.variable_delay} ms")
        rospy.loginfo(f"- Network jitter: {self.network_jitter} ms")
        rospy.loginfo(f"- Packet loss: {self.packet_loss*100}%")
        rospy.loginfo(f"- Burst loss: {self.burst_loss*100}%")
        rospy.loginfo(f"- Burst interval: {self.burst_interval} seconds")
        rospy.loginfo(f"- Burst length: {self.burst_length} seconds")
    
    def ping_callback(self, msg):
        # Extract sequence number to track duplicates and out-of-order messages
        try:
            seq = int(msg.data.split(' ')[0])
            if seq in self.sequence_numbers:
                rospy.logwarn(f"Received duplicate ping with sequence {seq}")
                return
            
            # Keep only the last 1000 sequence numbers to avoid unlimited memory growth
            self.sequence_numbers.append(seq)
            if len(self.sequence_numbers) > 1000:
                self.sequence_numbers.pop(0)
                
        except (ValueError, IndexError):
            rospy.logwarn(f"Malformed ping message: {msg.data}")
            seq = -1  # Unknown sequence
            
        # Check if we should drop this packet
        if self.should_drop_packet():
            self.drop_count += 1
            if self.drop_count % 10 == 0:
                rospy.loginfo(f"Intentionally dropped {self.drop_count} packets so far")
            return
        
        # Calculate the simulated delay
        delay_ms = self.calculate_delay()
        
        # Publish the simulated delay for monitoring
        delay_msg = Float64()
        delay_msg.data = delay_ms
        self.simulated_delay_pub.publish(delay_msg)
        
        # Apply the simulated delay
        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)
        
        # Echo back the message as a pong
        self.pong_pub.publish(msg)
        
        # Increment counter and periodically log
        self.response_count += 1
        if self.response_count % 100 == 0:
            drop_pct = 0
            if self.response_count + self.drop_count > 0:
                drop_pct = (self.drop_count / (self.response_count + self.drop_count)) * 100
            
            rospy.loginfo(f"Responded to {self.response_count} pings, dropped {self.drop_count} " +
                          f"({drop_pct:.1f}%), current delay: {delay_ms:.2f} ms")
    
    def calculate_delay(self):
        """Calculate the simulated delay based on the configuration"""
        delay = self.fixed_delay
        
        # Add variable delay component
        if self.variable_delay > 0:
            delay += random.uniform(0, self.variable_delay)
        
        # Add jitter component (can be positive or negative)
        if self.network_jitter > 0:
            delay += random.uniform(-self.network_jitter, self.network_jitter)
            
        # Ensure we don't have negative delay
        return max(0, delay)
    
    def should_drop_packet(self):
        """Determine if the current packet should be dropped"""
        # If we're in a burst loss period, drop with the burst probability
        if self.in_burst_mode:
            return random.random() < self.burst_loss
        
        # Otherwise use the regular packet loss probability
        return random.random() < self.packet_loss
    
    def schedule_burst(self):
        """Schedule the next burst loss period"""
        # Schedule a burst to start after the interval
        delay = self.burst_interval + random.uniform(-self.burst_interval/4, self.burst_interval/4)
        self.burst_timer = threading.Timer(delay, self.start_burst)
        self.burst_timer.daemon = True
        self.burst_timer.start()
        rospy.loginfo(f"Scheduled next burst loss period in {delay:.1f} seconds")
    
    def start_burst(self):
        """Start a burst loss period"""
        self.in_burst_mode = True
        rospy.loginfo("Starting burst loss period")
        
        # Schedule the end of the burst
        burst_length = self.burst_length + random.uniform(-self.burst_length/4, self.burst_length/4)
        threading.Timer(burst_length, self.end_burst).start()
    
    def end_burst(self):
        """End a burst loss period"""
        self.in_burst_mode = False
        rospy.loginfo("Ending burst loss period")
        
        # Schedule the next burst
        self.schedule_burst()
    
    def publish_status(self, event):
        """Publish the responder status periodically"""
        status = Bool()
        status.data = True  # Responder is alive
        self.responder_status_pub.publish(status)
    
    def shutdown_handler(self):
        """Handle shutdown cleanly"""
        rospy.loginfo(f"Shutting down latency responder after responding to {self.response_count} pings")
        if self.burst_timer:
            self.burst_timer.cancel()
    
    def run(self):
        # Keep the node running
        rospy.spin()


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ROS Latency Responder with network condition simulation")
    parser.add_argument("--fixed-delay", type=float, default=0.0,
                        help="Fixed delay to add in milliseconds")
    parser.add_argument("--variable-delay", type=float, default=0.0,
                        help="Variable delay range in milliseconds (0-value)")
    parser.add_argument("--jitter", type=float, default=0.0,
                        help="Network jitter in milliseconds (+/- value)")
    parser.add_argument("--packet-loss", type=float, default=0.0,
                        help="Packet loss probability (0.0-1.0)")
    parser.add_argument("--burst-loss", type=float, default=0.0,
                        help="Burst packet loss probability (0.0-1.0)")
    parser.add_argument("--burst-length", type=float, default=5.0,
                        help="Length of burst loss periods in seconds")
    parser.add_argument("--burst-interval", type=float, default=60.0,
                        help="Interval between burst loss periods in seconds")
    
    args = parser.parse_args()
    
    try:
        responder = LatencyResponder(args)
        responder.run()
    except rospy.ROSInterruptException:
        pass 