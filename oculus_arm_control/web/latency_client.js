/**
 * Client-side latency monitoring for Oculus VR with ROS
 * Requires rosbridge_suite and roslibjs
 */

// Initialize connection to ROS
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'  // Update with your rosbridge server address
});

// Connection event handlers
ros.on('connection', () => {
  console.log('Connected to rosbridge websocket server.');
  document.getElementById('status').innerHTML = 'Connected';
  document.getElementById('status').className = 'connected';
  startLatencyMonitoring();
});

ros.on('error', (error) => {
  console.error('Error connecting to rosbridge:', error);
  document.getElementById('status').innerHTML = 'Error';
  document.getElementById('status').className = 'error';
});

ros.on('close', () => {
  console.log('Connection to rosbridge closed.');
  document.getElementById('status').innerHTML = 'Disconnected';
  document.getElementById('status').className = 'disconnected';
  stopLatencyMonitoring();
});

// Topic definitions
const pingTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/latency/ping',
  messageType: 'std_msgs/String'
});

const pongTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/latency/pong',
  messageType: 'std_msgs/String'
});

const latencyTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/latency/measurement',
  messageType: 'std_msgs/Float64'
});

// Latency monitoring variables
let sequenceNumber = 0;
let pendingPings = {};
let latencySamples = [];
let monitoringInterval = null;
const MAX_SAMPLES = 100;
const PING_INTERVAL_MS = 100;

// Start latency monitoring
function startLatencyMonitoring() {
  // Subscribe to pong responses
  pongTopic.subscribe((message) => {
    handlePongMessage(message);
  });

  // Subscribe to latency measurements from ROS
  latencyTopic.subscribe((message) => {
    updateLatencyDisplay(message.data);
  });

  // Start sending pings at regular intervals
  monitoringInterval = setInterval(sendPing, PING_INTERVAL_MS);
  
  console.log('Latency monitoring started');
}

// Stop latency monitoring
function stopLatencyMonitoring() {
  if (monitoringInterval) {
    clearInterval(monitoringInterval);
    monitoringInterval = null;
  }
  
  pongTopic.unsubscribe();
  latencyTopic.unsubscribe();
  
  console.log('Latency monitoring stopped');
}

// Send a ping message
function sendPing() {
  const timestamp = Date.now();
  const message = new ROSLIB.Message({
    data: `${sequenceNumber} ${timestamp}`
  });
  
  // Store the timestamp for this sequence number
  pendingPings[sequenceNumber] = timestamp;
  
  // Send the ping
  pingTopic.publish(message);
  
  // Increment sequence number
  sequenceNumber++;
  
  // Clear old pending pings (older than 5 seconds)
  const now = Date.now();
  Object.keys(pendingPings).forEach(seq => {
    if (now - pendingPings[seq] > 5000) {
      delete pendingPings[seq];
    }
  });
}

// Handle a pong message
function handlePongMessage(message) {
  // Parse the message to get sequence number and timestamp
  const parts = message.data.split(' ');
  const seq = parseInt(parts[0]);
  const timestamp = parseInt(parts[1]);
  
  // Check if we have a pending ping with this sequence number
  if (pendingPings.hasOwnProperty(seq)) {
    // Calculate round-trip time
    const latency = Date.now() - timestamp;
    
    // Add to our samples
    addLatencySample(latency);
    
    // Remove from pending pings
    delete pendingPings[seq];
    
    // Update UI
    document.getElementById('latency').textContent = latency.toFixed(2) + ' ms';
    
    // Every 100 pings, log statistics
    if (seq % 100 === 0) {
      logLatencyStatistics();
    }
  }
}

// Add a latency sample to our history
function addLatencySample(latency) {
  latencySamples.push(latency);
  
  // Keep only the last MAX_SAMPLES
  if (latencySamples.length > MAX_SAMPLES) {
    latencySamples.shift();
  }
  
  // Update the graph
  updateLatencyGraph();
}

// Update the latency display based on measurements from ROS
function updateLatencyDisplay(latencyMs) {
  // This function receives latency measurements from ROS
  // Could be used to show additional information or cross-check with client measurements
  document.getElementById('ros-latency').textContent = latencyMs.toFixed(2) + ' ms';
}

// Calculate and log latency statistics
function logLatencyStatistics() {
  if (latencySamples.length === 0) return;
  
  // Calculate statistics
  const sum = latencySamples.reduce((a, b) => a + b, 0);
  const mean = sum / latencySamples.length;
  const min = Math.min(...latencySamples);
  const max = Math.max(...latencySamples);
  
  // Standard deviation
  const sqDiffs = latencySamples.map(x => Math.pow(x - mean, 2));
  const avgSqDiff = sqDiffs.reduce((a, b) => a + b, 0) / sqDiffs.length;
  const stdDev = Math.sqrt(avgSqDiff);
  
  console.log(`Latency statistics (ms):
  Samples: ${latencySamples.length}
  Mean:    ${mean.toFixed(2)}
  Min:     ${min.toFixed(2)}
  Max:     ${max.toFixed(2)}
  StdDev:  ${stdDev.toFixed(2)}`);
  
  // Update statistics in the UI
  document.getElementById('mean-latency').textContent = mean.toFixed(2) + ' ms';
  document.getElementById('min-latency').textContent = min.toFixed(2) + ' ms';
  document.getElementById('max-latency').textContent = max.toFixed(2) + ' ms';
  document.getElementById('stddev-latency').textContent = stdDev.toFixed(2) + ' ms';
}

// Update the latency graph
function updateLatencyGraph() {
  const canvas = document.getElementById('latency-graph');
  if (!canvas) return;
  
  const ctx = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  
  // Clear canvas
  ctx.clearRect(0, 0, width, height);
  
  // Draw background
  ctx.fillStyle = '#f0f0f0';
  ctx.fillRect(0, 0, width, height);
  
  // Draw grid lines
  ctx.strokeStyle = '#cccccc';
  ctx.lineWidth = 1;
  
  // Horizontal lines (every 20px)
  for (let y = 20; y < height; y += 20) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
  
  // Vertical lines (every 20px)
  for (let x = 20; x < width; x += 20) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  
  // If we have no samples, we're done
  if (latencySamples.length === 0) return;
  
  // Find max value for scaling
  const max = Math.max(...latencySamples) * 1.1; // 10% headroom
  
  // Draw the latency line
  ctx.strokeStyle = '#0066cc';
  ctx.lineWidth = 2;
  ctx.beginPath();
  
  // Plot each point
  latencySamples.forEach((latency, index) => {
    const x = (index / latencySamples.length) * width;
    const y = height - (latency / max) * height;
    
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  });
  
  ctx.stroke();
  
  // Draw scale
  ctx.fillStyle = '#000000';
  ctx.font = '10px Arial';
  ctx.fillText('0 ms', 2, height - 2);
  ctx.fillText(`${max.toFixed(0)} ms`, 2, 10);
}

// Connect button handler
document.getElementById('connect-button')?.addEventListener('click', () => {
  const serverUrl = document.getElementById('server-url').value;
  
  // Disconnect if already connected
  if (ros.isConnected) {
    ros.close();
  }
  
  // Connect to the specified server
  ros.connect(serverUrl);
});

// Initialize the UI once the document is loaded
document.addEventListener('DOMContentLoaded', () => {
  // Set up initial UI state
  document.getElementById('status').innerHTML = 'Disconnected';
  document.getElementById('status').className = 'disconnected';
  
  // Set default server URL if input exists
  const serverUrlInput = document.getElementById('server-url');
  if (serverUrlInput) {
    serverUrlInput.value = 'ws://localhost:9090';
  }
  
  // Create canvas for graph if needed
  if (!document.getElementById('latency-graph')) {
    const canvas = document.createElement('canvas');
    canvas.id = 'latency-graph';
    canvas.width = 400;
    canvas.height = 200;
    document.getElementById('graph-container').appendChild(canvas);
  }
}); 