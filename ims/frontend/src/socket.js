// Parses the JSON string from message_encoder.py, and sends it
// to the appropriate widgets. 

//{"type": "attitude", "payload": {"roll": 0.1, "pitch":-0.05, "yaw":1.2, etc}

const SOCKET_URL = 'SocketURL-ToBeFilled';

let socket = null;
const subscribers = {};
let reconnectAttempts = 0;
const RECONNECT_DELAY = 3000;
const MAX_RECONNECT_ATTEMPTS = 10;

function connect() {
  if (socket && (socket.readyState != WebSocket.CLOSED))

  socket = new WebSocket(SOCKET_URL);
  socket.onopen = () => {
      console.log('Connected to RPi');
      reconnectAttempts = 0;
  }
  
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    const { type, payload } = message;
  
    if (subscribers[type]) {
      subscribers[type].forEach(callback => callback(payload));
    }
  }
  
  socket.onerror = (error) => {
    console.error(error);
  }
  
  socket.onclose = () => {
    console.warn('Websocket connection closed');
    if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
      console.error('Max reconnect attempts reached');
      return;
    }
    console.log('Attemping to reconnect...');
    reconnectAttempts++;
    setTimeout(() => {
      connect();
    }, RECONNECT_DELAY);
  }
}

export function subscribe(type, callback) {
  if (!subscribers[type]) {
    subscribers[type] = [];
  }
  subscribers[type].push(callback);
}

export function unsubscribe(type, callback) {
    if (!subscribers[type]) return
    subscribers[type] = subscribers[type].filter(cb => cb !== callback)
}

connect();