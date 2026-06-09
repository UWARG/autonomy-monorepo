// Parses the JSON string from message_encoder.py, and sends it
// to the appropriate widgets. 


//{"type": "attitude", "payload": {"roll": 0.1, "pitch":-0.05, "yaw":1.2, etc}

const SOCKET_URL = 'SocketURL-ToBeFilled';

const socket = new WebSocket(SOCKET_URL);
const subscribers = {};

socket.onopen = () => {
    console.log('Connected to RPi')
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