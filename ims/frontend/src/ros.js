import ROSLIB from 'roslib';
import { ROS_URL, RECONNECT_DELAY_MS, MAX_RECONNECT_ATTEMPTS } from './constants.ts';

export const ros = new ROSLIB.Ros({ url: ROS_URL });

let reconnectAttempts = 0;

ros.on('connection', () => {
  console.log('Connected to rosbridge');
  reconnectAttempts = 0;
  ros.getTopics(
    ({ topics, types }) => {
      console.log('rosapi reachable, topics:', topics.map((t, i) => `${t} (${types[i]})`));
    },
    (error) => console.error('rosapi getTopics failed:', error),
  );
});

ros.on('error', (error) => {
  console.error(error);
});

ros.on('close', () => {
  console.warn('rosbridge connection closed');
  if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
    console.error('Max reconnect attempts reached');
    return;
  }
  console.log('Attempting to reconnect...');
  reconnectAttempts++;
  setTimeout(() => {
    ros.connect(ROS_URL);
  }, RECONNECT_DELAY_MS);
});
