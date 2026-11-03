import ROSLIB from 'roslib';
import { ROS_URL, RECONNECT_DELAY_MS } from './constants.ts';

export const ros = new ROSLIB.Ros({ url: ROS_URL });

ros.on('connection', () => {
  console.log('Connected to rosbridge');
});

ros.on('error', (error) => {
  console.error(error);
});

// Keep retrying for as long as the page is open: airside may start (or reboot)
// well after IMS does.
ros.on('close', () => {
  console.warn(`rosbridge connection closed, retrying in ${RECONNECT_DELAY_MS} ms`);
  setTimeout(() => ros.connect(ROS_URL), RECONNECT_DELAY_MS);
});
