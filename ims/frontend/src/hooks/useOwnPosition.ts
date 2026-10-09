import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import type { PositionMessage } from '../types';

const GLOBAL_POSITION_TOPIC = 'mavros/global_position/global';

interface NavSatFix {
  latitude: number;
  longitude: number;
  altitude: number;
}

/** Own GPS fix, straight from rosbridge (not routed through the relay). */
export default function useOwnPosition(): PositionMessage | undefined {
  const [position, setPosition] = useState<PositionMessage>();

  useEffect(() => {
    const fixTopic = new ROSLIB.Topic<NavSatFix>({
      ros,
      name: GLOBAL_POSITION_TOPIC,
      messageType: 'sensor_msgs/NavSatFix',
    });
    const onFix = (msg: NavSatFix) => {
      setPosition({ lat: msg.latitude, lon: msg.longitude, alt: msg.altitude });
    };
    fixTopic.subscribe(onFix);
    return () => fixTopic.unsubscribe(onFix);
  }, []);

  return position;
}
