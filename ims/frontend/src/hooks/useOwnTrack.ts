import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import { haversineM } from '../geo';
import type { PositionMessage } from '../types';

const GLOBAL_POSITION_TOPIC = 'mavros/global_position/global';
const HEADING_TOPIC = 'mavros/global_position/compass_hdg';
const REL_ALT_TOPIC = 'mavros/global_position/rel_alt';

/** Seconds of flight history kept for the trail. */
const TRAIL_WINDOW_S = 60;
/** Skip trail samples closer than this to the last one, so hovering adds nothing. */
const TRAIL_MIN_STEP_M = 0.5;

interface NavSatFix {
  latitude: number;
  longitude: number;
  altitude: number;
}

export interface TrailSample {
  lat: number;
  lon: number;
  /** Client receipt time, ms epoch. */
  t: number;
}

export interface OwnTrack {
  position?: PositionMessage;
  /** Degrees clockwise from true north. */
  heading?: number;
  /** Metres above home, the frame setpoints and waypoints use. */
  relAlt?: number;
  trail: TrailSample[];
}

/** Own GPS fix, compass heading, relative altitude and recent trail, from airside over rosbridge. */
export default function useOwnTrack(): OwnTrack {
  const [position, setPosition] = useState<PositionMessage>();
  const [heading, setHeading] = useState<number>();
  const [relAlt, setRelAlt] = useState<number>();
  const [trail, setTrail] = useState<TrailSample[]>([]);

  useEffect(() => {
    const fixTopic = new ROSLIB.Topic<NavSatFix>({
      ros,
      name: GLOBAL_POSITION_TOPIC,
      messageType: 'sensor_msgs/NavSatFix',
    });
    const headingTopic = new ROSLIB.Topic<{ data: number }>({
      ros,
      name: HEADING_TOPIC,
      messageType: 'std_msgs/Float64',
    });
    const relAltTopic = new ROSLIB.Topic<{ data: number }>({
      ros,
      name: REL_ALT_TOPIC,
      messageType: 'std_msgs/Float64',
    });

    const onFix = (msg: NavSatFix) => {
      if (!Number.isFinite(msg.latitude) || !Number.isFinite(msg.longitude)) return;
      const t = Date.now();
      setPosition({ lat: msg.latitude, lon: msg.longitude, alt: msg.altitude });
      setTrail((prev) => {
        const cutoff = t - TRAIL_WINDOW_S * 1000;
        const kept = prev[0] && prev[0].t < cutoff ? prev.filter((s) => s.t >= cutoff) : prev;
        const last = kept[kept.length - 1];
        if (last && haversineM(last.lat, last.lon, msg.latitude, msg.longitude) < TRAIL_MIN_STEP_M) {
          return kept;
        }
        return [...kept, { lat: msg.latitude, lon: msg.longitude, t }];
      });
    };
    const onHeading = (msg: { data: number }) => {
      if (Number.isFinite(msg.data)) setHeading(msg.data);
    };
    const onRelAlt = (msg: { data: number }) => {
      if (Number.isFinite(msg.data)) setRelAlt(msg.data);
    };

    fixTopic.subscribe(onFix);
    headingTopic.subscribe(onHeading);
    relAltTopic.subscribe(onRelAlt);
    return () => {
      relAltTopic.unsubscribe(onRelAlt);
      fixTopic.unsubscribe(onFix);
      headingTopic.unsubscribe(onHeading);
    };
  }, []);

  return { position, heading, relAlt, trail };
}
