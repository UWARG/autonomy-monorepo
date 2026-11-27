import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import type { PositionMessage } from '../types';

/** What the position controller is sending the FC, after obstacle avoidance. */
const SETPOINT_TOPIC = 'mavros/setpoint_raw/global';
/** The waypoint the engine wants to reach (before avoidance). */
const TARGET_TOPIC = 'position_controller/target';
/** The ordered lap waypoints, latched by the engine's LoadWaypointList. */
const WAYPOINTS_TOPIC = 'engine/waypoints';

/** The controller publishes at 10 Hz while armed in GUIDED. */
const SETPOINT_STALE_MS = 1000;
/** The engine republishes its target every 500 ms tick. */
const TARGET_STALE_MS = 1500;

interface GlobalPositionTarget {
  latitude: number;
  longitude: number;
  altitude: number;
}

interface Coordinate {
  lat: number;
  lon: number;
  alt: number;
}

export interface NavigationState {
  /** Latest position setpoint; altitude is relative to home. */
  setpoint?: PositionMessage;
  setpointStale: boolean;
  /** Latest engine position target; altitude is relative to home. */
  target?: PositionMessage;
  targetStale: boolean;
  /** Lap waypoints in flight order, once the engine has loaded them. */
  waypoints: PositionMessage[];
}

const isFiniteCoord = (c: PositionMessage) =>
  Number.isFinite(c.lat) && Number.isFinite(c.lon) && Number.isFinite(c.alt);

export default function useNavigation(): NavigationState {
  const [setpoint, setSetpoint] = useState<{ value: PositionMessage; at: number }>();
  const [target, setTarget] = useState<{ value: PositionMessage; at: number }>();
  const [waypoints, setWaypoints] = useState<PositionMessage[]>([]);
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    const setpointTopic = new ROSLIB.Topic<GlobalPositionTarget>({
      ros,
      name: SETPOINT_TOPIC,
      messageType: 'mavros_msgs/GlobalPositionTarget',
    });
    const targetTopic = new ROSLIB.Topic<Coordinate>({
      ros,
      name: TARGET_TOPIC,
      messageType: 'airside_interfaces/Coordinate',
    });
    const waypointsTopic = new ROSLIB.Topic<{ waypoints: Coordinate[] }>({
      ros,
      name: WAYPOINTS_TOPIC,
      messageType: 'airside_interfaces/WaypointList',
    });

    const onSetpoint = (msg: GlobalPositionTarget) => {
      const value = { lat: msg.latitude, lon: msg.longitude, alt: msg.altitude };
      if (isFiniteCoord(value)) setSetpoint({ value, at: Date.now() });
    };
    const onTarget = (msg: Coordinate) => {
      const value = { lat: msg.lat, lon: msg.lon, alt: msg.alt };
      if (isFiniteCoord(value)) setTarget({ value, at: Date.now() });
    };
    const onWaypoints = (msg: { waypoints: Coordinate[] }) => {
      setWaypoints(msg.waypoints.map((w) => ({ lat: w.lat, lon: w.lon, alt: w.alt })).filter(isFiniteCoord));
    };

    setpointTopic.subscribe(onSetpoint);
    targetTopic.subscribe(onTarget);
    waypointsTopic.subscribe(onWaypoints);
    const tick = setInterval(() => setNow(Date.now()), 250);

    return () => {
      clearInterval(tick);
      setpointTopic.unsubscribe(onSetpoint);
      targetTopic.unsubscribe(onTarget);
      waypointsTopic.unsubscribe(onWaypoints);
    };
  }, []);

  return {
    setpoint: setpoint?.value,
    setpointStale: !setpoint || now - setpoint.at > SETPOINT_STALE_MS,
    target: target?.value,
    targetStale: !target || now - target.at > TARGET_STALE_MS,
    waypoints,
  };
}
