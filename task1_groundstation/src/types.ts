/**
 * ROS message shapes as delivered by rosbridge (roslibjs), mirroring
 * airside/src/airside_interfaces/msg/TriggeredImageCapture.msg and its
 * field types. Field names match the .msg files exactly (snake_case).
 */

export interface RosImage {
  header: { stamp: { sec: number; nanosec: number }; frame_id: string };
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  /** base64-encoded raw pixel bytes, as rosbridge encodes uint8[] fields. */
  data: string;
}

export interface RosCoordinate {
  lat: number;
  lon: number;
  alt: number;
}

export interface RosImu {
  orientation: { x: number; y: number; z: number; w: number };
}

export interface RosRange {
  /** metres */
  range: number;
  min_range: number;
  max_range: number;
}

export interface TriggeredImageCapture {
  header: { stamp: { sec: number; nanosec: number }; frame_id: string };
  forward_image: RosImage;
  downward_image: RosImage;
  location: RosCoordinate;
  imu: RosImu;
  range: RosRange;
}

/** One triggered photo pair, logged for the crew to review while counting deer. */
export type Capture = {
  id: string;
  time: string;
  forwardImageUrl: string;
  downwardImageUrl: string;
};
