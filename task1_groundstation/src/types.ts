/**
 * ROS message shapes as delivered by rosbridge (roslibjs), mirroring
 * airside/src/airside_interfaces/srv/CaptureImage.srv (response) and its
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

export interface CaptureImageResponse {
  success: boolean;
  message: string;
  header: { stamp: { sec: number; nanosec: number }; frame_id: string };
  image: RosImage;
  location: RosCoordinate;
  imu: RosImu;
}

/** One triggered photo, logged for the crew to review while counting deer. */
export type Capture = {
  id: string;
  time: string;
  imageUrl: string;
};
