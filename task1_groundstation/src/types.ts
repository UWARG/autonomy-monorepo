/** rosbridge shapes of the CaptureImage.srv response; field names match the .msg files. */

export interface RosCompressedImage {
  header: { stamp: { sec: number; nanosec: number }; frame_id: string };
  format: string;
  /** base64-encoded JPEG bytes, as rosbridge encodes uint8[] fields. */
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
  image: RosCompressedImage;
  location: RosCoordinate;
  imu: RosImu;
}

/** One triggered photo, logged for the crew to review while counting deer. */
export type Capture = {
  id: string;
  time: string;
  imageUrl: string;
};
