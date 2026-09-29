import type { RosImage } from './types';

/**
 * Decodes a raw sensor_msgs/Image (as camera_node.py publishes: encoding
 * "rgb8", uint8 data) into a browser-displayable data URL, via an
 * off-DOM canvas. rosbridge base64-encodes the uint8[] `data` field.
 */
export function rosImageToDataUrl(image: RosImage): string {
  if (image.encoding !== 'rgb8') {
    console.warn(`rosImageToDataUrl: unsupported encoding '${image.encoding}', expected rgb8`);
  }

  const binary = atob(image.data);
  const rgb = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) {
    rgb[i] = binary.charCodeAt(i);
  }

  const rgba = new Uint8ClampedArray(image.width * image.height * 4);
  for (let px = 0; px < image.width * image.height; px++) {
    rgba[px * 4] = rgb[px * 3];
    rgba[px * 4 + 1] = rgb[px * 3 + 1];
    rgba[px * 4 + 2] = rgb[px * 3 + 2];
    rgba[px * 4 + 3] = 255;
  }

  const canvas = document.createElement('canvas');
  canvas.width = image.width;
  canvas.height = image.height;
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('rosImageToDataUrl: could not get 2d canvas context');
  ctx.putImageData(new ImageData(rgba, image.width, image.height), 0, 0);
  return canvas.toDataURL('image/png');
}
