import { number, object, text } from "./models";
export interface Frame {
  width?: number;
  height?: number;
  pixels?: Uint8ClampedArray<ArrayBuffer>;
  src?: string;
}
function bytes(data: unknown): Uint8Array {
  if (typeof data === "string") {
    if (data.length > 32_000_000) throw Error("Image exceeds limit");
    return Uint8Array.from(atob(data), (c) => c.charCodeAt(0));
  }
  if (
    !Array.isArray(data) ||
    data.length > 24_000_000 ||
    !data.every((n) => Number.isInteger(n) && n >= 0 && n <= 255)
  )
    throw Error("Invalid image bytes");
  return Uint8Array.from(data);
}
export function parseImage(value: unknown): Frame {
  const v = object(value);
  if ("format" in v) {
    const format = text(v.format).toLowerCase();
    const mime = format.includes("compresseddepth")
      ? undefined
      : format.includes("jpeg") || format.includes("jpg")
        ? "image/jpeg"
        : format.includes("png")
          ? "image/png"
          : undefined;
    if (!mime) throw Error(`Unsupported compressed format: ${format}`);
    const data = bytes(v.data);
    let binary = "";
    for (let i = 0; i < data.length; i += 8192)
      binary += String.fromCharCode(...data.subarray(i, i + 8192));
    return { src: `data:${mime};base64,${btoa(binary)}` };
  }
  const width = number(v.width),
    height = number(v.height),
    step = number(v.step),
    encoding = text(v.encoding);
  const channels: Record<string, number> = {
    rgb8: 3,
    bgr8: 3,
    rgba8: 4,
    bgra8: 4,
    mono8: 1,
  };
  const count = Object.hasOwn(channels, encoding)
    ? channels[encoding]
    : undefined;
  if (!count) throw Error(`Unsupported encoding: ${encoding}`);
  if (
    !Number.isInteger(width) ||
    !Number.isInteger(height) ||
    !Number.isInteger(step) ||
    width <= 0 ||
    height <= 0 ||
    width * height > 4_000_000 ||
    step < width * count ||
    step * height > 24_000_000
  )
    throw Error("Invalid image dimensions/stride");
  const data = bytes(v.data);
  if (data.length < step * height) throw Error("Truncated image");
  const pixels = new Uint8ClampedArray(width * height * 4);
  for (let y = 0; y < height; y++)
    for (let x = 0; x < width; x++) {
      const s = y * step + x * count,
        d = (y * width + x) * 4,
        bgr = encoding.startsWith("bgr");
      pixels[d] = data[s + (bgr ? 2 : 0)];
      pixels[d + 1] = data[s + (count === 1 ? 0 : 1)];
      pixels[d + 2] = data[s + (count === 1 ? 0 : bgr ? 0 : 2)];
      pixels[d + 3] = count === 4 ? data[s + 3] : 255;
    }
  return { width, height, pixels };
}
