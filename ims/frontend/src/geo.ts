export const R_EARTH_M = 6_371_000;
export const toRad = (d: number) => (d * Math.PI) / 180;
export const toDeg = (r: number) => (r * 180) / Math.PI;

export function haversineM(aLat: number, aLon: number, bLat: number, bLon: number): number {
  const dLat = toRad(bLat - aLat);
  const dLon = toRad(bLon - aLon);
  const s =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(aLat)) * Math.cos(toRad(bLat)) * Math.sin(dLon / 2) ** 2;
  return 2 * R_EARTH_M * Math.asin(Math.min(1, Math.sqrt(s)));
}

export function bearingDeg(aLat: number, aLon: number, bLat: number, bLon: number): number {
  const dLon = toRad(bLon - aLon);
  const y = Math.sin(dLon) * Math.cos(toRad(bLat));
  const x =
    Math.cos(toRad(aLat)) * Math.sin(toRad(bLat)) -
    Math.sin(toRad(aLat)) * Math.cos(toRad(bLat)) * Math.cos(dLon);
  return (toDeg(Math.atan2(y, x)) + 360) % 360;
}

export function enuOffsetM(
  originLat: number,
  originLon: number,
  lat: number,
  lon: number,
): { east: number; north: number } {
  const east = toRad(lon - originLon) * Math.cos(toRad(originLat)) * R_EARTH_M;
  const north = toRad(lat - originLat) * R_EARTH_M;
  return { east, north };
}
