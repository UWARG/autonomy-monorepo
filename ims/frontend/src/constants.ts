export const ROS_URL = `ws://${window.location.hostname}:9090`;
export const STALE_AFTER_MS = 5000;
export const RECONNECT_DELAY_MS = 3000;
export const MAX_RECONNECT_ATTEMPTS = 10;

// IMS subscribes to AEAC traffic on its own receive-only connection. Put the
// team token in ims/frontend/.env.local (gitignored) as VITE_AEAC_CONNECTION_TOKEN.
export const AEAC_WEBSOCKET_URL: string =
  import.meta.env.VITE_AEAC_WEBSOCKET_URL || 'wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod';
export const AEAC_TOKEN: string = import.meta.env.VITE_AEAC_CONNECTION_TOKEN ?? '';
