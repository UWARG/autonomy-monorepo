// Type declarations for socket.js
export function subscribe<T = unknown>(type: string, callback: (payload: T) => void): void;
export function unsubscribe<T = unknown>(type: string, callback: (payload: T) => void): void;