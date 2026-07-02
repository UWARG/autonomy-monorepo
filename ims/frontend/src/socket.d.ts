export function subscribe(type: string, callback: (payload: unknown) => void): void
export function unsubscribe(type: string, callback: (payload: unknown) => void): void
export function send(type: string, payload?: unknown): void
