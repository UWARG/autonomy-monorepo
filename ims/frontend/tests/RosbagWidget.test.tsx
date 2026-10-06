import { act, cleanup, fireEvent, render, screen } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import RosbagWidget from '../src/widgets/RosbagWidget';
import type { RosbagClient, RosbagState } from '../src/rosbagClient';

// The shared ROS module otherwise opens a real connection at import time.
vi.mock('../src/ros.js', () => ({ ros: {} }));

class Backend implements RosbagClient {
  state: RosbagState = { connected: true, status: { recording: false, bag_path: null, error: null }, error: null };
  listener?: (state: RosbagState) => void;
  requests: boolean[] = [];
  finish?: () => void;
  fail?: (reason: Error) => void;

  subscribe(listener: (state: RosbagState) => void) {
    this.listener = listener;
    listener(this.state);
    return () => { this.listener = undefined; };
  }

  setRecording(active: boolean) {
    this.requests.push(active);
    return new Promise<void>((resolve, reject) => {
      this.finish = resolve;
      this.fail = reject;
    });
  }

  emit(state: RosbagState) {
    this.state = state;
    this.listener?.(state);
  }
}

afterEach(cleanup);

describe('RosbagWidget', () => {
  it('waits for confirmation before changing Start to Stop and prevents repeated requests', async () => {
    const backend = new Backend();
    render(<RosbagWidget client={backend} />);
    fireEvent.click(screen.getByRole('button', { name: 'Start Recording' }));
    const starting = screen.getByRole('button', { name: 'Starting…' });
    expect((starting as HTMLButtonElement).disabled).toBe(true);
    fireEvent.click(starting);
    expect(backend.requests).toEqual([true]);
    await act(async () => {
      backend.emit({ connected: true, status: { recording: true, bag_path: '/bags/one', error: null }, error: null });
      backend.finish?.();
    });
    fireEvent.click(screen.getByRole('button', { name: 'Stop Recording' }));
    expect(backend.requests).toEqual([true, false]);
    await act(async () => {
      backend.emit({ connected: true, status: { recording: false, bag_path: '/bags/one', error: null }, error: null });
      backend.finish?.();
    });
    expect(screen.getByRole('button', { name: 'Start Recording' })).toBeTruthy();
  });

  it('shows failed commands without claiming recording started', async () => {
    const backend = new Backend();
    render(<RosbagWidget client={backend} />);
    fireEvent.click(screen.getByRole('button', { name: 'Start Recording' }));
    await act(async () => { backend.fail?.(new Error('Disk full')); });
    expect(screen.getByRole('alert').textContent).toContain('Disk full');
    expect(screen.getByRole('button', { name: 'Start Recording' })).toBeTruthy();
  });

  it('uses backend state after reload and disables commands on disconnect', () => {
    const backend = new Backend();
    backend.state.status = { recording: true, bag_path: '/bags/existing', error: null };
    render(<RosbagWidget client={backend} />);
    expect(screen.getByRole('button', { name: 'Stop Recording' })).toBeTruthy();
    act(() => backend.emit({ connected: false, status: null, error: null }));
    expect((screen.getByRole('button') as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByText('ROS disconnected')).toBeTruthy();
  });

  it('keeps the button disabled until the controller reports its state', () => {
    const backend = new Backend();
    backend.state.status = null;
    render(<RosbagWidget client={backend} />);
    expect((screen.getByRole('button') as HTMLButtonElement).disabled).toBe(true);
  });
});
