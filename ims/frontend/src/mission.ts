import type { MissionStatus } from './types';

/** The mission's steps in flight order, as the crew thinks of them. */
export const MISSION_STEPS = ['Arm', 'Takeoff', 'Laps', 'Recon', 'Land', 'Done'] as const;
export type MissionStep = (typeof MISSION_STEPS)[number];

/** Engine phase (a child of the Mission sequence in engine/manager.py) -> step. */
const PHASE_STEP: Record<string, MissionStep> = {
  RecordLaunchPoint: 'Arm',
  LoadWaypointList: 'Arm',
  Takeoff: 'Takeoff',
  Lapping: 'Laps',
  TargetReconnaissance: 'Recon',
  LandPhase: 'Land',
  MissionComplete: 'Done',
};

/** Index into MISSION_STEPS, or -1 before the mission starts (or for an unknown phase). */
export function stepIndex(status?: MissionStatus): number {
  const step = status ? PHASE_STEP[status.phase] : undefined;
  return step ? MISSION_STEPS.indexOf(step) : -1;
}

/** Short progress label for the header, e.g. "LAP 2/3 · WP 1/3" or "RECON". */
export function missionSummary(status?: MissionStatus): string | null {
  if (!status) return null;
  if (status.state === 'ABORTED') return 'ABORTED';
  if (status.state === 'STARTING') return 'STARTING';
  if (status.phase === 'Lapping' && status.lap) {
    const wp = status.waypoint ? ` · WP ${status.waypoint}/${status.waypoint_count}` : '';
    return `LAP ${status.lap}/${status.lap_count}${wp}`;
  }
  const i = stepIndex(status);
  return i >= 0 ? MISSION_STEPS[i].toUpperCase() : status.phase.toUpperCase() || null;
}

/** "TargetReconnaissance" -> "Target reconnaissance". */
export function humanize(name: string): string {
  const words = name.replace(/([a-z])([A-Z])/g, '$1 $2').toLowerCase();
  return words.charAt(0).toUpperCase() + words.slice(1);
}
