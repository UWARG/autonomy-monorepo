export type TaskState = 'RUNNING' | 'SUCCESS' | 'FAILURE' | 'INVALID';

export interface StatusPayload {
  task: string;
  state: TaskState;
  text: string;
}
