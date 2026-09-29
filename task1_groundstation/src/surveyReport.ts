export const TEAM_NAME = 'WARG';
export const SURVEY_FILENAME = `${TEAM_NAME}_task1_survey.txt`;

function joinWithAnd(items: string[]): string {
  if (items.length === 0) return '';
  if (items.length === 1) return items[0];
  if (items.length === 2) return `${items[0]} and ${items[1]}`;
  return `${items.slice(0, -1).join(', ')}, and ${items[items.length - 1]}`;
}

/**
 * Builds the Task 1 survey report text per the AEAC CONOPS §5.2.3 example:
 * "8 deer total in clusters of 2, 2, 3, and 1. ID tags: H7, K4, and 3P.
 * One deer is wearing a hat."
 *
 * Total deer count is derived from the cluster sizes (every deer belongs to
 * exactly one cluster per the spec's 10m-radius definition, solo deer being
 * a cluster of one), so there is one source of truth instead of two numbers
 * that could disagree.
 */
export function buildSurveyText(clusters: number[], tags: string[], description: string): string {
  const total = clusters.reduce((sum, n) => sum + n, 0);

  let text: string;
  if (clusters.length === 0) {
    text = '0 deer total.';
  } else if (clusters.length === 1) {
    text = `${total} deer total in a cluster of ${clusters[0]}.`;
  } else {
    text = `${total} deer total in clusters of ${joinWithAnd(clusters.map(String))}.`;
  }

  if (tags.length > 0) {
    text += ` ID tags: ${joinWithAnd(tags)}.`;
  }

  const desc = description.trim();
  if (desc) {
    text += ` ${/[.!?]$/.test(desc) ? desc : `${desc}.`}`;
  }

  return text;
}
