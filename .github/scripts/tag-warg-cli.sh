#!/usr/bin/env bash
# Creates the warg-cli release tag from pyproject.toml (if it doesn't exist yet)
# and starts the publish workflow for that tag.
set -euo pipefail

PYPROJECT="warg_cli/pyproject.toml"
PUBLISH_WORKFLOW="publish-warg-cli.yml"

# 1. Read the version from pyproject.toml
version="$(python - "${PYPROJECT}" <<'PY'
import sys
import tomllib

with open(sys.argv[1], "rb") as file:
    print(tomllib.load(file)["project"]["version"])
PY
)"
tag="warg-cli/v${version}"

# 2. Stop early if the tag already exists
if git ls-remote --exit-code --tags origin "refs/tags/${tag}" > /dev/null 2>&1; then
  echo "Tag ${tag} already exists. Nothing to do."
  exit 0
fi

# 3. Create and push the tag
git tag "${tag}"
git push origin "${tag}"
echo "Created and pushed ${tag}."

# 4. Start the publish workflow on the new tag
if ! gh workflow run "${PUBLISH_WORKFLOW}" --repo "${GITHUB_REPOSITORY}" --ref "${tag}"; then
  echo "::error::Tag ${tag} was created, but starting ${PUBLISH_WORKFLOW} failed. Run that workflow manually on ${tag} (Actions > Publish warg-cli > Run workflow) to publish."
  exit 1
fi
echo "Started ${PUBLISH_WORKFLOW} for ${tag}."
