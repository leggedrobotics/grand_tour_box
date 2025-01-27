# Exit if a single command fails
set -e

# Export necessary environment variables

# Token for creating github issues from inside the container.
export GITHUB_TOKEN=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

# UUID for the GrandTour Project. Overwrite this for testing in dev projects.
export PROJECT_UUID=3c97a27e-4180-4e40-b8af-59714de54a87