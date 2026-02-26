#!/usr/bin/env bash
# fetch_tests.sh — download the ps1-tests release zip from GitHub
#
# Usage:
#   tests/fetch_tests.sh [--dest DIR]
#
# Default destination: tests/ps1-tests  (relative to the repo root)
# The directory is created if it does not exist.
#
# Requires: curl (or wget as fallback) and unzip.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${PS1_TESTS_DEST:-$REPO_ROOT/tests/ps1-tests}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dest) DEST="$2"; shift 2 ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

URL="https://github.com/JaCzekanski/ps1-tests/releases/latest/download/tests.zip"
TMP_ZIP="$(mktemp /tmp/ps1-tests-XXXXXX.zip)"

echo "Downloading ps1-tests from GitHub..."

if command -v curl &>/dev/null; then
    # -L: follow redirects; print the final URL so we can extract the version tag
    FINAL_URL=$(curl -L -o "$TMP_ZIP" -w "%{url_effective}" -s "$URL")
else
    wget -q -O "$TMP_ZIP" "$URL"
    FINAL_URL="$URL"
fi

# Extract the version tag from the redirect URL (…/download/vX.Y.Z/ps1-tests.zip)
VERSION=$(echo "$FINAL_URL" | grep -oP '(?<=/download/)[^/]+(?=/)' || echo "unknown")
echo "Version: $VERSION"

mkdir -p "$DEST"
echo "Unpacking into '$DEST'..."
# The zip may contain a top-level directory or not; unzip directly into $DEST
# and flatten any single top-level dir with the --strip flag if using unzip.
TMP_DIR="$(mktemp -d /tmp/ps1-tests-unzip-XXXXXX)"
unzip -q "$TMP_ZIP" -d "$TMP_DIR"
rm -f "$TMP_ZIP"

# Move contents to $DEST (handle optional single top-level wrapper directory)
ITEMS=("$TMP_DIR"/*)
if [[ ${#ITEMS[@]} -eq 1 && -d "${ITEMS[0]}" ]]; then
    # Single top-level directory — move its contents
    mv "${ITEMS[0]}"/* "$DEST"/
else
    mv "$TMP_DIR"/* "$DEST"/
fi
rm -rf "$TMP_DIR"

EXE_COUNT=$(find "$DEST" -name "*.exe" | wc -l)
echo "Done. $EXE_COUNT test executables available in '$DEST'."
