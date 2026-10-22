#!/usr/bin/env bash
set -euo pipefail
handoff_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="On-aroll/rebot-arm-b601"
tag="handoff-2026-10-22"
gh auth status --hostname github.com
cd "$handoff_root/release-assets"
sha256sum -c SHA256SUMS > /dev/null
if ! gh release view "$tag" --repo "$repo" > /dev/null 2>&1; then
  gh release create "$tag" --repo "$repo" --verify-tag --title 'B601-RS 2026-10-22 paused cross-device handoff' --notes-file "$handoff_root/RELEASE_NOTES.md"
fi
# No --clobber: preserve any previously uploaded bytes; resume by name and size.
python3 "$handoff_root/upload_assets.py"
