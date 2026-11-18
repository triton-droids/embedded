#!/usr/bin/env bash
# Runs on the host before each container start or attach. Copies the X auth
# cookie into a directory that devcontainer.json mounts, since XWayland's
# cookie file name changes every login. The directory (not the file) is
# mounted because xauth replaces the file, which a file bind mount would miss.
# The ffff family makes the cookie valid for any hostname.
# Never fails, so hosts without X or xauth still start the container.
dir="${HOME}/.cache/triton-devcontainer"
mkdir -p "$dir"
chmod 700 "$dir"
tmp="$(mktemp "$dir/xauth.XXXXXX")"
if command -v xauth >/dev/null && [[ -n "${DISPLAY:-}" ]]; then
  xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$tmp" nmerge - 2>/dev/null || true
fi
mv -f "$tmp" "$dir/xauth"
exit 0
