#!/usr/bin/env bash
# Deploy the reviewed branch to the NUC and run non-driving shadow QA.
set -euo pipefail

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  cat <<'EOF'
Usage: deploy_and_run_nuc_shadow_qa.sh <nuc-host> <commit> <output-directory>

The host must present the pinned NUC ED25519 key. The script updates only
origin/relax/tracking-thresholds, exports the exact reviewed commit without
changing the NUC source checkout, copies its package into the catkin workspace,
and runs sensor-only QA. It never starts a wheel or route follower.
EOF
  exit 0
fi
[ "$#" -eq 3 ] || {
  echo "ERROR: expected NUC host, commit, and output directory; use --help" >&2
  exit 64
}

HOST="$1"
EXPECTED_COMMIT="$2"
OUT="$3"
PINNED_KEY="SHA256:mCyENhdOemXE2/aNUH+6nECgiaaAabBHsGfsHke5hg8"
IDENTITY="${IDENTITY:-$HOME/.ssh/codex_mprp3_10_73_61_199}"
[[ "$EXPECTED_COMMIT" =~ ^[0-9a-f]{7,40}$ ]] || {
  echo "ERROR: commit must be a 7-40 digit lowercase Git SHA" >&2
  exit 64
}
KNOWN_HOSTS="$(mktemp)"
trap 'rm -f "$KNOWN_HOSTS"' EXIT
ssh-keyscan -T 5 -t ed25519 "$HOST" > "$KNOWN_HOSTS" 2>/dev/null
SSH=(ssh -i "$IDENTITY" -o BatchMode=yes -o ConnectTimeout=8
     -o StrictHostKeyChecking=yes -o UserKnownHostsFile="$KNOWN_HOSTS"
     "mprp3@$HOST")
SCP=(scp -i "$IDENTITY" -o BatchMode=yes -o ConnectTimeout=8
     -o StrictHostKeyChecking=yes -o UserKnownHostsFile="$KNOWN_HOSTS")

observed="$(
  ssh-keygen -lf "$KNOWN_HOSTS" 2>/dev/null |
    awk '{print $2}'
)"
[ "$observed" = "$PINNED_KEY" ] || {
  echo "ERROR: NUC host-key mismatch for $HOST: ${observed:-missing}" >&2
  exit 65
}

"${SSH[@]}" 'bash -s' <<EOF
set -euo pipefail
REPO="\$HOME/wheelchair_localization_src"
WS="\$HOME/livox_static_localization_ws"
DEPLOY="\$(mktemp -d)"
cleanup_deploy() {
  rm -rf "\$DEPLOY"
}
trap cleanup_deploy EXIT
AUTONOMOUS_RE='[w]heel_cmd|[w]heel\\.launch|[w]aypoint_follower|[d]wa_follower|[m]pc_follower|[s]afety_gate|[t]ip_guard'
if pgrep -af "\$AUTONOMOUS_RE"; then
  echo "ERROR: autonomous process present before deployment" >&2
  exit 20
fi
cd "\$REPO"
git fetch origin relax/tracking-thresholds
test "\$(git rev-parse "$EXPECTED_COMMIT")" = \
  "\$(git rev-parse origin/relax/tracking-thresholds)"
git archive "$EXPECTED_COMMIT" | tar -x -C "\$DEPLOY"
rsync -a --delete \
  "\$DEPLOY/src/static_livox_localization/" \
  "\$WS/src/static_livox_localization/"
OUT="\$HOME/nuc-shadow-qa-$EXPECTED_COMMIT"
rm -rf "\$OUT"
REPO="\$DEPLOY" WS="\$WS" OUT="\$OUT" \
  "\$DEPLOY/tools/run_nuc_shadow_qa.sh"
tar -C "\$HOME" -czf "\$HOME/nuc-shadow-qa-$EXPECTED_COMMIT.tgz" \
  "nuc-shadow-qa-$EXPECTED_COMMIT"
EOF

mkdir -p "$OUT"
"${SCP[@]}" \
  "mprp3@$HOST:nuc-shadow-qa-$EXPECTED_COMMIT.tgz" \
  "$OUT/"
tar -C "$OUT" -xzf "$OUT/nuc-shadow-qa-$EXPECTED_COMMIT.tgz"
