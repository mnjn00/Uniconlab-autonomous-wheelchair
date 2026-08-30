# Shared perception defaults for bring-up and motion authorization.
# Explicit environment variables always override these profile defaults.

PERCEPTION_PROFILE="${PERCEPTION_PROFILE:-legacy_geometric}"
case "$PERCEPTION_PROFILE" in
  legacy_geometric)
    : "${START_POINTPILLARS:=false}"
    : "${GEOMETRIC_FIXED_MAP_SUBTRACTION:=false}"
    : "${GEOMETRIC_MIN_CELL_POINTS:=2}"
    : "${GEOMETRIC_MIN_CLUSTER_POINTS:=8}"
    : "${GEOMETRIC_MAX_CLUSTERS:=40}"
    # Keep an obstacle visible while the chair draws level with it. The
    # rider exclusion box, rather than this forward ROI, removes chair/rider
    # returns.
    : "${GEOMETRIC_ROI_X_MIN_M:=-0.30}"
    : "${GEOMETRIC_FORWARD_FOV_HALF_DEG:=115}"
    ;;
  hybrid_experimental)
    : "${START_POINTPILLARS:=true}"
    : "${GEOMETRIC_FIXED_MAP_SUBTRACTION:=false}"
    : "${GEOMETRIC_MIN_CELL_POINTS:=1}"
    : "${GEOMETRIC_MIN_CLUSTER_POINTS:=5}"
    : "${GEOMETRIC_MAX_CLUSTERS:=80}"
    : "${GEOMETRIC_ROI_X_MIN_M:=0.50}"
    : "${GEOMETRIC_FORWARD_FOV_HALF_DEG:=50}"
    ;;
  *)
    echo "ERROR: PERCEPTION_PROFILE must be legacy_geometric or hybrid_experimental" >&2
    exit 64
    ;;
esac

export PERCEPTION_PROFILE START_POINTPILLARS GEOMETRIC_FIXED_MAP_SUBTRACTION
export GEOMETRIC_MIN_CELL_POINTS GEOMETRIC_MIN_CLUSTER_POINTS GEOMETRIC_MAX_CLUSTERS
export GEOMETRIC_ROI_X_MIN_M GEOMETRIC_FORWARD_FOV_HALF_DEG
