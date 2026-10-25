# Source this file: `source env.sh`
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64}"
export BLENDER="${BLENDER:-$HOME/Downloads/blender-5.2.0-linux-x64/blender}"
export SITE_CONFIG="${SITE_CONFIG:-$PROJECT_ROOT/config/site.yaml}"
export BUILD_DIR="$PROJECT_ROOT/build"
