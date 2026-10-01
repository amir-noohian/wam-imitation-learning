#!/bin/bash

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export BARRETT_CONFIG_FILE="$PACKAGE_DIR/config/zeus-7dof/zeus.conf"
