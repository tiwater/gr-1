#!/bin/bash
set -e

# Start the service
service rocs-svr start

# Launch webots
eval "$WEBOTS"