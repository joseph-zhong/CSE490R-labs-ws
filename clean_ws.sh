#!/bin/bash
#
# clean_ws.sh
# ---
#
#   Cleans the workspace by deleting build/ and devel/
#

echo "Cleaning workspace..."
if [ -d build ]; then
  rm -r build
fi
if [ -d devel ]; then
  rm -r devel
fi
echo "Done!"

