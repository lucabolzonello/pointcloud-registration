#!/usr/bin/env bash

set -e

DATA_DIR="datasets"
URL="https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz"
ARCHIVE="$DATA_DIR/bunny.tar.gz"

echo "Creating data directory..."
mkdir -p "$DATA_DIR"

echo "Downloading Stanford Bunny dataset..."
if command -v curl >/dev/null 2>&1; then
    curl -L "$URL" -o "$ARCHIVE"
elif command -v wget >/dev/null 2>&1; then
    wget "$URL" -O "$ARCHIVE"
else
    echo "Error: curl or wget required."
    exit 1
fi

echo "Extracting..."
tar -xzf "$ARCHIVE" -C "$DATA_DIR"

echo "Cleaning up..."
rm "$ARCHIVE"

echo "Done! Dataset is in $DATA_DIR"