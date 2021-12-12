#!/bin/sh
# Run from source directory, e.g. ./resources/tu_version.sh "1.0.0"

if [ -z "$1" ]; then
	echo "Please specify version string"
	exit 1
fi

cat > TU_version.h <<EOF
// NOTE: DO NOT INCLUDE DIRECTLY, USE TU::Strings::VERSION
"$1"
EOF
