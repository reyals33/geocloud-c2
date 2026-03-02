#!/bin/bash
echo "=== GZ odom topic info ==="
gz topic -i -t /model/tb3_2/odom

echo ""
echo "=== GZ odom data (3s) ==="
timeout 3 gz topic -e -t /model/tb3_2/odom || echo "NO GZ ODOM DATA"

echo ""
echo "=== All GZ topics ==="
gz topic -l
