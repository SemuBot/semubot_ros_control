"""Run: python3 test/test_motor_fault_diagnostics.py (with ROS sourced)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from motor_fault_diagnostics import decode  # noqa: E402

# All clear
assert [s[1] for s in decode([0] * 3)] == [0, 0, 0]

# m2: FAULT + VDS_OCP; m3: UVLO
d = decode([0, (1 << 10) | (1 << 9), 1 << 7])
assert d[0][1] == 0 and d[0][2] == 'OK'
assert d[1][1] == 2 and d[1][2] == 'FAULT, VDS_OCP'
assert d[2][1] == 2 and d[2][2] == 'UVLO'
print('ok')
