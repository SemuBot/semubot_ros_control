"""Run: python3 test/test_motor_fault_diagnostics.py (with ROS sourced)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from motor_fault_diagnostics import decode  # noqa: E402

# All clear
assert [s[1] for s in decode([0] * 7)] == [0, 0, 0, 0]

# m2: FAULT + VDS_OCP; m3: OTW only; SPI error on m1; command timeout
d = decode([0, 0, (1 << 10) | (1 << 9), 0, 0, 1 << 7, 0b0011])
assert d[0][1] == 2 and 'SPI_ERR' in d[0][2]
assert d[1][1] == 2 and d[1][2] == 'FAULT, VDS_OCP'
assert d[2][1] == 1 and d[2][2] == 'OTW'
assert d[3][1] == 1 and d[3][2].startswith('CMD_TIMEOUT')
print('ok')
