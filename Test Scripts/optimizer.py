from scipy.optimize import differential_evolution
import os
import sys
from decimal import Decimal
import math

if os.name == 'posix' and sys.version_info[0] < 3:
    import subprocess32 as subprocess
else:
    import subprocess

bounds = [(0.0, 20.0), (0.0, 20.0), (0.0, 2.0)]

def f(z):
    alt, accel, model = z
    cmd = './rkal32_mc test_data.csv results.csv ' + str(alt) + ' ' + str(accel) + ' ' + str(model)

    print cmd

    p = subprocess.Popen(cmd,
                         shell=True,
                         stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT,
                         close_fds=True)
    output = p.stdout.read()
    print float(output)

    if math.isnan(float(output)):
        return float(1000)
    else:
        return float(output)

resbrute = differential_evolution(f, bounds)

print(resbrute)
