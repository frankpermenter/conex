#!/usr/bin/env python3
"""Compare conex vs Clarabel on small QPS (Maros-Meszaros) instances.

Reads QPS files, converts to Clarabel's standard form, solves with both
solvers, and reports objective/time/iterations.
"""
import subprocess, time, os, sys, re
import numpy as np
from scipy.sparse import csc_matrix, eye, vstack

try:
    import clarabel
except ImportError:
    print("pip install clarabel"); sys.exit(1)

QPS_DIR = "/agent-workspace/problem_libraries/maros_meszaros/QPS_Files"
CONEX_BIN = "/agent-workspace/conex/build/benchmark_solver"


def parse_qps(filepath):
    """Minimal QPS parser. Returns Q, c, A, b, bounds, eq_rows."""
    section = None
    obj_row = None
    row_names = []
    row_types = {}
    col_names = []
    col_idx = {}

    A_entries = {}  # (row, col) -> val
    Q_entries = {}  # (col1, col2) -> val
    rhs = {}        # row -> val
    bounds_lo = {}  # col -> val
    bounds_up = {}  # col -> val

    for line in open(filepath):
        line = line.rstrip('\n')
        if line.startswith("NAME"):
            continue
        if line.startswith("ROWS"):
            section = "ROWS"; continue
        if line.startswith("COLUMNS"):
            section = "COLUMNS"; continue
        if line.startswith("RHS"):
            section = "RHS"; continue
        if line.startswith("RANGES"):
            section = "RANGES"; continue
        if line.startswith("BOUNDS"):
            section = "BOUNDS"; continue
        if line.startswith("QUADOBJ"):
            section = "QUADOBJ"; continue
        if line.startswith("ENDATA"):
            break
        if not line.strip():
            continue

        parts = line.split()

        if section == "ROWS":
            rtype, rname = parts[0], parts[1]
            row_types[rname] = rtype
            if rtype == 'N':
                obj_row = rname
            else:
                row_names.append(rname)

        elif section == "COLUMNS":
            col = parts[0]
            if col not in col_idx:
                col_idx[col] = len(col_names)
                col_names.append(col)
            ci = col_idx[col]
            i = 1
            while i + 1 < len(parts):
                rname, val = parts[i], float(parts[i+1])
                A_entries[(rname, ci)] = A_entries.get((rname, ci), 0) + val
                i += 2

        elif section == "RHS":
            i = 1
            while i + 1 < len(parts):
                rname, val = parts[i], float(parts[i+1])
                rhs[rname] = val
                i += 2

        elif section == "BOUNDS":
            btype = parts[0]
            col = parts[2] if len(parts) > 2 else parts[1]
            ci = col_idx.get(col)
            if ci is None:
                continue
            if btype == "UP" and len(parts) > 3:
                bounds_up[ci] = float(parts[3])
            elif btype == "LO" and len(parts) > 3:
                bounds_lo[ci] = float(parts[3])
            elif btype == "FX" and len(parts) > 3:
                bounds_lo[ci] = float(parts[3])
                bounds_up[ci] = float(parts[3])
            elif btype == "FR":
                bounds_lo[ci] = -1e20
                bounds_up[ci] = 1e20
            elif btype == "MI":
                bounds_lo[ci] = -1e20
            elif btype == "PL":
                bounds_up[ci] = 1e20

        elif section == "QUADOBJ":
            c1, c2, val = parts[0], parts[1], float(parts[2])
            ci1, ci2 = col_idx[c1], col_idx[c2]
            Q_entries[(min(ci1,ci2), max(ci1,ci2))] = \
                Q_entries.get((min(ci1,ci2), max(ci1,ci2)), 0) + float(val)

    n = len(col_names)
    m = len(row_names)

    # Build c (objective)
    c = np.zeros(n)
    for (rname, ci), val in A_entries.items():
        if rname == obj_row:
            c[ci] = val

    # Build A, b
    row_map = {rname: i for i, rname in enumerate(row_names)}
    rows, cols, vals = [], [], []
    for (rname, ci), val in A_entries.items():
        if rname in row_map:
            rows.append(row_map[rname])
            cols.append(ci)
            vals.append(val)
    A_sp = csc_matrix((vals, (rows, cols)), shape=(m, n))

    b = np.zeros(m)
    for rname, val in rhs.items():
        if rname in row_map:
            b[row_map[rname]] = val

    # Build Q (upper triangular, QPS stores lower triangle values)
    Qrows, Qcols, Qvals = [], [], []
    for (ci1, ci2), val in Q_entries.items():
        Qrows.append(ci1); Qcols.append(ci2); Qvals.append(val)
        if ci1 != ci2:
            Qrows.append(ci2); Qcols.append(ci1); Qvals.append(val)
    Q_sp = csc_matrix((Qvals, (Qrows, Qcols)), shape=(n, n)) if Qvals else csc_matrix((n, n))

    # Row types for inequality/equality
    eq_rows = [i for i, rn in enumerate(row_names) if row_types[rn] == 'E']
    ge_rows = [i for i, rn in enumerate(row_names) if row_types[rn] == 'G']
    le_rows = [i for i, rn in enumerate(row_names) if row_types[rn] == 'L']

    return {
        'n': n, 'm': m, 'Q': Q_sp, 'c': c, 'A': A_sp, 'b': b,
        'eq_rows': eq_rows, 'ge_rows': ge_rows, 'le_rows': le_rows,
        'bounds_lo': bounds_lo, 'bounds_up': bounds_up,
        'col_names': col_names, 'row_names': row_names
    }


def solve_clarabel(prob):
    """Convert to Clarabel standard form and solve."""
    n, m = prob['n'], prob['m']
    Q, c, A, b = prob['Q'], prob['c'], prob['A'], prob['b']

    # Clarabel: min 0.5 x'Px + q'x s.t. Ax + s = b, s in K
    # We need: equality rows -> ZeroCone, inequality rows -> NonnegativeCone

    # Build constraint matrix and cones
    rows_list = []
    b_list = []
    cones = []

    # Equality constraints
    if prob['eq_rows']:
        idx = prob['eq_rows']
        rows_list.append(A[idx, :])
        b_list.append(b[idx])
        cones.append(clarabel.ZeroConeT(len(idx)))

    # GE constraints: Ax >= b -> -Ax + s = -b, s >= 0
    if prob['ge_rows']:
        idx = prob['ge_rows']
        rows_list.append(-A[idx, :])
        b_list.append(-b[idx])
        cones.append(clarabel.NonnegativeConeT(len(idx)))

    # LE constraints: Ax <= b -> Ax + s = b, s >= 0
    if prob['le_rows']:
        idx = prob['le_rows']
        rows_list.append(A[idx, :])
        b_list.append(b[idx])
        cones.append(clarabel.NonnegativeConeT(len(idx)))

    # Bounds: default 0 <= x <= inf (QPS convention)
    for i in range(n):
        lo = prob['bounds_lo'].get(i, 0.0)
        up = prob['bounds_up'].get(i, 1e20)
        if lo > -1e19:
            # x_i >= lo -> -x_i + s = -lo, s >= 0
            row = csc_matrix(([-1.0], ([0], [i])), shape=(1, n))
            rows_list.append(row)
            b_list.append(np.array([-lo]))
            cones.append(clarabel.NonnegativeConeT(1))
        if up < 1e19:
            # x_i <= up -> x_i + s = up, s >= 0
            row = csc_matrix(([1.0], ([0], [i])), shape=(1, n))
            rows_list.append(row)
            b_list.append(np.array([up]))
            cones.append(clarabel.NonnegativeConeT(1))

    if not rows_list:
        return None

    A_cl = vstack(rows_list, format='csc')
    b_cl = np.concatenate(b_list)

    settings = clarabel.DefaultSettings()
    settings.verbose = False
    settings.time_limit = 10.0

    # Clarabel needs upper-triangular P in CSC format.
    from scipy.sparse import triu
    P = triu(Q, format='csc')

    try:
        t0 = time.time()
        solver = clarabel.DefaultSolver(P, c, A_cl, b_cl, cones, settings)
        sol = solver.solve()
        elapsed = time.time() - t0
        x = np.array(sol.x) if sol.x is not None else None
        if x is not None:
            obj = 0.5 * x @ Q.toarray() @ x + c @ x
        else:
            obj = float('inf')
        return {
            'time': elapsed,
            'obj': obj,
            'status': str(sol.status),
            'iterations': sol.iterations
        }
    except Exception as e:
        return {'time': 0, 'obj': float('inf'), 'status': f'error: {e}', 'iterations': 0}


def run_conex(qps_path):
    """Run conex benchmark_solver on a QPS file."""
    if not os.path.exists(CONEX_BIN):
        return None
    try:
        t0 = time.time()
        result = subprocess.run(
            [CONEX_BIN, qps_path],
            capture_output=True, text=True, timeout=30
        )
        elapsed = time.time() - t0
        output = result.stdout + result.stderr

        # Parse output: look for algorithm summary lines and constant offset.
        obj = float('inf')
        fac = 0
        c0 = 0
        for line in output.split('\n'):
            # Algorithm summary: "name  iters  fac  solves  mu  obj  ..."
            # The obj column is the 6th field (0-indexed: 5).
            parts = line.split()
            if len(parts) >= 7 and 'TR_t' in parts[0]:
                try:
                    fac = int(parts[2])
                    obj = float(parts[5])
                except (ValueError, IndexError):
                    pass
            # Constant offset
            if 'Objective includes constant' in line:
                nums = re.findall(r'[-+]?\d*\.?\d+[eE][-+]?\d+', line)
                if nums:
                    c0 = float(nums[0])

        if obj != float('inf'):
            obj += c0

        return {'time': elapsed, 'obj': obj, 'fac': fac, 'rc': result.returncode}
    except subprocess.TimeoutExpired:
        return {'time': 30.0, 'obj': float('inf'), 'fac': 0, 'rc': -1}


# Find small QPS files
problems = []
for f in sorted(os.listdir(QPS_DIR)):
    if not f.endswith(".QPS"):
        continue
    path = os.path.join(QPS_DIR, f)
    try:
        prob = parse_qps(path)
        if prob['n'] <= 50:
            problems.append((f, path, prob))
    except Exception as e:
        pass

print(f"Comparing conex vs Clarabel on {len(problems)} small QPS instances (n ≤ 50)\n")
print(f"{'Problem':<18} {'n':>3} {'m':>3} | {'Clarabel':>10} {'iter':>4} {'obj':>12} | {'Conex':>10} {'fac':>4} {'obj':>12} | {'speedup':>7}")
print("-" * 100)

for name, path, prob in problems:
    cl = solve_clarabel(prob)
    cx = run_conex(path)

    cl_time = cl['time'] if cl else 0
    cl_iter = cl.get('iterations', 0) if cl else 0
    cl_obj = cl.get('obj', float('inf')) if cl else float('inf')
    cl_status = cl.get('status', '') if cl else ''

    cx_time = cx['time'] if cx else 0
    cx_fac = cx.get('fac', 0) if cx else 0
    cx_obj = cx.get('obj', float('inf')) if cx else float('inf')

    speedup = cl_time / cx_time if cx_time > 0 else 0

    print(f"{name:<18} {prob['n']:3d} {prob['m']:3d} | "
          f"{cl_time:9.4f}s {cl_iter:4d} {cl_obj:12.4f} | "
          f"{cx_time:9.4f}s {cx_fac:4d} {cx_obj:12.4f} | "
          f"{speedup:6.2f}x")
