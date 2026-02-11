import ctypes as ct
from pathlib import Path

import numpy as np


class intp:
    def __init__(self, value=0):
        self._value = ct.c_int(int(value))

    def value(self):
        return int(self._value.value)


class CONEX_SolverConfiguration(ct.Structure):
    _fields_ = [
        ("prepare_dual_variables", ct.c_int),
        ("initialization_mode", ct.c_int),
        ("inv_sqrt_mu_max", ct.c_double),
        ("minimum_mu", ct.c_double),
        ("maximum_mu", ct.c_double),
        ("divergence_upper_bound", ct.c_double),
        ("enable_line_search", ct.c_int),
        ("dinf_upper_bound", ct.c_double),
        ("final_centering_steps", ct.c_int),
        ("final_centering_tolerance", ct.c_double),
        ("initial_centering_steps_warmstart", ct.c_int),
        ("initial_centering_steps_coldstart", ct.c_int),
        ("warmstart_abort_threshold", ct.c_double),
        ("max_iterations", ct.c_int),
        ("iterative_refinement_iterations", ct.c_int),
        ("infeasibility_threshold", ct.c_double),
        ("kkt_error_tolerance", ct.c_double),
        ("enable_rescaling", ct.c_int),
        ("enable_scale_correction", ct.c_int),
        ("kkt_solver", ct.c_int),
        ("verbosity", ct.c_double),
    ]


class CONEX_IterationStats(ct.Structure):
    _fields_ = [
        ("mu", ct.c_double),
        ("iteration_number", ct.c_int),
    ]


class CONEX_SolutionStats(ct.Structure):
    _fields_ = [
        ("iterations", ct.c_int),
        ("duality_gap", ct.c_double),
    ]


def _lib_path():
    here = Path(__file__).resolve().parent
    return here.parent / "libconex.so"


_LIB = ct.CDLL(str(_lib_path()))
c_double_p = ct.POINTER(ct.c_double)
c_long_p = ct.POINTER(ct.c_long)
c_int_p = ct.POINTER(ct.c_int)


def _ptr_f64(arr):
    return arr.ctypes.data_as(ct.POINTER(ct.c_double))


def _ptr_i64(arr):
    return arr.ctypes.data_as(ct.POINTER(ct.c_long))


def _as_f64_1d(arr):
    return np.asarray(arr, dtype=np.float64).reshape(-1)


def _as_f64_2d_f(arr):
    return np.asfortranarray(np.asarray(arr, dtype=np.float64))


def _as_f64_3d_f(arr):
    return np.asfortranarray(np.asarray(arr, dtype=np.float64))


def _as_mut_f64_1d(arr):
    out = np.asarray(arr)
    if out.dtype == np.float64 and out.flags["C_CONTIGUOUS"]:
        return out, None
    tmp = np.asarray(out, dtype=np.float64).copy()
    return tmp, out


def _as_mut_f64_2d(arr):
    out = np.asarray(arr)
    if out.dtype == np.float64 and out.flags["C_CONTIGUOUS"]:
        return out, None
    tmp = np.asarray(out, dtype=np.float64).copy()
    return tmp, out


_LIB.CONEX_CreateConeProgram.restype = ct.c_void_p
_LIB.CONEX_DeleteConeProgram.argtypes = [ct.c_void_p]
_LIB.CONEX_SetNumberOfVariables.argtypes = [ct.c_void_p, ct.c_int]
_LIB.CONEX_SetNumberOfVariables.restype = ct.c_int
_LIB.CONEX_SetDefaultOptions.argtypes = [ct.POINTER(CONEX_SolverConfiguration)]

_LIB.CONEX_AddDenseLinearConstraint.argtypes = [ct.c_void_p, c_double_p, ct.c_int, ct.c_int, c_double_p, ct.c_int]
_LIB.CONEX_AddDenseLinearConstraint.restype = ct.c_int
_LIB.CONEX_AddLinearInequalities.argtypes = [ct.c_void_p, c_double_p, ct.c_int, ct.c_int, c_double_p, ct.c_int, c_double_p, ct.c_int]
_LIB.CONEX_AddLinearInequalities.restype = ct.c_int
_LIB.CONEX_AddQuadraticCost.argtypes = [ct.c_void_p, c_double_p, ct.c_int, ct.c_int]
_LIB.CONEX_AddQuadraticCost.restype = ct.c_int
_LIB.CONEX_AddDenseLMIConstraint.argtypes = [ct.c_void_p, c_double_p, ct.c_int, ct.c_int, ct.c_int, c_double_p, ct.c_int, ct.c_int]
_LIB.CONEX_AddDenseLMIConstraint.restype = ct.c_int
_LIB.CONEX_AddSparseLMIConstraint.argtypes = [ct.c_void_p, c_double_p, ct.c_int, ct.c_int, ct.c_int, c_double_p, ct.c_int, ct.c_int, c_long_p, ct.c_int]
_LIB.CONEX_AddSparseLMIConstraint.restype = ct.c_int
_LIB.CONEX_AddLinearCost.argtypes = [ct.c_void_p, c_double_p, ct.c_int]
_LIB.CONEX_AddLinearCost.restype = ct.c_int
_LIB.CONEX_Solve.argtypes = [ct.c_void_p, ct.POINTER(CONEX_SolverConfiguration), c_double_p, ct.c_int]
_LIB.CONEX_Solve.restype = ct.c_int
_LIB.CONEX_Maximize.argtypes = [ct.c_void_p, c_double_p, ct.c_int, ct.POINTER(CONEX_SolverConfiguration), c_double_p, ct.c_int]
_LIB.CONEX_Maximize.restype = ct.c_int
_LIB.CONEX_GetDualVariable.argtypes = [ct.c_void_p, ct.c_int, c_double_p, ct.c_int, ct.c_int]
_LIB.CONEX_GetIterationStats.argtypes = [ct.c_void_p, ct.POINTER(CONEX_IterationStats), ct.c_int]

_LIB.CONEX_NewLinearMatrixInequality.argtypes = [ct.c_void_p, ct.c_int, ct.c_int, c_int_p]
_LIB.CONEX_NewLinearMatrixInequality.restype = ct.c_int
_LIB.CONEX_UpdateLinearOperator.argtypes = [ct.c_void_p, ct.c_int, ct.c_double, ct.c_int, ct.c_int, ct.c_int, ct.c_int]
_LIB.CONEX_UpdateLinearOperator.restype = ct.c_int
_LIB.CONEX_UpdateAffineTerm.argtypes = [ct.c_void_p, ct.c_int, ct.c_double, ct.c_int, ct.c_int, ct.c_int]
_LIB.CONEX_UpdateAffineTerm.restype = ct.c_int
_LIB.CONEX_NewLorentzConeConstraint.argtypes = [ct.c_void_p, ct.c_int, c_int_p]
_LIB.CONEX_NewLorentzConeConstraint.restype = ct.c_int
_LIB.CONEX_NewLinearInequality.argtypes = [ct.c_void_p, ct.c_int, c_int_p]
_LIB.CONEX_NewLinearInequality.restype = ct.c_int
_LIB.CONEX_NewQuadraticCost.argtypes = [ct.c_void_p, c_int_p]
_LIB.CONEX_NewQuadraticCost.restype = ct.c_int
_LIB.CONEX_UpdateQuadraticCostMatrix.argtypes = [ct.c_void_p, ct.c_int, ct.c_double, ct.c_int, ct.c_int]
_LIB.CONEX_UpdateQuadraticCostMatrix.restype = ct.c_int

_LIB.CONEX_QP_Solver.argtypes = [
    c_double_p, ct.c_int, ct.c_int,
    c_double_p, ct.c_int,
    c_double_p, ct.c_int, ct.c_int,
    c_double_p, ct.c_int,
    c_double_p, ct.c_int,
    ct.POINTER(CONEX_SolverConfiguration),
    c_double_p, ct.c_int,
    ct.POINTER(CONEX_SolutionStats),
]
_LIB.CONEX_QP_Solver.restype = ct.c_int

_LIB.CONEX_QP_GetCanonicalProblemData.argtypes = [
    c_double_p, ct.c_int, ct.c_int,
    c_double_p, ct.c_int,
    c_double_p, ct.c_int, ct.c_int,
    c_double_p, ct.c_int,
    c_double_p, ct.c_int,
    c_int_p, c_int_p,
    c_double_p, ct.c_int, ct.c_int,
    c_double_p, ct.c_int,
    c_double_p, ct.c_int, ct.c_int,
    c_double_p, ct.c_int,
]
_LIB.CONEX_QP_GetCanonicalProblemData.restype = ct.c_int


def CONEX_CreateConeProgram():
    return _LIB.CONEX_CreateConeProgram()


def CONEX_DeleteConeProgram(prog):
    _LIB.CONEX_DeleteConeProgram(prog)


def CONEX_SetNumberOfVariables(program, m):
    return int(_LIB.CONEX_SetNumberOfVariables(program, int(m)))


def CONEX_SetDefaultOptions(config):
    _LIB.CONEX_SetDefaultOptions(ct.byref(config))


def CONEX_AddDenseLinearConstraint(prog, A, c):
    a = _as_f64_2d_f(A)
    c_ = _as_f64_1d(c)
    return int(
        _LIB.CONEX_AddDenseLinearConstraint(
            prog, _ptr_f64(a), a.shape[0], a.shape[1], _ptr_f64(c_), c_.shape[0]
        )
    )


def CONEX_AddLinearInequalities(prog, A, lb, ub):
    a = _as_f64_2d_f(A)
    lb_ = _as_f64_1d(lb)
    ub_ = _as_f64_1d(ub)
    return int(
        _LIB.CONEX_AddLinearInequalities(
            prog,
            _ptr_f64(a),
            a.shape[0],
            a.shape[1],
            _ptr_f64(lb_),
            lb_.shape[0],
            _ptr_f64(ub_),
            ub_.shape[0],
        )
    )


def CONEX_AddQuadraticCost(prog, A):
    a = _as_f64_2d_f(A)
    return int(_LIB.CONEX_AddQuadraticCost(prog, _ptr_f64(a), a.shape[0], a.shape[1]))


def CONEX_AddDenseLMIConstraint(prog, Aarray, cmat):
    a = _as_f64_3d_f(Aarray)
    c = _as_f64_2d_f(cmat)
    return int(
        _LIB.CONEX_AddDenseLMIConstraint(
            prog, _ptr_f64(a), a.shape[0], a.shape[1], a.shape[2], _ptr_f64(c), c.shape[0], c.shape[1]
        )
    )


def CONEX_AddSparseLMIConstraint(prog, Aarray, cmat, vars_):
    a = _as_f64_3d_f(Aarray)
    c = _as_f64_2d_f(cmat)
    v = np.asarray(vars_, dtype=np.int64).reshape(-1)
    return int(
        _LIB.CONEX_AddSparseLMIConstraint(
            prog,
            _ptr_f64(a),
            a.shape[0],
            a.shape[1],
            a.shape[2],
            _ptr_f64(c),
            c.shape[0],
            c.shape[1],
            _ptr_i64(v),
            v.shape[0],
        )
    )


def CONEX_AddLinearCost(prog, b):
    b_ = _as_f64_1d(b)
    return int(_LIB.CONEX_AddLinearCost(prog, _ptr_f64(b_), b_.shape[0]))


def CONEX_Solve(prog, config, y):
    y_, copyback = _as_mut_f64_1d(y)
    status = int(_LIB.CONEX_Solve(prog, ct.byref(config), _ptr_f64(y_), y_.shape[0]))
    if copyback is not None:
        np.copyto(copyback, y_)
    return status


def CONEX_Maximize(prog, b, config, y):
    b_ = _as_f64_1d(b)
    y_, copyback = _as_mut_f64_1d(y)
    status = int(
        _LIB.CONEX_Maximize(prog, _ptr_f64(b_), b_.shape[0], ct.byref(config), _ptr_f64(y_), y_.shape[0])
    )
    if copyback is not None:
        np.copyto(copyback, y_)
    return status


def CONEX_GetDualVariable(prog, i, x):
    x_, copyback = _as_mut_f64_2d(x)
    _LIB.CONEX_GetDualVariable(prog, int(i), _ptr_f64(x_), x_.shape[0], x_.shape[1])
    if copyback is not None:
        np.copyto(copyback, x_)


def CONEX_GetIterationStats(prog, stats, iter_num):
    _LIB.CONEX_GetIterationStats(prog, ct.byref(stats), int(iter_num))


def CONEX_NewLinearMatrixInequality(program, order, hyper_complex_dim, constraint_id):
    return int(
        _LIB.CONEX_NewLinearMatrixInequality(
            program, int(order), int(hyper_complex_dim), ct.byref(constraint_id._value)
        )
    )


def CONEX_UpdateLinearOperator(program, constraint, value, variable, row, col, hyper_complex_dim):
    return int(
        _LIB.CONEX_UpdateLinearOperator(
            program, int(constraint), float(value), int(variable), int(row), int(col), int(hyper_complex_dim)
        )
    )


def CONEX_UpdateAffineTerm(program, constraint, value, row, col, hyper_complex_dim):
    return int(
        _LIB.CONEX_UpdateAffineTerm(program, int(constraint), float(value), int(row), int(col), int(hyper_complex_dim))
    )


def CONEX_NewLorentzConeConstraint(program, order, constraint_id):
    return int(_LIB.CONEX_NewLorentzConeConstraint(program, int(order), ct.byref(constraint_id._value)))


def CONEX_NewLinearInequality(program, num_rows, constraint_id):
    return int(_LIB.CONEX_NewLinearInequality(program, int(num_rows), ct.byref(constraint_id._value)))


def CONEX_NewQuadraticCost(program, constraint_id):
    return int(_LIB.CONEX_NewQuadraticCost(program, ct.byref(constraint_id._value)))


def CONEX_UpdateQuadraticCostMatrix(program, cost_id, value, row, col):
    return int(_LIB.CONEX_UpdateQuadraticCostMatrix(program, int(cost_id), float(value), int(row), int(col)))


def CONEX_QP_Solver(quadratic_cost_matrix, cost_vector, inequality_matrix, inequality_upper_bound, inequality_lower_bound, config_input, solution, stats):
    q = _as_f64_2d_f(quadratic_cost_matrix)
    c = _as_f64_1d(cost_vector)
    a = _as_f64_2d_f(inequality_matrix)
    ub = _as_f64_1d(inequality_upper_bound)
    lb = _as_f64_1d(inequality_lower_bound)
    y, copyback = _as_mut_f64_1d(solution)
    status = int(
        _LIB.CONEX_QP_Solver(
            _ptr_f64(q),
            q.shape[0],
            q.shape[1],
            _ptr_f64(c),
            c.shape[0],
            _ptr_f64(a),
            a.shape[0],
            a.shape[1],
            _ptr_f64(ub),
            ub.shape[0],
            _ptr_f64(lb),
            lb.shape[0],
            ct.byref(config_input),
            _ptr_f64(y),
            y.shape[0],
            ct.byref(stats),
        )
    )
    if copyback is not None:
        np.copyto(copyback, y)
    return status


def CONEX_QP_GetCanonicalProblemData(
    quadratic_cost_matrix,
    cost_vector,
    inequality_matrix,
    inequality_upper_bound,
    inequality_lower_bound,
    num_ineq,
    num_eq,
    matrix_A,
    vector_b,
    matrix_B,
    vector_d,
):
    q = _as_f64_2d_f(quadratic_cost_matrix)
    c = _as_f64_1d(cost_vector)
    a = _as_f64_2d_f(inequality_matrix)
    ub = _as_f64_1d(inequality_upper_bound)
    lb = _as_f64_1d(inequality_lower_bound)
    A, A_copyback = _as_mut_f64_2d(matrix_A)
    b, b_copyback = _as_mut_f64_1d(vector_b)
    B, B_copyback = _as_mut_f64_2d(matrix_B)
    d, d_copyback = _as_mut_f64_1d(vector_d)
    status = int(
        _LIB.CONEX_QP_GetCanonicalProblemData(
            _ptr_f64(q),
            q.shape[0],
            q.shape[1],
            _ptr_f64(c),
            c.shape[0],
            _ptr_f64(a),
            a.shape[0],
            a.shape[1],
            _ptr_f64(ub),
            ub.shape[0],
            _ptr_f64(lb),
            lb.shape[0],
            ct.byref(num_ineq._value),
            ct.byref(num_eq._value),
            _ptr_f64(A),
            A.shape[0],
            A.shape[1],
            _ptr_f64(b),
            b.shape[0],
            _ptr_f64(B),
            B.shape[0],
            B.shape[1],
            _ptr_f64(d),
            d.shape[0],
        )
    )
    if A_copyback is not None:
        np.copyto(A_copyback, A)
    if b_copyback is not None:
        np.copyto(b_copyback, b)
    if B_copyback is not None:
        np.copyto(B_copyback, B)
    if d_copyback is not None:
        np.copyto(d_copyback, d)
    return status
