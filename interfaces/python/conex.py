"""
Thin Python shim for the compiled pybind11 extension.

The extension module is named ``_conex`` and is built by CMake.
"""

try:
    from _conex import *  # noqa: F401,F403
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Could not import '_conex'. Build the extension first with "
        "'make -C interfaces/python' (requires CMake)."
    ) from exc

import numpy as np


def sparse_ls(A, b, num_threads=1):
    """
    Solve min_x ||Ax - b||^2 using Conex tree-based normal-equations solver.

    Parameters
    ----------
    A : scipy.sparse.spmatrix
        Sparse design matrix.
    b : array_like, shape (m,)
        Right-hand-side vector.
    num_threads : int
        Number of threads used by the tree solver.
    """
    try:
        import scipy.sparse as sp
    except ImportError as exc:
        raise ImportError("scipy is required for conex.sparse_ls") from exc

    if not sp.issparse(A):
        raise TypeError("A must be a scipy sparse matrix.")
    Acsr = A.tocsr()
    bvec = np.asarray(b, dtype=np.float64).reshape(-1)
    if bvec.shape[0] != Acsr.shape[0]:
        raise ValueError("b length must match A.shape[0].")
    return sparse_ls_csr(
        Acsr.indptr.astype(np.int64, copy=False),
        Acsr.indices.astype(np.int64, copy=False),
        Acsr.data.astype(np.float64, copy=False),
        int(Acsr.shape[0]),
        int(Acsr.shape[1]),
        bvec,
        num_threads=int(num_threads),
    )
