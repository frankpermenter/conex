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


def sparse_ls(A, b, num_threads=1, clique_tree_method=CLIQUE_TREE_METHOD_AMD):
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
    clique_tree_method : int
        Clique-tree ordering strategy (e.g., CLIQUE_TREE_METHOD_AMD or
        CLIQUE_TREE_METHOD_WEIGHTED_DFS).
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
        clique_tree_method=int(clique_tree_method),
    )


def sparse_ls_ne(A, rhs):
    """
    Solve A^T A x = rhs using SparseLinearConstraint-based tree solver.

    Decomposes A by row-support containment, builds a tree solver from
    LinearConstraint sub-blocks, and solves the normal equations.

    Parameters
    ----------
    A : scipy.sparse.spmatrix
        Sparse matrix whose normal equations A^T A define the system.
    rhs : array_like, shape (n,)
        Right-hand-side vector (length = A.cols).

    Returns
    -------
    dict with keys:
        x : ndarray, shape (n,) — solution
        construction_us : float — construction time in microseconds
        assemble_and_factor_us : float — assemble+factor time in microseconds
        solve_us : float — solve time in microseconds
    """
    try:
        import scipy.sparse as sp
    except ImportError as exc:
        raise ImportError("scipy is required for conex.sparse_ls_ne") from exc

    if not sp.issparse(A):
        raise TypeError("A must be a scipy sparse matrix.")
    Acsr = A.tocsr()
    rhs_vec = np.asarray(rhs, dtype=np.float64).reshape(-1)
    if rhs_vec.shape[0] != Acsr.shape[1]:
        raise ValueError("rhs length must match A.shape[1].")
    return sparse_ls_normal_equations(
        Acsr.indptr.astype(np.int64, copy=False),
        Acsr.indices.astype(np.int64, copy=False),
        Acsr.data.astype(np.float64, copy=False),
        int(Acsr.shape[0]),
        int(Acsr.shape[1]),
        rhs_vec,
    )


def _row_supports(Acsr):
    groups = {}
    for r in range(Acsr.shape[0]):
        lo = Acsr.indptr[r]
        hi = Acsr.indptr[r + 1]
        support = tuple(sorted(set(int(c) for c in Acsr.indices[lo:hi])))
        if not support:
            continue
        groups.setdefault(support, []).append(r)
    return groups


def _bags_from_treewidth_min_fill(num_vars, ata_pattern):
    try:
        import networkx as nx
        from networkx.algorithms.approximation import treewidth_min_fill_in
    except Exception as exc:  # pragma: no cover
        raise ImportError("networkx is required for sparse_ls_tree.") from exc

    g = nx.Graph()
    g.add_nodes_from(range(num_vars))
    coo = ata_pattern.tocoo()
    for i, j in zip(coo.row, coo.col):
        if i == j:
            continue
        if i < j:
            g.add_edge(int(i), int(j))
    _, decomp = treewidth_min_fill_in(g)
    bags = [tuple(sorted(int(v) for v in bag)) for bag in decomp.nodes]
    adj = {bag: [] for bag in bags}
    for u, v in decomp.edges:
        uu = tuple(sorted(int(x) for x in u))
        vv = tuple(sorted(int(x) for x in v))
        adj[uu].append(vv)
        adj[vv].append(uu)
    return bags, adj


def _orient_bag_forest(bags, adj):
    parent_map = {}
    order = []
    seen = set()
    for start in bags:
        if start in seen:
            continue
        # Root each component at its largest bag.
        component = []
        queue = [start]
        seen.add(start)
        while queue:
            cur = queue.pop(0)
            component.append(cur)
            for nxt in adj.get(cur, []):
                if nxt in seen:
                    continue
                seen.add(nxt)
                queue.append(nxt)
        root = max(component, key=len)
        parent_map[root] = None
        queue = [root]
        visited = {root}
        while queue:
            cur = queue.pop(0)
            order.append(cur)
            for nxt in adj.get(cur, []):
                if nxt in visited:
                    continue
                visited.add(nxt)
                parent_map[nxt] = cur
                queue.append(nxt)
    return order, parent_map


def _bags_from_min_fill_supports(num_vars, supports):
    try:
        import networkx as nx
        from networkx.algorithms.approximation import treewidth_min_fill_in
    except Exception as exc:  # pragma: no cover
        raise ImportError("networkx is required for sparse_solve_blocks_tree.") from exc

    g = nx.Graph()
    g.add_nodes_from(range(num_vars))
    for support in supports:
        vars_ = list(support)
        for i in range(len(vars_)):
            for j in range(i + 1, len(vars_)):
                g.add_edge(int(vars_[i]), int(vars_[j]))
    _, decomp = treewidth_min_fill_in(g)
    bags = [tuple(sorted(int(v) for v in bag)) for bag in decomp.nodes]
    adj = {bag: [] for bag in bags}
    for u, v in decomp.edges:
        uu = tuple(sorted(int(x) for x in u))
        vv = tuple(sorted(int(x) for x in v))
        adj[uu].append(vv)
        adj[vv].append(uu)
    return bags, adj


def sparse_ls_tree(A, b, num_threads=1):
    """
    Solve min_x ||Ax - b||^2 using the exposed KKTTreeSolver API and an
    off-the-shelf tree decomposition heuristic (NetworkX min-fill).
    Falls back to sparse_ls if NetworkX is unavailable.
    """
    try:
        import scipy.sparse as sp
    except ImportError as exc:
        raise ImportError("scipy is required for conex.sparse_ls_tree") from exc

    if not sp.issparse(A):
        raise TypeError("A must be a scipy sparse matrix.")
    Acsr = A.tocsr()
    bvec = np.asarray(b, dtype=np.float64).reshape(-1)
    if bvec.shape[0] != Acsr.shape[0]:
        raise ValueError("b length must match A.shape[0].")

    try:
        ata_pattern = (Acsr.T @ Acsr).tocsr(copy=False)
        bags, adj = _bags_from_treewidth_min_fill(int(Acsr.shape[1]), ata_pattern)
    except ImportError:
        return sparse_ls(Acsr, bvec, num_threads=num_threads)

    n = int(Acsr.shape[1])
    if not bags:
        return np.zeros(n, dtype=np.float64)

    # Ensure every variable is present in at least one bag.
    covered = set(v for bag in bags for v in bag)
    for v in range(n):
        if v not in covered:
            bags.append((v,))
            adj[(v,)] = []

    order, parent_map = _orient_bag_forest(bags, adj)

    # Convert tree bags to clique-tree arrays.
    bag_to_idx = {bag: i for i, bag in enumerate(order)}
    parent = []
    supernodes = []
    separators = []
    for bag in order:
        p = parent_map[bag]
        if p is None:
            sep = []
            parent.append(-1)
        else:
            sep = sorted(set(bag).intersection(p))
            parent.append(bag_to_idx[p])
        sup = sorted(v for v in bag if v not in set(sep))
        separators.append(np.asarray(sep, dtype=np.int64))
        supernodes.append(np.asarray(sup, dtype=np.int64))

    # Build local clique matrices from support groups.
    groups = _row_supports(Acsr)
    bag_sets = [set(bag) for bag in order]
    local_mats = [np.zeros((len(bag), len(bag)), dtype=np.float64) for bag in order]
    local_pos = [{v: j for j, v in enumerate(bag)} for bag in order]
    for support, rows in groups.items():
        sset = set(support)
        candidates = [i for i, bset in enumerate(bag_sets) if sset.issubset(bset)]
        if not candidates:
            # Fallback placement on root.
            target = 0
        else:
            target = min(candidates, key=lambda i: len(order[i]))
        vars_ = list(support)
        Ag = Acsr[rows, :][:, vars_].toarray()
        block = Ag.T @ Ag
        pos = local_pos[target]
        for ii, vi in enumerate(vars_):
            ri = pos[vi]
            for jj, vj in enumerate(vars_):
                cj = pos[vj]
                local_mats[target][ri, cj] += block[ii, jj]

    solver = KKTTreeSolver()
    solver.set_num_threads(int(num_threads))
    solver.set_factorization_mode(True)
    for bag, local in zip(order, local_mats):
        solver.add_dense_subsystem(np.asarray(bag, dtype=np.int64), local)
    solver.finalize(supernodes, separators, np.asarray(parent, dtype=np.int64))
    if not solver.assemble_and_factor():
        raise RuntimeError("KKTTreeSolver factorization failed in sparse_ls_tree.")

    rhs = (Acsr.T @ bvec).reshape(-1, 1)
    x = solver.solve(rhs, True)
    return np.asarray(x, dtype=np.float64).reshape(-1)


def _aggregate_labeled_blocks(submatrices_by_group, labels):
    if len(submatrices_by_group) != len(labels):
        raise ValueError("submatrices_by_group and labels must have same length.")
    aggregated = {}
    for mats, label in zip(submatrices_by_group, labels):
        bag = tuple(sorted(int(v) for v in label))
        if len(bag) == 0:
            continue
        if not isinstance(mats, (list, tuple)):
            raise TypeError("Each submatrices_by_group entry must be a list/tuple of 2D arrays.")
        if bag not in aggregated:
            aggregated[bag] = np.zeros((len(bag), len(bag)), dtype=np.float64)
        for mat in mats:
            block = np.asarray(mat, dtype=np.float64)
            if block.ndim != 2 or block.shape[0] != block.shape[1]:
                raise ValueError("Each submatrix must be a square 2D array.")
            if block.shape[0] != len(bag):
                raise ValueError("Submatrix shape must match its label tuple length.")
            aggregated[bag] += block
    if not aggregated:
        raise ValueError("No non-empty labeled blocks were provided.")
    return aggregated


def _embed_block_in_bag(original_bag, expanded_bag, block):
    out = np.zeros((len(expanded_bag), len(expanded_bag)), dtype=np.float64)
    pos = {v: i for i, v in enumerate(expanded_bag)}
    for i, vi in enumerate(original_bag):
        ri = pos[vi]
        for j, vj in enumerate(original_bag):
            cj = pos[vj]
            out[ri, cj] = block[i, j]
    return out


def clique_tree_stats(tree):
    """
    Report structural properties of a clique tree.

    Parameters
    ----------
    tree : dict | tuple
        Either a dict with keys `supernodes`, `separators`, `node_to_parent`,
        or a tuple `(supernodes, separators, node_to_parent)`.
    """
    if isinstance(tree, dict):
        supernodes = tree["supernodes"]
        separators = tree["separators"]
        parent = tree["node_to_parent"]
    else:
        if len(tree) != 3:
            raise ValueError(
                "tree must be dict or 3-tuple: (supernodes, separators, node_to_parent)."
            )
        supernodes, separators, parent = tree

    n = len(parent)
    if len(supernodes) != n or len(separators) != n:
        raise ValueError("supernodes/separators/node_to_parent size mismatch.")

    clique_sizes = []
    separator_sizes = []
    for sup, sep in zip(supernodes, separators):
        sup_set = set(int(v) for v in np.asarray(sup, dtype=np.int64).reshape(-1))
        sep_set = set(int(v) for v in np.asarray(sep, dtype=np.int64).reshape(-1))
        clique_sizes.append(len(sup_set.union(sep_set)))
        separator_sizes.append(len(sep_set))

    roots = int(np.sum(np.asarray(parent, dtype=np.int64) == -1))
    return {
        "num_cliques": int(n),
        "num_roots": roots,
        "clique_sizes": clique_sizes,
        "separator_sizes": separator_sizes,
        "clique_size_min": int(min(clique_sizes)) if clique_sizes else 0,
        "clique_size_max": int(max(clique_sizes)) if clique_sizes else 0,
        "clique_size_mean": float(np.mean(clique_sizes)) if clique_sizes else 0.0,
        "separator_size_min": int(min(separator_sizes)) if separator_sizes else 0,
        "separator_size_max": int(max(separator_sizes)) if separator_sizes else 0,
        "separator_size_mean": float(np.mean(separator_sizes)) if separator_sizes else 0.0,
    }


def sparse_solve_blocks_tree(
    submatrices_by_group,
    labels,
    b,
    num_threads=1,
    tree=None,
    clique_tree_method=CLIQUE_TREE_METHOD_AMD,
    return_tree_stats=False,
    parallelize_roots_only=False,
):
    """
    Solve Kx=b from labeled block contributions using KKTTreeSolver.

    Parameters
    ----------
    submatrices_by_group : list[list[array_like]]
        Each outer entry corresponds to one label tuple. Inner list contains one
        or more square submatrices (same shape as label length) to be summed.
    labels : list[tuple[int] | list[int] | array_like]
        Global variable labels for each group of submatrices.
    b : array_like, shape (n,) or (n, k)
        Right-hand side(s) in global variable order.
    num_threads : int
        Number of tree-solver threads.
    tree : None | dict | tuple
        Optional explicit tree specification. If provided, it must define
        `supernodes`, `separators`, and `node_to_parent` (either as a dict
        with those keys or a 3-tuple in that order).
    clique_tree_method : int
        Clique-tree construction method passed to C++ `build_clique_tree`
        when `tree` is None.
    return_tree_stats : bool
        If True, return `(x, stats)` where `stats = clique_tree_stats(tree_used)`.
    parallelize_roots_only : bool
        If True, only parallelize across root subtrees in the tree solver and
        disable subsystem-internal threading.
    """
    blocks = _aggregate_labeled_blocks(submatrices_by_group, labels)
    original_bags = list(blocks.keys())

    n = 1 + max(v for bag in original_bags for v in bag)
    rhs = np.asarray(b, dtype=np.float64)
    if rhs.ndim == 1:
        rhs = rhs.reshape(-1, 1)
    if rhs.ndim != 2:
        raise ValueError("b must be 1D or 2D.")
    if rhs.shape[0] != n:
        raise ValueError("b row count must match max labeled variable index + 1.")

    if tree is None:
        tree_data = build_clique_tree(
            [list(bag) for bag in original_bags], method=int(clique_tree_method)
        )
        supernodes = tree_data["supernodes"]
        separators = tree_data["separators"]
        parent = tree_data["node_to_parent"]
    elif isinstance(tree, dict):
        supernodes = tree["supernodes"]
        separators = tree["separators"]
        parent = tree["node_to_parent"]
    else:
        if len(tree) != 3:
            raise ValueError(
                "tree must be dict or 3-tuple: (supernodes, separators, node_to_parent)."
            )
        supernodes, separators, parent = tree

    if (
        len(supernodes) != len(original_bags)
        or len(separators) != len(original_bags)
        or len(parent) != len(original_bags)
    ):
        raise ValueError("tree arrays must match number of aggregated labeled blocks.")

    solver = KKTTreeSolver()
    solver.set_num_threads(int(num_threads))
    solver.set_parallelize_roots_only(bool(parallelize_roots_only))
    solver.set_factorization_mode(True)
    for bag in original_bags:
        solver.add_dense_subsystem(np.asarray(bag, dtype=np.int64), blocks[bag])
    solver.finalize(supernodes, separators, np.asarray(parent, dtype=np.int64))
    if not solver.assemble_and_factor():
        raise RuntimeError("KKTTreeSolver factorization failed in sparse_solve_blocks_tree.")
    x = solver.solve(rhs, True)
    x = np.asarray(x, dtype=np.float64).reshape(rhs.shape)
    if return_tree_stats:
        return x, clique_tree_stats(
            {"supernodes": supernodes, "separators": separators, "node_to_parent": parent}
        )
    return x
