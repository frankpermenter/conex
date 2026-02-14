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


def sparse_solve_blocks_tree(submatrices_by_group, labels, b, num_threads=1):
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

    bags, adj = _bags_from_min_fill_supports(n, original_bags)
    if not bags:
        raise ValueError("Failed to build clique bags from labeled supports.")
    order, parent_map = _orient_bag_forest(bags, adj)

    bag_sets = [set(bag) for bag in order]
    local_mats = [np.zeros((len(bag), len(bag)), dtype=np.float64) for bag in order]
    for support in original_bags:
        sset = set(support)
        candidates = [i for i, bset in enumerate(bag_sets) if sset.issubset(bset)]
        if not candidates:
            raise RuntimeError(
                "No decomposition clique contains support "
                f"{support}. Recompute decomposition with augmented supports."
            )
        target = min(candidates, key=lambda i: len(order[i]))
        local_mats[target] += _embed_block_in_bag(support, order[target], blocks[support])

    supernodes = []
    separators = []
    parent = []
    bag_to_idx = {bag: i for i, bag in enumerate(order)}
    for bag in order:
        p = parent_map[bag]
        if p is None:
            sep = []
            parent_i = -1
        else:
            sep = sorted(set(bag).intersection(set(p)))
            parent_i = bag_to_idx[p]
        sup = sorted(v for v in bag if v not in set(sep))
        supernodes.append(np.asarray(sup, dtype=np.int64))
        separators.append(np.asarray(sep, dtype=np.int64))
        parent.append(parent_i)

    solver = KKTTreeSolver()
    solver.set_num_threads(int(num_threads))
    solver.set_factorization_mode(True)
    for bag, local in zip(order, local_mats):
        solver.add_dense_subsystem(np.asarray(bag, dtype=np.int64), local)
    solver.finalize(supernodes, separators, np.asarray(parent, dtype=np.int64))
    if not solver.assemble_and_factor():
        raise RuntimeError("KKTTreeSolver factorization failed in sparse_solve_blocks_tree.")
    x = solver.solve(rhs, True)
    return np.asarray(x, dtype=np.float64).reshape(rhs.shape)
