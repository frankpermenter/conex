// Benchmark: GpuTreeSolver vs cuDSS vs cuSOLVER-Sp on block-arrow matrices.
//
// This file must be compiled with CUDA 12 headers (for cuDSS).
// The gpu_tree_solver library itself is compiled with CUDA 11.5.

#include "conex/gpu_tree_solver/gpu_tree_solver.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_solver_factory.h"

#include <chrono>
#include <cstdio>
#include <set>
#include <unordered_map>
#include <vector>

#include <cuda_runtime.h>
#include <cusolverSp.h>
#include <cusolverSp_LOWLEVEL_PREVIEW.h>
#include <cusparse.h>
#include <cudss.h>

namespace conex {
namespace {

// ============================================================================
// Problem construction
// ============================================================================

struct BenchProblem {
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd rhs;
  Eigen::VectorXd x_true;
  CliqueTree clique_tree;
  std::vector<std::vector<int>> maximal_cliques;
};

BenchProblem MakeBlockArrow(int num_blocks, int block_size, int sep_size,
                            int rows_per_block, int seed = 42) {
  srand(seed);
  BenchProblem p;

  int n = num_blocks * block_size + sep_size;
  int total_rows = num_blocks * rows_per_block;

  std::vector<Eigen::Triplet<double>> trips;
  for (int b = 0; b < num_blocks; ++b) {
    int col_start = b * block_size;
    int sep_start = num_blocks * block_size;
    for (int i = 0; i < rows_per_block; ++i) {
      int row = b * rows_per_block + i;
      for (int j = 0; j < block_size; ++j) {
        double val = static_cast<double>(rand()) / RAND_MAX - 0.5;
        trips.emplace_back(row, col_start + j, val);
      }
      for (int j = 0; j < sep_size; ++j) {
        double val = 0.1 * (static_cast<double>(rand()) / RAND_MAX - 0.5);
        trips.emplace_back(row, sep_start + j, val);
      }
    }
  }

  p.A.resize(total_rows, n);
  p.A.setFromTriplets(trips.begin(), trips.end());

  p.x_true = Eigen::VectorXd::Random(n);
  Eigen::MatrixXd Ad(p.A);
  p.rhs = Ad.transpose() * (Ad * p.x_true);

  auto slc = std::make_unique<SparseLinearConstraint>(
      p.A, Eigen::VectorXd::Zero(total_rows));
  std::vector<std::vector<int>> cliques;
  for (const auto& s : slc->row_supports()) cliques.push_back(s);
  p.clique_tree =
      MakeCliqueTreeMinDegreeFromRowSupports(cliques, &p.maximal_cliques);

  return p;
}

// ============================================================================
// Assembly helper for tree solver
// ============================================================================

struct AssembledBlocks {
  std::vector<Eigen::MatrixXd> blocks;
};

AssembledBlocks AssembleBlocks(const BenchProblem& prob) {
  const auto& ct = prob.clique_tree;
  const int n = prob.A.cols();
  const int num_cliques = static_cast<int>(ct.supernodes.size());

  Eigen::VectorXi perm = Eigen::VectorXi::Constant(n, -1);
  int epos = 0;
  for (int ci : ct.post_order_position_to_clique)
    for (int v : ct.supernodes[ci])
      if (v >= 0 && v < n) perm(v) = epos++;

  std::vector<std::set<int>> clique_vars(num_cliques);
  for (int ci = 0; ci < num_cliques; ++ci) {
    for (int v : ct.supernodes[ci]) clique_vars[ci].insert(v);
    for (int v : ct.separators[ci]) clique_vars[ci].insert(v);
  }

  std::vector<std::vector<int>> clique_var_list(num_cliques);
  std::vector<std::unordered_map<int, int>> var_to_local(num_cliques);
  for (int ci = 0; ci < num_cliques; ++ci) {
    auto& vars = clique_var_list[ci];
    vars = ct.supernodes[ci];
    vars.insert(vars.end(), ct.separators[ci].begin(),
                ct.separators[ci].end());
    for (int k = 0; k < static_cast<int>(vars.size()); ++k)
      var_to_local[ci][vars[k]] = k;
  }

  AssembledBlocks out;
  out.blocks.resize(num_cliques);
  for (int ci = 0; ci < num_cliques; ++ci)
    out.blocks[ci] = Eigen::MatrixXd::Zero(clique_var_list[ci].size(),
                                            clique_var_list[ci].size());

  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row(prob.A);
  for (int r = 0; r < A_row.rows(); ++r) {
    std::vector<std::pair<int, double>> row_entries;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             A_row, r); it; ++it)
      row_entries.push_back({static_cast<int>(it.col()), it.value()});
    if (row_entries.empty()) continue;

    int first_elim_var = -1, first_elim_pos = n;
    for (const auto& e : row_entries)
      if (perm(e.first) < first_elim_pos) {
        first_elim_pos = perm(e.first);
        first_elim_var = e.first;
      }

    int owner = -1;
    for (int ci = 0; ci < num_cliques; ++ci) {
      for (int v : ct.supernodes[ci])
        if (v == first_elim_var) { owner = ci; break; }
      if (owner >= 0) break;
    }
    if (owner < 0) continue;

    bool all_in = true;
    for (const auto& e : row_entries)
      if (clique_vars[owner].find(e.first) == clique_vars[owner].end()) {
        all_in = false;
        break;
      }
    if (!all_in) {
      std::set<int> support;
      for (const auto& e : row_entries) support.insert(e.first);
      owner = -1;
      for (int ci = 0; ci < num_cliques; ++ci) {
        bool ok = true;
        for (int v : support)
          if (clique_vars[ci].find(v) == clique_vars[ci].end()) {
            ok = false;
            break;
          }
        if (ok) {
          if (owner < 0) {
            owner = ci;
          } else {
            int me_ci = n, me_ow = n;
            for (int v : ct.supernodes[ci])
              me_ci = std::min(me_ci, (int)perm(v));
            for (int v : ct.supernodes[owner])
              me_ow = std::min(me_ow, (int)perm(v));
            if (me_ci < me_ow) owner = ci;
          }
        }
      }
      if (owner < 0) continue;
    }

    const auto& lm = var_to_local[owner];
    for (const auto& ei : row_entries) {
      auto it_i = lm.find(ei.first);
      if (it_i == lm.end()) continue;
      for (const auto& ej : row_entries) {
        auto it_j = lm.find(ej.first);
        if (it_j == lm.end()) continue;
        out.blocks[owner](it_i->second, it_j->second) +=
            ei.second * ej.second;
      }
    }
  }

  return out;
}

// ============================================================================
// CSR lower-triangle builder (shared by cuSOLVER-Sp and cuDSS)
// ============================================================================

struct CsrLowerTriangle {
  std::vector<int> row_ptr;
  std::vector<int> col_ind;
  std::vector<double> val;
  int n;
  int nnz;
};

CsrLowerTriangle BuildCsrLower(const BenchProblem& prob) {
  const int n = prob.A.cols();
  Eigen::SparseMatrix<double, Eigen::ColMajor> AtA_full =
      (prob.A.transpose() * prob.A).pruned();
  Eigen::SparseMatrix<double, Eigen::RowMajor> AtA_row(AtA_full);

  CsrLowerTriangle csr;
  csr.n = n;
  csr.row_ptr.resize(n + 1, 0);
  for (int i = 0; i < n; ++i) {
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             AtA_row, i); it; ++it) {
      int col = static_cast<int>(it.col());
      if (col <= i) {
        csr.col_ind.push_back(col);
        csr.val.push_back(it.value());
      }
    }
    csr.row_ptr[i + 1] = static_cast<int>(csr.col_ind.size());
  }
  csr.nnz = static_cast<int>(csr.val.size());
  return csr;
}

// ============================================================================
// Timing result
// ============================================================================

struct TimingResult {
  double assemble_ms;  // H2D transfer (tree solver) or N/A
  double factor_ms;    // numeric factorization only
  double solve_ms;     // triangular solve (incl. permute + H2D/D2H for tree solver)
  double rel_err;
};

static void CheckCuda(cudaError_t e, const char* m) {
  if (e != cudaSuccess) {
    fprintf(stderr, "CUDA error at %s: %s\n", m, cudaGetErrorString(e));
    exit(1);
  }
}

// ============================================================================
// GPU Tree Solver benchmark
// ============================================================================

TimingResult BenchTreeSolver(const BenchProblem& prob, int warmup, int trials) {
  const auto& ct = prob.clique_tree;
  const int num_cliques = static_cast<int>(ct.supernodes.size());
  auto assembled = AssembleBlocks(prob);

  GpuTreeSolver gpu;
  gpu.Finalize(ct);
  for (int ci = 0; ci < num_cliques; ++ci)
    gpu.SetSupernodeData(ci, assembled.blocks[ci]);

  double total_asm = 0, total_factor = 0, total_solve = 0;
  Eigen::VectorXd x_gpu;

  for (int t = 0; t < warmup + trials; ++t) {
    for (int ci = 0; ci < num_cliques; ++ci)
      gpu.SetSupernodeData(ci, assembled.blocks[ci]);

    cudaDeviceSynchronize();
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = t0;  // assembly not timed separately for now
    gpu.AssembleAndFactor();
    cudaDeviceSynchronize();
    auto t2 = std::chrono::high_resolution_clock::now();
    x_gpu = gpu.Solve(prob.rhs);
    cudaDeviceSynchronize();
    auto t3 = std::chrono::high_resolution_clock::now();

    if (t >= warmup) {
      total_asm +=
          std::chrono::duration<double, std::milli>(t1 - t0).count();
      total_factor +=
          std::chrono::duration<double, std::milli>(t2 - t1).count();
      total_solve +=
          std::chrono::duration<double, std::milli>(t3 - t2).count();
    }
  }

  double rel_err = (x_gpu - prob.x_true).norm() / prob.x_true.norm();
  return {total_asm / trials, total_factor / trials, total_solve / trials,
          rel_err};
}

// ============================================================================
// cuSOLVER sparse Cholesky benchmark
// ============================================================================

static void CheckCusolver(cusolverStatus_t s, const char* m) {
  if (s != CUSOLVER_STATUS_SUCCESS) {
    fprintf(stderr, "cuSOLVER error at %s: %d\n", m, (int)s);
    exit(1);
  }
}

TimingResult BenchCuSolverSp(const BenchProblem& prob, int warmup,
                              int trials) {
  auto csr = BuildCsrLower(prob);
  const int n = csr.n;
  const int nnz = csr.nnz;

  int *d_row = nullptr, *d_col = nullptr;
  double *d_val = nullptr, *d_b = nullptr, *d_x = nullptr;
  CheckCuda(cudaMalloc(&d_row, (n + 1) * sizeof(int)), "row");
  CheckCuda(cudaMalloc(&d_col, nnz * sizeof(int)), "col");
  CheckCuda(cudaMalloc(&d_val, nnz * sizeof(double)), "val");
  CheckCuda(cudaMalloc(&d_b, n * sizeof(double)), "b");
  CheckCuda(cudaMalloc(&d_x, n * sizeof(double)), "x");
  CheckCuda(cudaMemcpy(d_row, csr.row_ptr.data(), (n + 1) * sizeof(int),
                        cudaMemcpyHostToDevice), "cp");
  CheckCuda(cudaMemcpy(d_col, csr.col_ind.data(), nnz * sizeof(int),
                        cudaMemcpyHostToDevice), "cp");
  CheckCuda(cudaMemcpy(d_val, csr.val.data(), nnz * sizeof(double),
                        cudaMemcpyHostToDevice), "cp");
  CheckCuda(cudaMemcpy(d_b, prob.rhs.data(), n * sizeof(double),
                        cudaMemcpyHostToDevice), "cp");

  cusolverSpHandle_t handle;
  CheckCusolver(cusolverSpCreate(&handle), "create");
  cusparseMatDescr_t descr;
  cusparseCreateMatDescr(&descr);
  cusparseSetMatType(descr, CUSPARSE_MATRIX_TYPE_GENERAL);
  cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);
  csrcholInfo_t info;
  CheckCusolver(cusolverSpCreateCsrcholInfo(&info), "cholInfo");
  CheckCusolver(
      cusolverSpXcsrcholAnalysis(handle, n, nnz, descr, d_row, d_col, info),
      "analysis");
  size_t isz = 0, wsz = 0;
  CheckCusolver(cusolverSpDcsrcholBufferInfo(handle, n, nnz, descr, d_val,
                                              d_row, d_col, info, &isz, &wsz),
                "bufInfo");
  void* d_work = nullptr;
  CheckCuda(cudaMalloc(&d_work, wsz), "work");

  double total_factor = 0, total_solve = 0;
  for (int t = 0; t < warmup + trials; ++t) {
    cudaDeviceSynchronize();
    auto t0 = std::chrono::high_resolution_clock::now();
    CheckCusolver(cusolverSpDcsrcholFactor(handle, n, nnz, descr, d_val, d_row,
                                            d_col, info, d_work), "factor");
    cudaDeviceSynchronize();
    auto t1 = std::chrono::high_resolution_clock::now();
    CheckCusolver(
        cusolverSpDcsrcholSolve(handle, n, d_b, d_x, info, d_work), "solve");
    cudaDeviceSynchronize();
    auto t2 = std::chrono::high_resolution_clock::now();
    if (t >= warmup) {
      total_factor +=
          std::chrono::duration<double, std::milli>(t1 - t0).count();
      total_solve +=
          std::chrono::duration<double, std::milli>(t2 - t1).count();
    }
  }

  Eigen::VectorXd x_sol(n);
  CheckCuda(cudaMemcpy(x_sol.data(), d_x, n * sizeof(double),
                        cudaMemcpyDeviceToHost), "cp");
  double rel_err = (x_sol - prob.x_true).norm() / prob.x_true.norm();

  cusolverSpDestroyCsrcholInfo(info);
  cusparseDestroyMatDescr(descr);
  cusolverSpDestroy(handle);
  cudaFree(d_work);
  cudaFree(d_row);
  cudaFree(d_col);
  cudaFree(d_val);
  cudaFree(d_b);
  cudaFree(d_x);
  return {0.0, total_factor / trials, total_solve / trials, rel_err};
}

// ============================================================================
// cuDSS benchmark
// ============================================================================

static void CheckCudss(cudssStatus_t s, const char* m) {
  if (s != CUDSS_STATUS_SUCCESS) {
    fprintf(stderr, "cuDSS error at %s: %d\n", m, (int)s);
    exit(1);
  }
}

TimingResult BenchCuDSS(const BenchProblem& prob, int warmup, int trials) {
  auto csr = BuildCsrLower(prob);
  const int64_t n = csr.n;
  const int64_t nnz = csr.nnz;

  // Upload CSR to device.
  int *d_row = nullptr, *d_col = nullptr;
  double *d_val = nullptr, *d_b = nullptr, *d_x = nullptr;
  CheckCuda(cudaMalloc(&d_row, (n + 1) * sizeof(int)), "row");
  CheckCuda(cudaMalloc(&d_col, nnz * sizeof(int)), "col");
  CheckCuda(cudaMalloc(&d_val, nnz * sizeof(double)), "val");
  CheckCuda(cudaMalloc(&d_b, n * sizeof(double)), "b");
  CheckCuda(cudaMalloc(&d_x, n * sizeof(double)), "x");
  CheckCuda(cudaMemcpy(d_row, csr.row_ptr.data(), (n + 1) * sizeof(int),
                        cudaMemcpyHostToDevice), "cp");
  CheckCuda(cudaMemcpy(d_col, csr.col_ind.data(), nnz * sizeof(int),
                        cudaMemcpyHostToDevice), "cp");
  CheckCuda(cudaMemcpy(d_val, csr.val.data(), nnz * sizeof(double),
                        cudaMemcpyHostToDevice), "cp");
  CheckCuda(cudaMemcpy(d_b, prob.rhs.data(), n * sizeof(double),
                        cudaMemcpyHostToDevice), "cp");
  CheckCuda(cudaMemset(d_x, 0, n * sizeof(double)), "memset");

  // cuDSS setup.
  cudssHandle_t handle;
  CheckCudss(cudssCreate(&handle), "create");

  cudaStream_t stream;
  CheckCuda(cudaStreamCreate(&stream), "stream");
  CheckCudss(cudssSetStream(handle, stream), "setStream");

  // Create sparse matrix (SPD, lower triangle).
  cudssMatrix_t matA;
  CheckCudss(cudssMatrixCreateCsr(&matA, n, n, nnz,
                                   d_row, nullptr, d_col, d_val,
                                   CUDA_R_32I, CUDA_R_64F,
                                   CUDSS_MTYPE_SPD, CUDSS_MVIEW_LOWER,
                                   CUDSS_BASE_ZERO),
             "matA");

  // Create dense RHS and solution vectors.
  cudssMatrix_t matB, matX;
  CheckCudss(cudssMatrixCreateDn(&matB, n, 1, n, d_b, CUDA_R_64F,
                                  CUDSS_LAYOUT_COL_MAJOR), "matB");
  CheckCudss(cudssMatrixCreateDn(&matX, n, 1, n, d_x, CUDA_R_64F,
                                  CUDSS_LAYOUT_COL_MAJOR), "matX");

  // Solver config + data.
  cudssConfig_t config;
  cudssData_t data;
  CheckCudss(cudssConfigCreate(&config), "config");
  CheckCudss(cudssDataCreate(handle, &data), "data");

  // Analysis phase (not timed).
  CheckCudss(cudssExecute(handle, CUDSS_PHASE_ANALYSIS, config, data,
                           matA, matX, matB), "analysis");
  cudaStreamSynchronize(stream);

  double total_factor = 0, total_solve = 0;
  for (int t = 0; t < warmup + trials; ++t) {
    cudaStreamSynchronize(stream);
    auto t0 = std::chrono::high_resolution_clock::now();
    CheckCudss(cudssExecute(handle, CUDSS_PHASE_FACTORIZATION, config, data,
                             matA, matX, matB), "factor");
    cudaStreamSynchronize(stream);
    auto t1 = std::chrono::high_resolution_clock::now();
    CheckCudss(cudssExecute(handle, CUDSS_PHASE_SOLVE, config, data,
                             matA, matX, matB), "solve");
    cudaStreamSynchronize(stream);
    auto t2 = std::chrono::high_resolution_clock::now();

    if (t >= warmup) {
      total_factor +=
          std::chrono::duration<double, std::milli>(t1 - t0).count();
      total_solve +=
          std::chrono::duration<double, std::milli>(t2 - t1).count();
    }
  }

  Eigen::VectorXd x_sol(n);
  CheckCuda(cudaMemcpy(x_sol.data(), d_x, n * sizeof(double),
                        cudaMemcpyDeviceToHost), "cp");
  double rel_err = (x_sol - prob.x_true).norm() / prob.x_true.norm();

  // Cleanup.
  cudssMatrixDestroy(matA);
  cudssMatrixDestroy(matB);
  cudssMatrixDestroy(matX);
  cudssDataDestroy(handle, data);
  cudssConfigDestroy(config);
  cudssDestroy(handle);
  cudaStreamDestroy(stream);
  cudaFree(d_row);
  cudaFree(d_col);
  cudaFree(d_val);
  cudaFree(d_b);
  cudaFree(d_x);

  return {0.0, total_factor / trials, total_solve / trials, rel_err};
}

// ============================================================================
// CPU tree solver (reference)
// ============================================================================

TimingResult BenchCpuTreeSolver(const BenchProblem& prob, int warmup,
                                int trials) {
  const int n = prob.A.cols();
  int total_rows = prob.A.rows();
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(total_rows);

  double total_factor = 0, total_solve = 0;
  Eigen::VectorXd x_cpu;

  for (int t = 0; t < warmup + trials; ++t) {
    auto slc = std::make_unique<SparseLinearConstraint>(prob.A, b_zero);
    std::set<int> vs;
    for (const auto& s : slc->row_supports()) vs.insert(s.begin(), s.end());
    std::vector<int> av(vs.begin(), vs.end());
    ConstraintManager cm(n);
    auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
        std::move(slc), av);
    cm.AddCustomAssembler(assembler.get());
    SolverConfiguration cfg;

    auto t0 = std::chrono::high_resolution_clock::now();
    auto solver = MakeTreeSolver(&cm, cfg);
    solver->AssembleAndFactor();
    auto t1 = std::chrono::high_resolution_clock::now();
    x_cpu = solver->Solve(prob.rhs);
    auto t2 = std::chrono::high_resolution_clock::now();

    if (t >= warmup) {
      total_factor +=
          std::chrono::duration<double, std::milli>(t1 - t0).count();
      total_solve +=
          std::chrono::duration<double, std::milli>(t2 - t1).count();
    }
  }

  double rel_err = (x_cpu - prob.x_true).norm() / prob.x_true.norm();
  return {0.0, total_factor / trials, total_solve / trials, rel_err};
}

}  // namespace
}  // namespace conex

int main() {
  using namespace conex;

  printf("Block-Arrow: GPU Tree Solver vs cuDSS vs cuSOLVER-Sp vs CPU\n");
  printf("============================================================\n\n");

  struct Config {
    int num_blocks, block_size, sep_size, rows_per_block;
  };

  std::vector<Config> configs = {
      // Single dense block — sanity check.
      {1, 100, 0, 120},
      {1, 400, 0, 480},
      {1, 1600, 0, 1920},
      // Block diagonal (no separator coupling).
      {10, 100, 0, 120},
      {20, 100, 0, 120},
      {50, 100, 0, 120},
      {100, 100, 0, 120},
      {10, 200, 0, 240},
      {20, 200, 0, 240},
      {50, 200, 0, 240},
      // Block-arrow with separators.
      {10, 100, 20, 120},
      {20, 100, 20, 120},
      {50, 100, 20, 120},
      {10, 200, 20, 240},
      {5, 400, 20, 480},
  };

  int warmup = 2, trials = 5;

  printf("%-5s %-5s %-4s %-6s | %-28s | %-19s | %-19s | %-19s\n",
         "blks", "bsz", "sep", "n",
         "     GPU tree (ms)        ", "   cuDSS (ms)    ",
         " cuSOLVER-Sp (ms)", "    CPU (ms)     ");
  printf("%-5s %-5s %-4s %-6s | %7s %7s %7s  | %8s %8s  | %8s %8s  | %8s %8s\n",
         "", "", "", "",
         "asm", "factor", "solve",
         "factor", "solve",
         "factor", "solve",
         "factor", "solve");
  printf("%s\n", std::string(117, '-').c_str());

  for (const auto& cfg : configs) {
    int n = cfg.num_blocks * cfg.block_size + cfg.sep_size;
    auto prob = MakeBlockArrow(cfg.num_blocks, cfg.block_size, cfg.sep_size,
                               cfg.rows_per_block);

    auto gpu = BenchTreeSolver(prob, warmup, trials);
    auto dss = BenchCuDSS(prob, warmup, trials);
    auto cusp = BenchCuSolverSp(prob, warmup, trials);
    auto cpu = BenchCpuTreeSolver(prob, warmup, trials);

    printf("%-5d %-5d %-4d %-6d | %6.2f  %6.2f  %6.2f  | %7.2f  %7.2f  | %7.2f  %7.2f  | %7.2f  %7.2f\n",
           cfg.num_blocks, cfg.block_size, cfg.sep_size, n,
           gpu.assemble_ms, gpu.factor_ms, gpu.solve_ms,
           dss.factor_ms, dss.solve_ms,
           cusp.factor_ms, cusp.solve_ms,
           cpu.factor_ms, cpu.solve_ms);

    printf("  rel_err: gpu=%.1e  dss=%.1e  cusp=%.1e  cpu=%.1e\n",
           gpu.rel_err, dss.rel_err, cusp.rel_err, cpu.rel_err);
  }

  printf("\nDone.\n");
  return 0;
}
