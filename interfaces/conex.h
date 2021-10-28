#ifndef CONEX_API_H
#define CONEX_API_H
#ifdef __cplusplus
extern "C" {
#endif

typedef int CONEX_STATUS;
enum { CONEX_SUCCESS = 0, CONEX_FAILURE = 1 };

typedef struct {
  int prepare_dual_variables;
  int initialization_mode;
  double inv_sqrt_mu_max;
  double minimum_mu;
  double maximum_mu;
  double divergence_upper_bound;
  int enable_line_search;
  double dinf_upper_bound;
  int final_centering_steps;
  double final_centering_tolerance;
  int initial_centering_steps_warmstart;
  int initial_centering_steps_coldstart;
  double warmstart_abort_threshold;
  int max_iterations;
  int iterative_refinement_iterations;
  double infeasibility_threshold;
  double kkt_error_tolerance;
  int enable_rescaling;
  int kkt_solver;
  // QP solver settings.
  double sqrt_mu_weight;
  double theta_weight;
  double target_duality_gap;
  double theta_truncation_threshold;
  double endgame_rescaling_threshold; 
  double endgame_rescaling_factor; 
} CONEX_SolverConfiguration;

typedef struct {
  double mu;
  int iteration_number;
} CONEX_IterationStats;

typedef struct {
  int iterations;
} CONEX_SolutionStats;

void* CONEX_CreateConeProgram();
void CONEX_DeleteConeProgram(void* program);

int CONEX_AddDenseLinearConstraint(void* program, const double* A, int Ar,
                                   int Ac, const double* c, int cr);

int CONEX_AddLinearInequalities(void* program, const double* A, int Ar, int Ac,
                                const double* lb, int num_lb, const double* ub,
                                int num_ub);

int CONEX_AddQuadraticCost(void* program, const double* A, int Ar, int Ac);
//  Parameters Aarrayr, Aarrayc, cr, cc all equal the
//  order n of LMI.
// TODO(FrankPermenter): update this.
int CONEX_AddDenseLMIConstraint(void* program, const double* Aarray,
                                int Aarrayr, int Aarrayc, int m,
                                const double* cmat, int cr, int cc);

int CONEX_AddSparseLMIConstraint(void* program, const double* Aarray,
                                 int Aarrayr, int Aarrayc, int m,
                                 const double* cmat, int cr, int cc,
                                 const long* vars, int vars_c);

int CONEX_Maximize(void* program, const double* b, int br,
                   const CONEX_SolverConfiguration* config, double* y, int yr);

int CONEX_Solve(void* program, const CONEX_SolverConfiguration* config,
                double* y, int yr);

void CONEX_GetDualVariable(void* program, int i, double* x, int xr, int xc);

int CONEX_GetDualVariableSize(void* program, int i);

void CONEX_SetDefaultOptions(CONEX_SolverConfiguration* config);

void CONEX_GetIterationStats(void* program, CONEX_IterationStats* stats,
                             int iter_num);

CONEX_STATUS CONEX_UpdateLinearOperator(void* program, int constraint,
                                        double value, int variable, int row,
                                        int col, int hyper_complex_dim);

CONEX_STATUS CONEX_NewLinearMatrixInequality(void* program, int order,
                                             int hyper_complex_dim,
                                             int* constraint_id);

CONEX_STATUS CONEX_UpdateAffineTerm(void* program, int constraint, double value,
                                    int row, int col, int hyper_complex_dim);

CONEX_STATUS CONEX_NewLorentzConeConstraint(void* program, int order,
                                            int* constraint_id);

CONEX_STATUS CONEX_NewLinearInequality(void* program, int num_rows,
                                       int* constraint_id);

CONEX_STATUS CONEX_NewQuadraticCost(void* program, int* constraint_id);
CONEX_STATUS CONEX_UpdateQuadraticCostMatrix(void* p, int id, double value,
                                             int row, int col);

CONEX_STATUS CONEX_SetNumberOfVariables(void* program, int m);


int CONEX_QP_Solver(const double* quadratic_cost_matrix,
                    int quadratic_cost_matrix_num_row,
                    int quadratic_cost_matrix_num_col,
                    const double* cost_vector, int num_row_cost_vector,
                    const double* inequality_matrix, int num_row_ineq,
                    int num_col_ineq, const double* inequality_upper_bound,
                    int num_row_ineq_ub, const double* inequality_lower_bound,
                    int num_row_ineq_lb,
                    const CONEX_SolverConfiguration* config_input,
                    double* solution, int num_row,
                    CONEX_SolutionStats* stats);


int CONEX_QP_GetCanonicalProblemData(const double* quadratic_cost_matrix,
                    int quadratic_cost_matrix_num_row,
                    int quadratic_cost_matrix_num_col,
                    const double* cost_vector, int num_row_cost_vector,
                    const double* inequality_matrix, int num_row_ineq,
                    int num_col_ineq, const double* inequality_upper_bound,
                    int num_row_ineq_ub, const double* inequality_lower_bound,
                    int num_row_ineq_lb,
                    int *num_ineq,
                    int *num_eq,
                    double* matrix_A, int num_row_A, int num_col_A,
                    double* vector_b, int num_row_b,
                    double* matrix_B, int num_row_B, int num_col_B,
                    double* vector_d, int num_row_d);




#ifdef __cplusplus
}  // extern "C"
#endif
#endif
