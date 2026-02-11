%module conex

%{
    #define SWIG_FILE_WITH_INIT
    #include "../conex.h"
%}

%include cpointer.i
%pointer_class(int, intp);

%include "typemaps.i"

%apply (double* IN_FARRAY2, int DIM1, int DIM2) {(const double* A, int Ar, int Ac)}
%apply (double* IN_FARRAY2, int DIM1, int DIM2) {(const double* cmat, int cr, int cc)}
%apply (double* IN_FARRAY3, int DIM1, int DIM2, int DIM3) {(const double* Aarray, int Aarrayr, int Aarrayc, int m)}
%apply (double* IN_ARRAY1, int DIM1) {(const double* b, int br)}
%apply (double* IN_ARRAY1, int DIM1) {(const double* c, int cr)}
%apply (double* IN_ARRAY1, int DIM1) {(const double* lb, int num_lb)}
%apply (double* IN_ARRAY1, int DIM1) {(const double* ub, int num_ub)}
%apply (double* INPLACE_ARRAY1, int DIM1) {(double* y, int yr)}
%apply (double* INPLACE_ARRAY1, int DIM1) {(double* x, int xr)}
%apply (long* INPLACE_ARRAY1, int DIM1) {(const long* vars, int vars_c)}
%apply (double* INPLACE_FARRAY2, int DIM1, int DIM2) {(double* x, int xr, int xc)}

%apply (double* IN_FARRAY2, int DIM1, int DIM2) {(const double* quadratic_cost_matrix, int quadratic_cost_matrix_num_row, int quadratic_cost_matrix_num_col)}
%apply (double* INPLACE_ARRAY1, int DIM1) {(const double* cost_vector, int num_row_cost_vector)}
%apply (double* IN_FARRAY2, int DIM1, int DIM2) {(const double* inequality_matrix, int num_row_ineq, int num_col_ineq)}
%apply (double* INPLACE_ARRAY1, int DIM1)  {(const double* inequality_upper_bound, int num_row_ineq_ub)}
%apply (double* INPLACE_ARRAY1, int DIM1) {(const double* inequality_lower_bound, int num_row_ineq_lb)}
%apply (double* INPLACE_ARRAY1, int DIM1) {(double* solution, int num_row)}

%apply (double* INPLACE_ARRAY1, int DIM1) {(double* vector_b, int num_row_b)}
%apply (double* INPLACE_ARRAY1, int DIM1) {(double* vector_d, int num_row_d)}

%apply (double* INPLACE_FARRAY2, int DIM1, int DIM2) {(double* matrix_A, int num_row_A, int num_col_A)}
%apply (double* INPLACE_FARRAY2, int DIM1, int DIM2) {(double* matrix_B, int num_row_B, int num_col_B)}
%include "../conex.h"
