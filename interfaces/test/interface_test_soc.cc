#include "interfaces/conex.h"

#include "gtest/gtest.h"

#define TEST_CONSTRAINT_API(AddConstraintCommand)                              \
  int status;                                                                  \
  void* p = CONEX_CreateConeProgram();                                         \
  status = CONEX_SetNumberOfVariables(p, num_vars);                            \
  status = AddConstraintCommand(p, order, &constraint_id);                     \
  EXPECT_EQ(CONEX_SUCCESS, status);                                            \
                                                                               \
  status = CONEX_UpdateLinearOperator(p, constraint_id, .3, valid_row,         \
                                      valid_col, 0, 0);                        \
  EXPECT_EQ(CONEX_SUCCESS, status);                                            \
                                                                               \
  status = CONEX_UpdateLinearOperator(p, constraint_id, .3, valid_row,         \
                                      valid_col, 0, bad_hyper_complex_dim);    \
  EXPECT_EQ(CONEX_FAILURE, status);                                            \
  status =                                                                     \
      CONEX_UpdateLinearOperator(p, constraint_id, .3, 2, bad_variable, 0, 0); \
  EXPECT_EQ(CONEX_FAILURE, status);                                            \
  status = CONEX_UpdateLinearOperator(p, constraint_id, .3, valid_row,         \
                                      valid_col, bad_column_index, 0);         \
  EXPECT_EQ(CONEX_FAILURE, status);                                            \
                                                                               \
  status = CONEX_UpdateAffineTerm(p, constraint_id, .3, 0, 0, 0);              \
  EXPECT_EQ(CONEX_SUCCESS, status);                                            \
  status = CONEX_UpdateAffineTerm(p, constraint_id, .3, valid_row, 0, 0);      \
  EXPECT_EQ(CONEX_SUCCESS, status);                                            \
                                                                               \
  status = CONEX_UpdateAffineTerm(p, constraint_id, .3, valid_row, 0,          \
                                  bad_hyper_complex_dim);                      \
  EXPECT_EQ(CONEX_FAILURE, status);                                            \
  status = CONEX_UpdateAffineTerm(p, constraint_id, .3, valid_row,             \
                                  bad_column_index, 0);                        \
  EXPECT_EQ(CONEX_FAILURE, status);                                            \
                                                                               \
  CONEX_DeleteConeProgram(p);

TEST(TestSOCPInterface, AddConstraint) {
  void* p = CONEX_CreateConeProgram();
  int constraint_id = 0;
  EXPECT_TRUE(CONEX_NewLorentzConeConstraint(p, 2, &constraint_id) ==
              CONEX_SUCCESS);
  EXPECT_EQ(0, constraint_id);
  EXPECT_TRUE(CONEX_NewLorentzConeConstraint(p, 2, &constraint_id) ==
              CONEX_SUCCESS);
  EXPECT_EQ(1, constraint_id);

  void* null_ptr = NULL;
  EXPECT_TRUE(CONEX_NewLorentzConeConstraint(null_ptr, 2, &constraint_id) ==
              CONEX_FAILURE);

  int bad_order = 0;
  EXPECT_TRUE(CONEX_NewLorentzConeConstraint(p, bad_order, &constraint_id) ==
              CONEX_FAILURE);
  CONEX_DeleteConeProgram(p);
}

TEST(TestSOCPInterface, UpdateConstraint) {
  int num_vars = 4;
  int constraint_id = 0;
  int order = 2;

  int bad_hyper_complex_dim = 1;
  int bad_variable = -1;
  int bad_column_index = 1;
  int valid_row = 2;
  int valid_col = order - 1;

  TEST_CONSTRAINT_API(CONEX_NewLorentzConeConstraint);
}

TEST(TestLPInterface, UpdateConstraint) {
  int num_vars = 2;

  int constraint_id = 0;
  int order = 2; /* num rows*/

  int bad_hyper_complex_dim = 1;
  int bad_variable = -1;
  int bad_column_index = num_vars + 2;

  int valid_row = 1;
  int valid_col = num_vars - 1;

  TEST_CONSTRAINT_API(CONEX_NewLinearInequality);
}
