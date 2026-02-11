EIGEN ?= $(EIGEN_DIR)
ifeq ($(strip $(EIGEN)),)
ifneq ("$(wildcard ../bazel-conex/external/+http_archive+eigen/Eigen)","")
EIGEN := ../bazel-conex/external/+http_archive+eigen
else ifneq ("$(wildcard /usr/include/eigen3/Eigen)","")
EIGEN := /usr/include/eigen3
endif
endif
CONEX=../

CPPFLAGS =-O3 -fPIC -std=c++17 -fdata-sections -ffunction-sections -DNDEBUG -DEIGEN_NO_DEBUG -I$(CONEX)
ifneq ($(strip $(EIGEN)),)
CPPFLAGS += -I$(EIGEN)
endif

ifeq ($(strip $(EIGEN)),)
$(error Could not find Eigen headers. Set EIGEN_DIR to an Eigen include root)
endif

USE_BLAS ?= 0
CPPFLAGS_LIBRARY=$(CPPFLAGS)
LFLAGS=
ifeq ($(USE_BLAS),1)
CPPFLAGS_LIBRARY += -DEIGEN_USE_BLAS
LFLAGS += -lblas
endif

# MKL
#CPPFLAGS_LIBRARY=$(CPPFLAGS) -DEIGEN_USE_MKL_ALL -I$(MKLROOT)/include 
#LFLAGS= -Wl,--start-group ${MKLROOT}/lib/intel64/libmkl_intel_lp64.a ${MKLROOT}/lib/intel64/libmkl_gnu_thread.a ${MKLROOT}/lib/intel64/libmkl_core.a -Wl,--end-group -lpthread -lm -ldl -fopenmp

all: libconex.so

SOURCES = approximate_eigenvalues.cc \
	  block_triangular_operations.cc \
	  clique_ordering.cc \
	  clique_ordering_utils.cc \
	  constraint_manager.cc \
	  conjugate_gradient_solvers.cc \
	  cone_program.cc \
	  dense_lmi_constraint.cc \
	  divergence.cc \
	  exponential_map.cc \
	  equality_constraint.cc \
	  exponential_map_pade.cc \
	  hermitian_psd.cc \
	  json_parser.cc \
	  kkt_solver.cc \
	  kkt_subsystem.cc \
	  kkt_simplicial_solver.cc \
	  kkt_tree_solver.cc \
	  kkt_solver_factory.cc \
	  linear_constraint.cc \
	  psd_constraint.cc \
	  quadratic_cone_constraint.cc \
	  self_dual_embedding.cc \
	  soc_constraint.cc \
	  serialize.cc \
	  static_subsystem.cc \
	  supernodal_solver.cc \
	  supernodal_assembler.cc \
	  triangular_matrix_workspace.cc \
	  tree_utils.cc \
	  quadratic_cost.cc \
	  jordan_matrix_algebra.cc \

OBJECTS := $(SOURCES:.cc=.o)

%.o : $(CONEX)/conex/%.cc $(CONEX)/conex/%.h
	$(CXX) -c $(CPPFLAGS_LIBRARY) -fvisibility=hidden $< 

libsource: $(SOURCES:.cc=.o)

conex.o: conex.h conex.cc
	$(CXX) -c $(CPPFLAGS_LIBRARY) conex.cc

conex_qp.o: conex.h conex_qp.cc
	$(CXX) -c $(CPPFLAGS_LIBRARY) conex_qp.cc

libconex.so:  conex.o conex_qp.o libsource 
	$(CXX) -static-libstdc++ -Wl,--exclude-libs,ALL -Wl,--gc-sections -shared -o libconex.so conex.o conex_qp.o $(OBJECTS) $(QP_OBJECTS) $(LFLAGS)
main: libconex.so
	$(CXX) -L./ test/test_app.cc -lconex 

.PHONY: clean

clean:
	rm -f *.so
	rm -f *.o
	rm -f *.out
