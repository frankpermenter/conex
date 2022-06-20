EIGEN=$(EIGEN_DIR)
CONEX=../

CPPFLAGS =-O3 -fPIC -std=c++17 -fdata-sections -ffunction-sections -DNDEBUG -DEIGEN_NO_DEBUG -I $(EIGEN) -I $(CONEX) 

# BLAS 
CPPFLAGS_LIBRARY=$(CPPFLAGS) -DEIGEN_USE_BLAS 
LFLAGS=-lblas 

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

libconex.so:  conex.o libsource
	$(CXX) -static-libstdc++ -Wl,--exclude-libs,ALL -Wl,--gc-sections -shared -o libconex.so conex.o $(OBJECTS) $(LFLAGS)

main: libconex.so
	$(CXX) -L./ test/test_app.cc -lconex 

.PHONY: clean

clean:
	rm -f *.so
	rm -f *.o
	rm -f *.out
