EIGEN=$(EIGEN_DIR)
CONEX=../

CPPFLAGS =-O3 -fPIC -std=c++17 -fdata-sections -ffunction-sections  

# BLAS 
CPPFLAGS_LIBRARY=$(CPPFLAGS) -I $(EIGEN) -I $(CONEX) -DEIGEN_USE_BLAS -fvisibility=hidden
LFLAGS=-lblas 

# MKL
#CPPFLAGS_LIBRARY=$(CPPFLAGS) -I $(EIGEN) -I $(CONEX) -DEIGEN_USE_MKL_ALL -I$(MKLROOT)/include -fvisibility=hidden
#LFLAGS= -Wl,--start-group ${MKLROOT}/lib/intel64/libmkl_intel_lp64.a ${MKLROOT}/lib/intel64/libmkl_gnu_thread.a ${MKLROOT}/lib/intel64/libmkl_core.a -Wl,--end-group -lpthread -lm -ldl -fopenmp

all: libconex.so

# The source for the cone program solver
SOURCES = approximate_eigenvalues.cc \
	  block_triangular_operations.cc \
	  clique_ordering.cc \
	  cone_program.cc \
	  quadratic_cost.cc \
	  dense_lmi_constraint.cc \
	  divergence.cc \
	  exponential_map.cc \
	  equality_constraint.cc \
	  exponential_map_pade.cc \
	  hermitian_psd.cc \
	  kkt_assembler.cc \
	  kkt_solver.cc \
	  linear_constraint.cc \
	  psd_constraint.cc \
	  quadratic_cone_constraint.cc \
	  soc_constraint.cc \
	  supernodal_solver.cc \
	  sparsity_utils.cc \
	  triangular_matrix_workspace.cc \
	  tree_utils.cc \
	  jordan_matrix_algebra.cc \

OBJECTS := $(SOURCES:.cc=.o)
%.o : $(CONEX)/conex/%.cc
	$(CXX) -c $(CPPFLAGS_LIBRARY) $< 
libsource: $(SOURCES:.cc=.o)


# The source for the qp solver
QP_SOURCES = utils.cc  \
	     logspace_interior_point_method.cc  \
	     two_dimensional_lp.cc  \

%.o : $(CONEX)/conex/quadratic_programs/%.cc
	$(CXX) -c $(CPPFLAGS_LIBRARY) $< 

QP_OBJECTS := $(QP_SOURCES:.cc=.o)
lib_qp_source: $(QP_SOURCES:.cc=.o)

conex.o: conex.h conex.cc
	g++ -c $(CPPFLAGS) -I${EIGEN_DIR} -I $(CONEX) conex.cc

conex_qp.o: conex.h conex_qp.cc
	g++ -c $(CPPFLAGS) -I${EIGEN_DIR} -I $(CONEX) conex_qp.cc

libconex.so:  conex.o conex_qp.o libsource lib_qp_source 
	g++ -static-libstdc++ -Wl,--exclude-libs,ALL -Wl,--gc-sections -shared -o libconex.so conex.o conex_qp.o $(OBJECTS) $(QP_OBJECTS) $(LFLAGS)

main: libconex.so
	g++ -L./ test/test_app.cc -lconex 

.PHONY: clean

clean:
	rm -f *.so
	rm -f *.o
	rm -f *.out
