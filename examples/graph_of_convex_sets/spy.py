import matplotlib.pyplot as plt
import numpy as np 

def my_spy(ax, matrix, title):
    #title = ""
    ax.spy(matrix, precision=0.1, markersize=5)
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    #ax.set_title(title)# + "  " + str(np.count_nonzero(matrix)))
    ax.set_title(title + "  " + str(np.count_nonzero(matrix)))
    return ax

def load_matrix(filename):
    x = np.loadtxt(open(filename, "rb"), delimiter=",", skiprows=0)
    np.array(x)
    n = int(np.sqrt(x.shape[0])); x = x.reshape((n, n))
    return x
  
#path = '/home/frank/conex/bazel-out/k8-fastbuild/bin/examples/graph_of_convex_sets/gcs_solver_test.runfiles/conex/'



def MakePlot(folder):
    path = '/home/frank/conex/bazel-out/k8-dbg/bin/examples/graph_of_convex_sets/gcs_solver_test.runfiles/conex/' + folder + '/'
    file_sparsity_cholesky_factor_topological = path + "sparsity_cholesky_factor_topological.txt"
    #file_sparsity_cholesky_factor_topological_1 = path + "sparsity_cholesky_factor_topological_1.txt"

    file_sparsity_cholesky_factor_amd = path + "sparsity_cholesky_factor_amd.txt"

    file_kkt_matrix = path + "kkt_matrix.txt"
    file_kkt_matrix_amd_order = path + "kkt_matrix_amd_order.txt"
    file_kkt_matrix_topological_order = path + "kkt_matrix_topological_order.txt"

    factor_amd = load_matrix(file_sparsity_cholesky_factor_amd)
    factor_top = load_matrix(file_sparsity_cholesky_factor_topological)

    kkt_matrix = load_matrix(file_kkt_matrix)
    kkt_matrix_amd_order = load_matrix(file_kkt_matrix_amd_order)
    kkt_matrix_topological_order = load_matrix(file_kkt_matrix_topological_order)

    fig, axs = plt.subplots(2, 2)
    ax1 = axs[0][0]
    ax2 = axs[0][1]

    ax1 = my_spy(ax1, kkt_matrix_topological_order, "Permuted KKT Matrix ")
    ax2 = my_spy(ax2, factor_top, "Cholesky Factor")

    ax3 = axs[1][0]
    ax4 = axs[1][1]
    ax3 = my_spy(ax3, kkt_matrix_amd_order, "Permuted KKT Matrix")
    ax4 = my_spy(ax4, factor_amd, "Cholesky Factor")

    #plt.show()
    plt.savefig('sparsity'+folder+'.png')

#MakePlot('path')
#MakePlot('random')
#MakePlot('nonunique')
#MakePlot('simple')
#MakePlot('simple')

def MakePlot2(folder):
    path = '/home/frank/conex/bazel-out/k8-dbg/bin/examples/graph_of_convex_sets/edge_topological_order_test.runfiles/conex/' + folder + '/'
    file_sparsity_cholesky_factor_topological = path + "sparsity_cholesky_factor_edge_top.txt"
    factor_top = load_matrix(file_sparsity_cholesky_factor_topological)

#    file_kkt_matrix = path + "kkt_matrix.txt"
    file_kkt_matrix_amd_order = path + "kkt_matrix_amd_order.txt"
    file_kkt_matrix_edge_topological_order = path + "kkt_matrix_edge_top.txt"
    file_kkt_matrix_topological_order = path + "kkt_matrix_topological_order.txt"

    file_sparsity_cholesky_factor_amd = path + "sparsity_cholesky_factor_amd.txt"
    factor_amd = load_matrix(file_sparsity_cholesky_factor_amd)

    kkt_matrix_topological_order = load_matrix(file_kkt_matrix_topological_order)
    kkt_matrix_edge_top = load_matrix(file_kkt_matrix_edge_topological_order)

    fig, axs = plt.subplots(2, 2)
    ax1 = axs[0][0]
    ax2 = axs[0][1]
    ax3 = axs[1][0]
    ax4 = axs[1][1]

    ax1 = my_spy(ax1, kkt_matrix_topological_order, "KKT Matrix top ")
    ax2 = my_spy(ax2, factor_top, "Cholesky Factor")
    ax3 = my_spy(ax3, factor_amd, "AMD Factor")
    ax4 = my_spy(ax4, kkt_matrix_edge_top, "KKT Matrix Edge Top")


    plt.show()
    #plt.savefig('sparsity'+folder+'.png')

MakePlot2('nonunique')
