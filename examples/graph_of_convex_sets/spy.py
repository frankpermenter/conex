import matplotlib.pyplot as plt
import numpy as np


def my_spy(ax, matrix, title):
    ax.spy(matrix, precision=0.1, markersize=5)
    ax.set_yticklabels([])
    ax.set_xticklabels([])
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


    factor_amd = load_matrix(file_sparsity_cholesky_factor_amd)
    factor_top = load_matrix(file_sparsity_cholesky_factor_topological)
    #factor_top_1 = load_matrix(file_sparsity_cholesky_factor_topological_1)
    kkt_matrix = load_matrix(file_kkt_matrix)
    kkt_matrix_amd_order = load_matrix(file_kkt_matrix_amd_order)

    fig, axs = plt.subplots(2, 2)
    ax1 = axs[0][0]
    ax2 = axs[0][1]
    ax3 = axs[1][0]
    ax4 = axs[1][1]

    ax1 = my_spy(ax1, kkt_matrix, "KKT Matrix")

    ax2 = my_spy(ax2, factor_amd, "AMD Factor")
    #ax2 = my_spy(ax2, factor_top_1, "Top 2. Factor")

    ax3 = my_spy(ax3, factor_top, "Topog. Factor")
    ax4 = my_spy(ax4, kkt_matrix_amd_order, "KKT Matrix AMD")

    plt.show()

MakePlot('path')
MakePlot('random')
MakePlot('nonunique')

