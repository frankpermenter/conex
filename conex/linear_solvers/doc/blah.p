import os
import subprocess

tex_content = r"""\documentclass[11pt]{article}
\usepackage[T1]{fontenc}
\usepackage{lmodern}
\usepackage{amsmath, amssymb, amsthm}
\usepackage[margin=1.2in]{geometry}
\usepackage{hyperref}
\usepackage{xcolor}

\newtheorem{theorem}{Theorem}
\newtheorem{lemma}{Lemma}
\newtheorem{definition}{Definition}

\title{\textbf{Geodesic Divergence Curvature,\\ Torsional Decomposition, and the Commutator Bound}}
\author{}
\date{\vspace{-5ex}}

\begin{document}
\maketitle

\section{Kinematic Bound on Divergence Curvature}
We study a curve $z(t)$ tracing a geodesic on a Hessian manifold defined by a self-concordant barrier $\phi$. 

\begin{theorem}
Let $z(t)$ be a geodesic parameterized by arc length with constant kinetic energy $E = \|\dot{z}\|_{H(z)}^2$. Let $z_*$ be an arbitrary target point. Then the second time derivative of the generalized Jeffreys divergence $V(z) = \langle z - z_*, \nabla \phi(z) - \nabla \phi(z_*) \rangle$ is given by:
\begin{equation}
    \ddot{V} = 2E - \langle \ddot{z}, U \rangle_z
\end{equation}
where $U = (z - z_*) - H(z)^{-1}(\nabla \phi(z) - \nabla \phi(z_*))$ is the exact aggregated residual vector, and the inner product is in the local metric $\langle a, b \rangle_z = a^T H(z) b$.
\end{theorem}

\begin{proof}
Taking the first time derivative of $V(z)$ along the geodesic $z(t)$:
\begin{equation}
    \dot{V} = \langle \dot{z}, \nabla \phi(z) - \nabla \phi(z_*) \rangle + \langle z - z_*, H(z)\dot{z} \rangle
\end{equation}
Taking the second time derivative yields:
\begin{align}
    \ddot{V} &= \langle \ddot{z}, \nabla \phi(z) - \nabla \phi(z_*) \rangle + \langle \dot{z}, H(z)\dot{z} \rangle \nonumber \\
             &\quad + \langle \dot{z}, H(z)\dot{z} \rangle + \langle z - z_*, \dot{H}(z)\dot{z} \rangle + \langle z - z_*, H(z)\ddot{z} \rangle
\end{align}
Recognizing that the kinetic energy is $E = \langle \dot{z}, H(z)\dot{z} \rangle$, we substitute $2E$ and group the acceleration terms:
\begin{equation} \label{eq:ddot_V_expanded}
    \ddot{V} = 2E + \langle \ddot{z}, \nabla \phi(z) - \nabla \phi(z_*) + H(z)(z - z_*) \rangle + D^3\phi(z)[\dot{z}, \dot{z}, z - z_*]
\end{equation}

By the definition of a geodesic on a Hessian manifold, the acceleration $\ddot{z}$ is driven by the Levi-Civita connection:
\begin{equation}
    \ddot{z} = -\frac{1}{2} H(z)^{-1} D^3\phi(z)[\dot{z}, \dot{z}, \cdot]
\end{equation}
Applying this identity to the vector $w = z - z_*$ rewrites the third derivative term as a local inner product with the acceleration:
\begin{equation}
    D^3\phi(z)[\dot{z}, \dot{z}, z - z_*] = -2 \langle \ddot{z}, H(z)(z - z_*) \rangle
\end{equation}
Substituting this back into Equation \ref{eq:ddot_V_expanded} results in the exact cancellation of one of the primal residual terms:
\begin{align}
    \ddot{V} &= 2E + \langle \ddot{z}, \nabla \phi(z) - \nabla \phi(z_*) + H(z)(z - z_*) \rangle - 2 \langle \ddot{z}, H(z)(z - z_*) \rangle \nonumber \\
             &= 2E + \langle \ddot{z}, \nabla \phi(z) - \nabla \phi(z_*) - H(z)(z - z_*) \rangle
\end{align}
Factoring out $-H(z)$ reveals the exact generalized aggregated residual $U$:
\begin{align}
    \ddot{V} &= 2E - \langle \ddot{z}, H(z) \left[ (z - z_*) - H(z)^{-1}(\nabla \phi(z) - \nabla \phi(z_*)) \right] \rangle \nonumber \\
             &= 2E - \langle \ddot{z}, U \rangle_z
\end{align}
\end{proof}

\section{The Commutator and Torsional Decomposition}
For a general self-concordant barrier $\phi(z)$, the metric-warping operator $M_u(v) = H(z)^{-1} D^3\phi(z)[u, v, \cdot]$ equips the tangent space with a commutative (but generally non-associative) algebraic product $u \bullet v = M_u(v)$. The local geometric square is $u^2 = M_u(u)$. 

We formally define the \textbf{Sectional Curvature Commutator} as the Lie bracket of the multiplication operators:
\begin{equation}
    \mathcal{C}(u) = [M_u, M_{u^2}] = M_u M_{u^2} - M_{u^2} M_u
\end{equation}

By expanding the generalized residual $U$ via the operator algebra along the step $\Delta = z_* - z$, the residual splits strictly into a power-associative component $U_{\mathrm{sym}}$ and a non-associative torsional component $U_{\mathrm{torsion}}$:
\begin{equation}
    U = U_{\mathrm{sym}} + U_{\mathrm{torsion}}
\end{equation}
The decomposition of the inner product yields:
\begin{equation}
    \langle \ddot{z}, U \rangle_z = \langle \ddot{z}, U_{\mathrm{sym}} \rangle_z + \langle \ddot{z}, \mathcal{C}(\Delta)[v] \rangle_z
\end{equation}
where $v$ encapsulates higher-order remainders. The energy leakage in the inner product exists purely because $\mathcal{C}(\Delta) \neq 0$.

\section{Recovering the Symmetric Cone Case and the Exact Formula}
If the barrier possesses the self-scaled symmetry $\phi(-\nabla \phi(z)) = \phi(z) + c$, the operators form a Euclidean Jordan Algebra. The commutator universally vanishes ($\mathcal{C} \equiv 0 \implies U_{\mathrm{torsion}} = 0$), so $U = U_{\mathrm{sym}}$.

In this perfect geometry, we map the residual to an isotropic frame using the quadratic representation $P(z) = H(z)^{-1}$. Let the relative stretch vector be $w = P(z)^{-1/2} z_*$. The primal acceleration maps to $\tilde{\ddot{z}} = -\frac{1}{2}\tilde{\dot{z}}^2$.

The dual-gradient mapping acts as the Jordan inverse. The generalized residual $U = (z - z_*) - P(z)(-\nabla \phi(z) + \nabla \phi(z_*))$ maps identically to the isotropic frame as $\tilde{U}_{sym} = 2\mathbf{1} - w - w^{-1}$. Thus, the inner product evaluates to the spectacularly clean associative formula:
\begin{equation}
    \langle \ddot{z}, U_{\mathrm{sym}} \rangle_z = -\frac{1}{2} \langle \tilde{\dot{z}}^2, 2\mathbf{1} - w - w^{-1} \rangle
\end{equation}
Since $w \approx \mathbf{1} + \Delta w$, a Taylor expansion yields $2\mathbf{1} - w - w^{-1} \approx -(\Delta w)^2$. The symmetric residual flawlessly produces the pure quadratic centering force without higher-order torsion.

\section{Polyhedral Torsion and the Mehrotra Cross-Term}
For the polyhedral cone $Ax > 0$, we lift tangent vectors into the $m$-dimensional scaled slack space using $\tilde{u} = S^{-1}Au$, where $S = \mathrm{diag}(Az)$. Here, $w = S^{-1}s_*$. The geometry is characterized entirely by the orthogonal projection matrix $P$:
\begin{equation}
    P = S^{-1}A (A^T S^{-2} A)^{-1} A^T S^{-1}
\end{equation}
We evaluate the exact polyhedral residual $\tilde{U} = S^{-1}AU$. Applying the inverse Hessian and projecting:
\begin{equation}
    \tilde{U} = (\mathbf{1} - w) + P(\mathbf{1} - w^{-1})
\end{equation}

For the acceleration, lifting the Levi-Civita geodesic equation $\ddot{z} = -\frac{1}{2}H^{-1}D^3\phi$ yields exactly $\tilde{\ddot{z}} = P(\tilde{\dot{z}}^{\circ 2})$. Evaluating the metric inner product $\langle \ddot{z}, U \rangle_z = \tilde{\ddot{z}}^T \tilde{U}$:
\begin{equation}
    \langle \ddot{z}, U \rangle_z = P(\tilde{\dot{z}}^{\circ 2})^T \Big( (\mathbf{1} - w) + P(\mathbf{1} - w^{-1}) \Big)
\end{equation}
Assuming $P(\mathbf{1}) = \mathbf{1}$ and distributing the symmetric projection $P$ perfectly recovers the isotropic trace formula, but critically perturbed by the projection:
\begin{equation}
    \langle \ddot{z}, U \rangle_z = (\tilde{\dot{z}}^{\circ 2})^T P \Big( 2\mathbf{1} - w - w^{-1} \Big)
\end{equation}

The strictness of this formulation isolates the mathematical truth: the underlying force $2\mathbf{1} - w - w^{-1}$ belongs purely to $U_{\mathrm{sym}}$, identical to the Symmetric Cone case. The energy leak is strictly due to the insertion of the projection matrix $P$ evaluating the cross-terms of the Hadamard square, which explicitly embodies the non-zero $\epsilon$-commutator torsion.

\end{document}
"""

with open("geodesic_divergence_unified.tex", "w") as f:
    f.write(tex_content)

result = subprocess.run(["pdflatex", "-interaction=nonstopmode", "geodesic_divergence_unified.tex"], capture_output=True, text=True)
if result.returncode == 0:
    print("[file-tag: geodesic_divergence_unified.pdf]")
else:
    print("Error:", result.stdout)
