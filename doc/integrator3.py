import jax
import jax.numpy as jnp
import numpy as np

# Force double precision for accurate truncation error analysis
jax.config.update("jax_enable_x64", True)

# =====================================================================
# 1. Manifold Definition: The Exponential Cone
# =====================================================================
def phi_exp_cone(x_vec):
    x, y, z = x_vec[0], x_vec[1], x_vec[2]
    s = y * jnp.log(z / y) - x
    return -jnp.log(s) - jnp.log(y) - jnp.log(z)

def in_domain_exp_cone(x_vec):
    x, y, z = x_vec[0], x_vec[1], x_vec[2]
    return (y > 0) and (z > 0) and ((y * jnp.log(z / y) - x) > 0)

def initial_exp_cone():
    return jnp.array([0.0, 1.0, jnp.exp(1.0)])

def phi_linear_cone(x_vec):
    x, y, z = x_vec[0], x_vec[1], x_vec[2]
    return -jnp.log(x) - jnp.log(y) - jnp.log(z)

def in_domain_linear_cone(x_vec):
    x, y, z = x_vec[0], x_vec[1], x_vec[2]
    return (x > 0) and (y > 0) and  (z > 0)

def initial_linear_cone():
    return jnp.array([3.0, 1.0, 2])

def in_domain(x):
    return in_domain_linear_cone(x)

def phi(x):
    return phi_linear_cone(x)

def initial_point():
    return initial_linear_cone()

grad_phi = jax.jit(jax.grad(phi))
hess_phi = jax.jit(jax.hessian(phi))
d3_phi   = jax.jit(jax.jacobian(hess_phi))


@jax.jit
def local_norm(x, v):
    H = hess_phi(x)
    return jnp.sqrt(jnp.dot(v, jnp.dot(H, v)))

# =====================================================================
# 2. Universal Newton Solver with Custom Domain Check
# =====================================================================
def solve_implicit(residual_fn, jacobian_fn, x_guess, domain_check_fn, tol=1e-12, max_iter=30):
    x_k = jnp.array(x_guess)
    for i in range(max_iter):
        R = residual_fn(x_k)
        if jnp.linalg.norm(R) < tol:
            break
            
        J = jacobian_fn(x_k)
        dx = jnp.linalg.solve(J, -R)
        
        # Backtracking line search
        alpha = 1.0
        for _ in range(20):
            x_next = x_k + alpha * dx
            if domain_check_fn(x_next):
                break
            alpha *= 0.5
            
        x_k = x_k + alpha * dx
    return x_k

# =====================================================================
# 3. Integrator 1: Primal Midpoint (Your Original)
# =====================================================================
def step_primal_midpoint(x0, v0, h):
    x_half = x0 + (h / 2.0) * v0
    lam_0 = grad_phi(x0)
    lam_half = grad_phi(x_half)
    lam_1 = 2.0 * lam_half - lam_0
    
    @jax.jit
    def res(x): return grad_phi(x) - lam_1
    @jax.jit
    def jac(x): return hess_phi(x)
    
    x1 = solve_implicit(res, jac, x0 + h * v0, domain_check_fn=in_domain)
    v1 = (x1 - x0) / h
    return x1, v1

# =====================================================================
# 4. Integrator 2: Dual Midpoint
# =====================================================================
def step_dual_midpoint(x0, v0, h):
    lam_0 = grad_phi(x0)
    H0 = hess_phi(x0)
    w0 = H0 @ v0
    
    lam_half = lam_0 + (h / 2.0) * w0
    
    @jax.jit
    def res(x): return grad_phi(x) - lam_half
    @jax.jit
    def jac(x): return hess_phi(x)
    
    x_half = solve_implicit(res, jac, x0 + (h / 2.0) * v0, domain_check_fn=in_domain)
    x1 = 2.0 * x_half - x0
    
    # Velocity update
    lam_1 = grad_phi(x1)
    H1 = hess_phi(x1)
    w1 = (lam_1 - lam_0) / h
    v1 = jnp.linalg.solve(H1, w1)
    return x1, v1

# =====================================================================
# 5. Integrator 3: Symmetric Method
# =====================================================================
def step_symmetric(x0, v0, h):
    lam_0 = grad_phi(x0)
    H0 = hess_phi(x0)
    w0 = H0 @ v0
    target = lam_0 + H0 @ x0 + 2.0 * h * w0
    
    @jax.jit
    def res(x): return grad_phi(x) + H0 @ x - target
    @jax.jit
    def jac(x): return hess_phi(x) + H0
    
    x1 = solve_implicit(res, jac, x0 + h * v0, domain_check_fn=in_domain)
    
    lam_1 = grad_phi(x1)
    H1 = hess_phi(x1)
    w1 = (lam_1 - lam_0 + H1 @ (x1 - x0)) / (2.0 * h)
    v1 = jnp.linalg.solve(H1, w1)
    return x1, v1
# =====================================================================
# 6. Integrator 4: Yoshida Composition (4th-Order Geometric)
# =====================================================================
def step_yoshida4_symmetric(x0, v0, h):
    # Yoshida weights
    cbrt2 = 2.0 ** (1.0 / 3.0)
    w1 = 1.0 / (2.0 - cbrt2)
    w0 = 1.0 - 2.0 * w1
    
    # Three sub-steps using the strictly time-symmetric primitive
    x1, v1 = step_symmetric(x0, v0, w1 * h)       # Forward
    x2, v2 = step_symmetric(x1, v1, w0 * h)       # Backward (large)
    x3, v3 = step_symmetric(x2, v2, w1 * h)       # Forward
    
    return x3, v3

# =====================================================================
# 7. Integrator 5: Yoshida Composition (6th-Order Geometric)
# =====================================================================
def step_yoshida6_symmetric(x0, v0, h):
    # Yoshida weights for 4th -> 6th order
    fifth_root_2 = 2.0 ** (1.0 / 5.0)
    w1 = 1.0 / (2.0 - fifth_root_2)
    w0 = 1.0 - 2.0 * w1
    
    # Three sub-steps using the strictly time-symmetric 4th-order primitive
    x1, v1 = step_yoshida4_symmetric(x0, v0, w1 * h)       # Forward
    x2, v2 = step_yoshida4_symmetric(x1, v1, w0 * h)       # Backward
    x3, v3 = step_yoshida4_symmetric(x2, v2, w1 * h)       # Forward
    
    return x3, v3



# =====================================================================
# 7. Ground Truth Integrator: Exact ODE via RK4
# =====================================================================
@jax.jit
def geodesic_accel(x, v):
    H = hess_phi(x)
    T = d3_phi(x)
    T_vv = jnp.einsum('ijk,j,k->i', T, v, v)
    return -0.5 * jnp.linalg.solve(H, T_vv)

def step_rk4_exact(x0, v0, h, sub_steps=2000):
    dt = h / sub_steps
    x, v = x0, v0
    for _ in range(sub_steps):
        k1_x = v
        k1_v = geodesic_accel(x, v)
        k2_x = v + 0.5 * dt * k1_v
        k2_v = geodesic_accel(x + 0.5 * dt * k1_x, k2_x)
        k3_x = v + 0.5 * dt * k2_v
        k3_v = geodesic_accel(x + 0.5 * dt * k2_x, k3_x)
        k4_x = v + dt * k3_v
        k4_v = geodesic_accel(x + dt * k3_x, k4_x)
        
        x = x + (dt / 6.0) * (k1_x + 2*k2_x + 2*k3_x + k4_x)
        v = v + (dt / 6.0) * (k1_v + 2*k2_v + 2*k3_v + k4_v)
    return x, v

# =====================================================================
# 8. Simulation & Comparison Loop
# =====================================================================
if __name__ == "__main__":
    x0 = initial_point() 
    v0 = jnp.array([0.1, 0.2, -0.1])
    
    speed_0 = local_norm(x0, v0)
    print(f"Initial Local Speed ||v0||_x0 : {speed_0:.8f}\n")
    
    step_sizes = [0.1, 0.05, 0.025, 0.0125]
    
    print(f"{'h':<8} | {'P-Mid (O(h^2))':<15} | {'D-Mid (O(h^2))':<15} | {'Symm (O(h^2))':<15} | {'Yoshida4 (O(h^4))':<15} | {'Yoshida6 (O(h^6))':<15} | {'RK4 Speed Err':<15}")
    print("-" * 92)
    
    for h in step_sizes:
        # Ground Truth
        x_ex, v_ex = step_rk4_exact(x0, v0, h)
        speed_err = jnp.abs(local_norm(x_ex, v_ex) - speed_0)
        
        # O(h^2) Integrators
        x_pmid, _ = step_primal_midpoint(x0, v0, h)
        x_dmid, _ = step_dual_midpoint(x0, v0, h)
        x_sym, _  = step_symmetric(x0, v0, h)
        
        # O(h^4) Integrator
        x_y4, _  = step_yoshida4_symmetric(x0, v0, h)
        x_y6, _  = step_yoshida6_symmetric(x0, v0, h)
        
        # Errors
        err_pmid = jnp.linalg.norm(x_pmid - x_ex)
        err_dmid = jnp.linalg.norm(x_dmid - x_ex)
        err_sym  = jnp.linalg.norm(x_sym - x_ex)
        err_y4  = jnp.linalg.norm(x_y4 - x_ex)
        err_y6  = jnp.linalg.norm(x_y6 - x_ex)
        
        print(f"{h:<8.4f} | {err_pmid:<15.4e} | {err_dmid:<15.4e} | {err_sym:<15.4e} | {err_y4:<15.4e} | {err_y6:<15.4e}| {speed_err:<15.4e}")
