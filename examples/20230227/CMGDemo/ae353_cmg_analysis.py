import sympy as sym
import numpy as np
from sympy.physics import mechanics

def get_model(params):

    ## DEFINE PARAMETERS IN SYMBOLIC FORM

    # Principal moments of inertia
    J_1x, J_1y, J_1z = sym.symbols('J_1x, J_1y, J_1z')
    J_2x, J_2y, J_2z = sym.symbols('J_2x, J_2y, J_2z')
    J_3x, J_3y, J_3z = sym.symbols('J_3x, J_3y, J_3z')

    # Moment of inertia matrices in body-fixed reference frames
    J1 = sym.Matrix.diag(J_1x, J_1y, J_1z)
    J2 = sym.Matrix.diag(J_2x, J_2y, J_2z)
    J3 = sym.Matrix.diag(J_3x, J_3y, J_3z)

    # Spar length
    r = sym.symbols('r')

    # Load mass
    m = sym.symbols('m')

    # Acceleration of gravity
    g = sym.symbols('g')

    ## DEFINE VARIABLES

    # Time
    t = sym.Symbol('t')

    # Joint angles:
    q1, q2, q3 = mechanics.dynamicsymbols('q1, q2, q3')

    # Joint velocities
    v1 = q1.diff(t)
    v2 = q2.diff(t)
    v3 = q3.diff(t)

    # Joint accelerations
    a1 = v1.diff(t)
    a2 = v2.diff(t)
    a3 = v3.diff(t)

    # Torques:
    tau2, tau3 = sym.symbols('tau2, tau3')

    ## DERIVE EOMS

    c1 = sym.cos(q1)
    s1 = sym.sin(q1)
    R_p_in_w = sym.Matrix([[c1, -s1, 0], [s1, c1, 0], [0, 0, 1]])

    c2 = sym.cos(q2)
    s2 = sym.sin(q2)
    R_g_in_p = sym.Matrix([[1, 0, 0], [0, c2, -s2], [0, s2, c2]])

    w1 = sym.Matrix([[0], [0], [v1]])
    w2 = R_p_in_w.T * w1 + sym.Matrix([[v2], [0], [0]])
    w3 = R_g_in_p.T * w2 + sym.Matrix([[0], [-v3], [0]])
    
    p = R_p_in_w * sym.Matrix([-r, 0, 0])
    v = p.diff(t)

    T = ((w1.T * J1 * w1) + (w2.T * J2 * w2) + (w3.T * J3 * w3) + (v.T * m * v)) / 2
    V = m * g * (sym.Matrix([1, 0, 0]).T * p)
    L = sym.simplify(T - V)

    EOM = L.jacobian([v1, v2, v3]).diff(t) - L.jacobian([q1, q2, q3]) - sym.Matrix([0, tau2, tau3]).T

    sol = sym.solve(EOM, [a1, a2, a3])
    h = sym.together(sym.simplify(sym.Matrix([sol[a1], sol[a2], sol[a3]]), full=True))

    h = h[0:2, 0].subs(tau3, 0)

    # Standard form
    f_symparams = sym.Matrix.vstack(sym.Matrix([v1, v2]), h)

    # Standard form without dynamic symbols
    qq1, qq2, vv1, vv2, v_rotor = sym.symbols('qq1, qq2, vv1, vv2, v_rotor')
    f_symparams = mechanics.msubs(f_symparams, {
        q1: qq1,
        q2: qq2,
        v1: vv1,
        v2: vv2,
        v3: v_rotor,
    })
    q1, q2, v1, v2 = sym.symbols('q1, q2, v1, v2')
    f_symparams = f_symparams.subs([
        (qq1, q1),
        (qq2, q2),
        (vv1, v1),
        (vv2, v2),
    ])

    f = sym.simplify(sym.nsimplify(f_symparams.subs([
        (J_1z, 0.5),
        (J_2x, 0.001),
        (J_2z, 0.001),
        (J_3x, 0.01),
        (J_3y, 0.01),
        (J_3z, 0.01),
        (m, 1.),
        (r, 2.),
        (g, params['g']),
        (v_rotor, 500.),
    ]), rational=True))

    ## DERIVE STATE-SPACE MODEL

    q1e = params['q1e']
    q2e = params['q2e']
    v1e = params['v1e']
    v2e = params['v2e']
    tau2e = params['taue']

    # Verify that this is actually an equilibrium point
    f_num = sym.lambdify([q1, q2, v1, v2, tau2], f)
    if not np.allclose(f_num(q1e, q2e, v1e, v2e, tau2e), 0.):
        raise Exception('INVALID EQUILIBRIUM POINT!')
    
    # Find A and B in symbolic form
    A_sym = f.jacobian([q1, q2, v1, v2])
    B_sym = f.jacobian([tau2])

    # Create lambda functions to allow numerical evaluation of A and B
    A_num = sym.lambdify([q1, q2, v1, v2, tau2], A_sym)
    B_num = sym.lambdify([q1, q2, v1, v2, tau2], B_sym)

    # Find A and B in numeric form (making sure the result is floating-point)
    A = A_num(q1e, q2e, v1e, v2e, tau2e).astype(float)
    B = B_num(q1e, q2e, v1e, v2e, tau2e).astype(float)

    return A, B