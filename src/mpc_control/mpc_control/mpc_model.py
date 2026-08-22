"""Diff-drive MPC problem definition, solved with acados.

Run once to generate the C solver:

    ros2 run mpc_control build_solver
"""

import math

import numpy as np

# Horizon
N = 20            # prediction steps
TF = 2.0          # horizon length in seconds -> dt = 0.1 s

# Robot limits, matching nav.yaml (vx_min / vx_max / wz_max)
V_MIN = -0.35
V_MAX = 0.30
W_MAX = 1.9

JSON_FILE = 'acados_ocp_diff_drive.json'


def unwrap(angle, near):
    """Shift angle by whole turns so it lands within pi of `near`."""
    return near + math.atan2(math.sin(angle - near), math.cos(angle - near))


def diff_drive_model():
    """Unicycle model: state [x, y, theta], input [v, w]."""
    import casadi as ca
    from acados_template import AcadosModel

    px = ca.SX.sym('px')
    py = ca.SX.sym('py')
    theta = ca.SX.sym('theta')
    x = ca.vertcat(px, py, theta)

    v = ca.SX.sym('v')
    w = ca.SX.sym('w')
    u = ca.vertcat(v, w)

    f_expl = ca.vertcat(v * ca.cos(theta),
                        v * ca.sin(theta),
                        w)

    model = AcadosModel()
    model.name = 'diff_drive'
    model.x = x
    model.u = u
    model.f_expl_expr = f_expl
    model.xdot = ca.SX.sym('xdot', 3)
    model.f_impl_expr = model.xdot - f_expl
    return model


def build_solver(json_file=JSON_FILE, nlp_solver='SQP_RTI', max_iter=1):
    """Create the acados OCP solver (generates C code on first call)."""
    from acados_template import AcadosOcp, AcadosOcpSolver

    ocp = AcadosOcp()
    ocp.model = diff_drive_model()
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = TF

    nx = 3
    nu = 2

    # Least-squares cost on [x, y, theta, v, w]; terminal cost on [x, y, theta]
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])
    ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
    ocp.cost.Vx_e = np.eye(nx)

    # Yaw is tracked, so the robot turns to face the path instead of
    # crabbing towards it. The reference yaw must be unwrapped near the
    # current heading before it is passed in -- see unwrap() below.
    ocp.cost.W = np.diag([10.0, 10.0, 2.0, 0.1, 0.05])
    ocp.cost.W_e = np.diag([50.0, 50.0, 5.0])
    ocp.cost.yref = np.zeros(nx + nu)
    ocp.cost.yref_e = np.zeros(nx)

    # Input limits
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([V_MIN, -W_MAX])
    ocp.constraints.ubu = np.array([V_MAX, W_MAX])
    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.integrator_type = 'ERK'
    # SQP_RTI: one warm-started iteration per call, ~0.8 ms worst case.
    # Full SQP was measured at 85-95 ms and never reached status 0 on
    # turns, so it is only useful for offline experiments.
    ocp.solver_options.nlp_solver_type = nlp_solver
    ocp.solver_options.nlp_solver_max_iter = max_iter
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'

    return AcadosOcpSolver(ocp, json_file=json_file)


def main():
    build_solver()
    print('acados solver generated (c_generated_code/)')


if __name__ == '__main__':
    main()
