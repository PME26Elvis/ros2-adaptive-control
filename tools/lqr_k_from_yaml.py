#!/usr/bin/env python3
import argparse, yaml, numpy as np
from scipy.linalg import solve_continuous_are

def lqr(A,B,Q,R):
    # 連續 LQR：K = R^{-1} B^T P, 其中 P 解 CARE
    P = solve_continuous_are(A,B,Q,R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml_path")
    ap.add_argument("--ns", default="controller_manager.adaptive_controller.ros__parameters",
                    help="YAML namespace to read")
    args = ap.parse_args()

    with open(args.yaml_path,"r") as f:
        cfg = yaml.safe_load(f)

    # 走 namespace
    ns = cfg
    for key in args.ns.split("."):
        if key not in ns:
            raise KeyError(f"namespace '{args.ns}' missing at '{key}'")
        ns = ns[key]

    dof = int(ns.get("dof",1))
    nx  = 2*dof
    # 讀 A/B（若沒提供則使用 canonical）
    A = np.zeros((nx,nx)); B = np.zeros((nx,dof))
    # A: [0 I; 0 0]
    A[:dof, dof:] = np.eye(dof)
    # B: [0; I]
    B[dof:, : ]   = np.eye(dof)

    if "A_data" in ns:
        Ad = np.array(ns["A_data"], dtype=float)
        if Ad.size != nx*nx: raise ValueError("A_data size mismatch")
        A = Ad.reshape((nx,nx))
    if "B_data" in ns:
        Bd = np.array(ns["B_data"], dtype=float)
        if Bd.size != nx*dof: raise ValueError("B_data size mismatch")
        B = Bd.reshape((nx,dof))

    # Q/R：若未提供，給一個合理的對角（位置權重大於速度）
    if "Q_data" in ns:
        Qd = np.array(ns["Q_data"], dtype=float)
        if Qd.size != nx*nx: raise ValueError("Q_data size mismatch")
        Q = Qd.reshape((nx,nx))
    else:
        Q = np.diag([10.0]*dof + [1.0]*dof)

    if "R_data" in ns:
        Rd = np.array(ns["R_data"], dtype=float)
        if Rd.size != dof*dof: raise ValueError("R_data size mismatch")
        R = Rd.reshape((dof,dof))
    else:
        R = 0.1*np.eye(dof)

    K = lqr(A,B,Q,R)  # shape (dof, 2*dof)
    print("# Paste this K_data back into your YAML under the same ns")
    flat = K.reshape(-1).tolist()
    print("K_data:", [float(f) for f in flat])

if __name__ == "__main__":
    main()
