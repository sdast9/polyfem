"""RB-09 analytical references (no solver involved).

Block benchmark: a Neo-Hookean block (PolyFEM's form,
W = mu/2 (|F|^2 - 3 - 2 ln J) + lam/2 (ln J)^2, P = mu (F - F^-T) + lam ln J F^-T)
of reference height H and reference cross-section A in homogeneous uniaxial
compression with traction-free lateral faces: F = diag(l, l, lz).
Lateral equilibrium P_xx = 0 gives  mu (l^2 - 1) + lam ln J = 0, J = l^2 lz.
The top reaction (force conjugate to the prescribed top displacement, i.e. the
first Piola traction times the reference area) is R_z = A P_zz(lz).

Hard-contact reference: the bottom face sits on the floor, lz = 1 - (delta - g0)/H
for a top displacement delta after closing the initial gap g0. Barrier-consistent
reference: lz = 1 - (delta - g0 + g)/H with the realized gap g, isolating the
gap error of the barrier from the discretisation/solver error.

Spring benchmark: one DOF x on a linear spring of stiffness k pulled to x_p
below a floor at 0 by the clamped-log barrier kappa * b(x, dhat),
b = -(x - dhat)^2 ln(x/dhat) for 0 < x < dhat (ipc::ClampedLogBarrier, called
on the squared distance in PolyFEM; here written in the gap directly for the
1-DOF scalar problem). The exact equilibrium is the root of
k (x - x_p) + kappa b'(x) = 0 on (0, dhat), bracketed and bisected to 1e-15.
"""
import math


def lame(E, nu):
    return E / (2 * (1 + nu)), E * nu / ((1 + nu) * (1 - 2 * nu))


def lateral_stretch(lz, mu, lam):
    """Solve mu (l^2 - 1) + lam (ln lz + 2 ln l) = 0 for l > 0 (monotone in l)."""
    f = lambda l: mu * (l * l - 1) + lam * (math.log(lz) + 2 * math.log(l))
    lo, hi = 1e-6, 10.0
    assert f(lo) < 0 < f(hi)
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(mid) < 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def uniaxial(lz, E, nu, A=1.0):
    """Homogeneous uniaxial state at axial stretch lz: lateral stretch, ln J,
    P_zz, total top reaction R_z = A P_zz (negative in compression), Cauchy
    sigma_zz = P_zz / l^2 and the energy density."""
    mu, lam = lame(E, nu)
    l = lateral_stretch(lz, mu, lam)
    J = l * l * lz
    lnJ = math.log(J)
    Pzz = mu * (lz - 1 / lz) + lam * lnJ / lz
    W = mu / 2 * (2 * l * l + lz * lz - 3 - 2 * lnJ) + lam / 2 * lnJ * lnJ
    return {'lz': lz, 'l': l, 'J': J, 'P_zz': Pzz, 'R_z': A * Pzz, 'sigma_zz': Pzz / (l * l), 'W': W, 'mu': mu, 'lam': lam}


def reaction_sensitivity(lz, E, nu, A=1.0, h=1e-6):
    """dR_z/dlz by central difference of the exact state (conditioning of the
    reaction with respect to an axial-stretch error such as a residual gap)."""
    return (uniaxial(lz + h, E, nu, A)['R_z'] - uniaxial(lz - h, E, nu, A)['R_z']) / (2 * h)


def block_reference(delta, g0, H, E, nu, A=1.0, gap=0.0):
    """Reference state for a top displacement delta (>0 downwards) with initial
    gap g0 and a realized floor gap `gap` (0 = hard contact)."""
    lz = 1 - (delta - g0 + gap) / H
    state = uniaxial(lz, E, nu, A)
    state.update(delta=delta, g0=g0, H=H, gap=gap, side_displacement=state['l'] - 1)
    return state


def clamped_log(x, dhat):
    if x <= 0:
        return math.inf
    if x >= dhat:
        return 0.0
    return -(x - dhat) ** 2 * math.log(x / dhat)


def clamped_log_d(x, dhat):
    if x >= dhat:
        return 0.0
    return -2 * (x - dhat) * math.log(x / dhat) - (x - dhat) ** 2 / x


def spring_reference(k, x_p, kappa, dhat):
    """Exact 1-DOF equilibrium gap of k/2 (x - x_p)^2 + kappa b(x) with x_p <= 0."""
    f = lambda x: k * (x - x_p) + kappa * clamped_log_d(x, dhat)
    lo, hi = 1e-300, dhat
    # f -> -inf as x -> 0+ (barrier pushes up), f(dhat) = k (dhat - x_p) > 0
    assert f(hi) > 0
    for _ in range(2000):
        mid = 0.5 * (lo + hi)
        if f(mid) < 0:
            lo = mid
        else:
            hi = mid
        if hi - lo <= 1e-16 * max(1.0, hi):
            break
    x = 0.5 * (lo + hi)
    return {'gap': x, 'spring_force': k * (x_p - x), 'barrier_force': -kappa * clamped_log_d(x, dhat), 'residual': f(x)}


if __name__ == '__main__':
    import json
    E, nu = 1e7, 0.45
    out = {}
    for name, (delta, g0) in {'public_press_t1': (0.25, 0.02), 'quarter': (0.25, 0.0)}.items():
        ref = block_reference(delta, g0, 1.0, E, nu)
        ref['dR_dlz'] = reaction_sensitivity(ref['lz'], E, nu)
        out[name] = ref
    print(json.dumps(out, indent=1))
