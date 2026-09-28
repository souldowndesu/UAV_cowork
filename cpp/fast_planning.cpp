// fast_planning.cpp -- A* and B-spline C++ kernels with GIL release.
//
// The Python A* and B-spline optimizer are pure-Python loops that hold the GIL,
// blocking the 50Hz control loop and causing "API call was not received" (the
// simple_flight watchdog). Porting them to C++ AND releasing the GIL with
// `gil_scoped_release` lets the control loop run concurrently during planning.
//
// Build: see build_extension.bat (VS BuildTools + venv python).

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <cmath>
#include <vector>
#include <queue>
#include <tuple>
#include <cstdint>
#include <algorithm>

namespace py = pybind11;

// ---------------------------------------------------------------------------
// Trilinear distance-field query (matches nav/bspline.py `_dist_at` exactly,
// including the interpolation indexing).
// ---------------------------------------------------------------------------
static double dist_at(const double* p, const float* dist, int nx, int ny, int nz,
                      const double* origin, double res) {
    double gx = (p[0] - origin[0]) / res;
    double gy = (p[1] - origin[1]) / res;
    double gz = (p[2] - origin[2]) / res;
    int ix = (int)std::floor(gx);
    int iy = (int)std::floor(gy);
    int iz = (int)std::floor(gz);
    double fx = gx - ix, fy = gy - iy, fz = gz - iz;
    double d[8];
    for (int dx = 0; dx <= 1; dx++)
        for (int dy = 0; dy <= 1; dy++)
            for (int dz = 0; dz <= 1; dz++) {
                int cx = ix + dx, cy = iy + dy, cz = iz + dz;
                if (cx >= 0 && cx < nx && cy >= 0 && cy < ny && cz >= 0 && cz < nz)
                    d[dx * 4 + dy * 2 + dz] = dist[(cx * ny + cy) * nz + cz];
                else
                    d[dx * 4 + dy * 2 + dz] = 1e9;  // 越界视为远离障碍（>=d_safe，避免虚假惩罚）
            }
    double c00 = d[0] * (1 - fx) + d[1] * fx;
    double c01 = d[2] * (1 - fx) + d[3] * fx;
    double c10 = d[4] * (1 - fx) + d[5] * fx;
    double c11 = d[6] * (1 - fx) + d[7] * fx;
    double c0 = c00 * (1 - fy) + c10 * fy;
    double c1 = c01 * (1 - fy) + c11 * fy;
    return c0 * (1 - fz) + c1 * fz;
}

// ---------------------------------------------------------------------------
// 末端钳制均匀三次 B-spline 在时刻 t 的采样（匹配 nav/bspline.py `BSplineTrajectory.eval`）。
// ctrl 为扁平 (n*3) 行主序；pad = [ctrl, ctrl[-1], ctrl[-1]]（末端钳制）。
// ---------------------------------------------------------------------------
static void bspline_eval(const double* ctrl, int n, double dt, double t, double* p) {
    int seg = (int)(t / dt);
    double u;
    if (seg >= n - 1) { seg = n - 2; u = 1.0; }
    else if (seg < 0) { seg = 0; u = 0.0; }
    else { u = t / dt - seg; }
    auto P = [&](int idx, int d) {
        int i = (idx < n) ? idx : (n - 1);
        return ctrl[i * 3 + d];
    };
    double u2 = u * u, u3 = u * u2;
    double b0 = (1 - u) * (1 - u) * (1 - u) / 6.0;
    double b1 = (3 * u3 - 6 * u2 + 4) / 6.0;
    double b2 = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6.0;
    double b3 = u3 / 6.0;
    for (int d = 0; d < 3; d++)
        p[d] = b0 * P(seg, d) + b1 * P(seg + 1, d) + b2 * P(seg + 2, d) + b3 * P(seg + 3, d);
}

// ---------------------------------------------------------------------------
// B-spline total cost J (matches nav/bspline.py `_cost`).
// ctrl is a flat (n*3) row-major array.
// ---------------------------------------------------------------------------
static double bspline_cost(const double* ctrl, int n, const double* goal,
                           const float* dist, int nx, int ny, int nz,
                           const double* origin, double res, double dt,
                           double w_goal, double w_obs, double w_smooth, double w_dyn,
                           double d_safe, double d_min, double w_hard,
                           double v_max, double a_max,
                           int smooth_skip) {
    double j = 0.0;
    // J_goal: endpoint (end-clamped -> p(T) = ctrl[n-1])
    double dg0 = ctrl[(n - 1) * 3 + 0] - goal[0];
    double dg1 = ctrl[(n - 1) * 3 + 1] - goal[1];
    double dg2 = ctrl[(n - 1) * 3 + 2] - goal[2];
    j += w_goal * (dg0 * dg0 + dg1 * dg1 + dg2 * dg2);
    // J_obs（软，仅控制点）
    for (int k = 0; k < n; k++) {
        double dd = dist_at(ctrl + k * 3, dist, nx, ny, nz, origin, res);
        if (dd < d_safe) { double t = d_safe - dd; j += w_obs * t * t; }
    }
    // J_hard（硬净空 barrier）：沿 B-spline 曲线密集采样，净空 < d_min 处加二次惩罚。
    // 放在优化代价里（而非事后拒绝），让优化主动把曲线推离障碍，避免"无轨迹→停止"。
    const int n_samp = 100;
    double hp[3];
    for (int s = 0; s <= n_samp; s++) {
        double tt = (n - 1) * dt * (double)s / (double)n_samp;
        bspline_eval(ctrl, n, dt, tt, hp);
        double dd = dist_at(hp, dist, nx, ny, nz, origin, res);
        if (dd < d_min) { double g = d_min - dd; j += w_hard * g * g; }
    }
    // J_smooth (second differences) — skip the first smooth_skip terms so the
    // locked bunched start can accelerate instead of flattening the trajectory
    for (int k = smooth_skip; k < n - 2; k++) {
        double a0 = ctrl[(k + 2) * 3 + 0] - 2 * ctrl[(k + 1) * 3 + 0] + ctrl[k * 3 + 0];
        double a1 = ctrl[(k + 2) * 3 + 1] - 2 * ctrl[(k + 1) * 3 + 1] + ctrl[k * 3 + 1];
        double a2 = ctrl[(k + 2) * 3 + 2] - 2 * ctrl[(k + 1) * 3 + 2] + ctrl[k * 3 + 2];
        j += w_smooth * (a0 * a0 + a1 * a1 + a2 * a2);
    }
    // J_dyn velocity
    if (n >= 2) {
        for (int k = 0; k < n - 1; k++) {
            double vx = (ctrl[(k + 1) * 3 + 0] - ctrl[k * 3 + 0]) / dt;
            double vy = (ctrl[(k + 1) * 3 + 1] - ctrl[k * 3 + 1]) / dt;
            double vz = (ctrl[(k + 1) * 3 + 2] - ctrl[k * 3 + 2]) / dt;
            double sp = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (sp > v_max) { double t = sp - v_max; j += w_dyn * t * t; }
        }
    }
    // J_dyn acceleration — skip the first smooth_skip terms too
    if (n >= 3) {
        for (int k = smooth_skip; k < n - 2; k++) {
            double ax = (ctrl[(k + 2) * 3 + 0] - 2 * ctrl[(k + 1) * 3 + 0] + ctrl[k * 3 + 0]) / (dt * dt);
            double ay = (ctrl[(k + 2) * 3 + 1] - 2 * ctrl[(k + 1) * 3 + 1] + ctrl[k * 3 + 1]) / (dt * dt);
            double az = (ctrl[(k + 2) * 3 + 2] - 2 * ctrl[(k + 1) * 3 + 2] + ctrl[k * 3 + 2]) / (dt * dt);
            double am = std::sqrt(ax * ax + ay * ay + az * az);
            if (am > a_max) { double t = am - a_max; j += w_dyn * t * t; }
        }
    }
    return j;
}

// ---------------------------------------------------------------------------
// B-spline gradient-descent optimization (matches nav/bspline.py `optimize`).
// Returns optimized control points as (n, 3) float64.
// ---------------------------------------------------------------------------
py::array_t<double> bspline_optimize(
    py::array_t<double, py::array::c_style> ctrl0,
    py::array_t<double, py::array::c_style> goal,
    py::array_t<float, py::array::c_style> dist,
    py::array_t<double, py::array::c_style> origin,
    double res, double dt,
    double w_goal, double w_obs, double w_smooth, double w_dyn,
    double d_safe, double d_min, double w_hard, double v_max, double a_max,
    int iters, double lr, double eps, double max_step, int lock_first,
    int smooth_skip) {

    auto c0 = ctrl0.unchecked<2>();
    const double* go = goal.data();
    const float* dd = dist.data();
    const double* og = origin.data();

    const int n = (int)c0.shape(0);
    const int nx = (int)dist.shape(0), ny = (int)dist.shape(1), nz = (int)dist.shape(2);

    std::vector<double> ctrl(n * 3);
    for (int i = 0; i < n; i++)
        for (int d = 0; d < 3; d++)
            ctrl[i * 3 + d] = c0(i, d);

    std::vector<double> grad(n * 3);

    {
        py::gil_scoped_release release;  // release GIL during the heavy loop
        for (int iter = 0; iter < iters; iter++) {
        std::fill(grad.begin(), grad.end(), 0.0);
        for (int i = lock_first; i < n; i++) {
            for (int d = 0; d < 3; d++) {
                int idx = i * 3 + d;
                ctrl[idx] += eps;
                double fp = bspline_cost(ctrl.data(), n, go, dd,
                                         nx, ny, nz, og, res, dt,
                                         w_goal, w_obs, w_smooth, w_dyn,
                                         d_safe, d_min, w_hard, v_max, a_max, smooth_skip);
                ctrl[idx] -= 2 * eps;
                double fm = bspline_cost(ctrl.data(), n, go, dd,
                                         nx, ny, nz, og, res, dt,
                                         w_goal, w_obs, w_smooth, w_dyn,
                                         d_safe, d_min, w_hard, v_max, a_max, smooth_skip);
                ctrl[idx] += eps;
                grad[idx] = (fp - fm) / (2 * eps);
            }
        }
        // delta = lr * grad, clip per control point
        for (int i = lock_first; i < n; i++) {
            double d0 = lr * grad[i * 3 + 0];
            double d1 = lr * grad[i * 3 + 1];
            double d2 = lr * grad[i * 3 + 2];
            double nm = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
            if (nm > max_step) {
                double s = max_step / (nm > 1e-12 ? nm : 1e-12);
                d0 *= s; d1 *= s; d2 *= s;
            }
            ctrl[i * 3 + 0] -= d0;
            ctrl[i * 3 + 1] -= d1;
            ctrl[i * 3 + 2] -= d2;
        }
        // lock first lock_first points
        for (int i = 0; i < lock_first; i++)
            for (int d = 0; d < 3; d++)
                ctrl[i * 3 + d] = c0(i, d);
        }
    }  // GIL re-acquired here

    py::array_t<double> out({n, 3});
    auto o = out.mutable_unchecked<2>();
    for (int i = 0; i < n; i++)
        for (int d = 0; d < 3; d++)
            o(i, d) = ctrl[i * 3 + d];
    return out;
}

// ---------------------------------------------------------------------------
// 3D 26-connected A* (matches nav/astar.py AStarPlanner.plan).
// Returns the path as (M,3) world-coordinate waypoints (voxel centers),
// excluding the start; empty if no path.
// ---------------------------------------------------------------------------
py::array_t<double> astar_plan(
    py::array_t<std::int8_t, py::array::c_style> occ,
    py::array_t<float, py::array::c_style> dist,
    py::array_t<double, py::array::c_style> origin,
    double res,
    py::array_t<double, py::array::c_style> start,
    py::array_t<double, py::array::c_style> goal,
    double c_free, double c_unknown, double w_obs, double d_safe, double d_min,
    double h_weight, int max_nodes,
    double w_dir, py::array_t<double, py::array::c_style> v_dir) {

    auto oc = occ.unchecked<3>();
    auto di = dist.unchecked<3>();
    auto og = origin.unchecked<1>();
    auto st = start.unchecked<1>();
    auto gl = goal.unchecked<1>();

    // 速度方向引导（与 Python plan() 一致）：归一化 v_dir；零向量/未提供则禁用
    double vd[3] = {0.0, 0.0, 0.0};
    bool use_vdir = false;
    if (w_dir > 0.0 && v_dir.size() >= 3) {
        auto va = v_dir.unchecked<1>();
        double vn = std::sqrt(va(0) * va(0) + va(1) * va(1) + va(2) * va(2));
        if (vn >= 1e-6) {
            vd[0] = va(0) / vn; vd[1] = va(1) / vn; vd[2] = va(2) / vn;
            use_vdir = true;
        }
    }

    const int nx = (int)oc.shape(0), ny = (int)oc.shape(1), nz = (int)oc.shape(2);
    const double ox = og(0), oy = og(1), oz = og(2);

    auto to_voxel = [&](double x, double y, double z, int* vi) {
        vi[0] = (int)std::floor((x - ox) / res);
        vi[1] = (int)std::floor((y - oy) / res);
        vi[2] = (int)std::floor((z - oz) / res);
    };
    auto in_bounds = [&](int i, int j, int k) {
        return i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz;
    };

    // cost of a voxel (matches AStarPlanner._cost); returns +inf if blocked
    // 指数距离惩罚 phi(d)=exp(d_safe-d)-1（d<d_safe 时），d_safe=5 时 5m→0/2m→19。
    auto voxel_cost = [&](int i, int j, int k) -> double {
        std::int8_t s = oc(i, j, k);
        if (s == 2) return 1e30;                     // OCCUPIED
        double dd = di(i, j, k);
        if (dd < d_min) return 1e30;                 // 净空硬截断（FREE/UNKNOWN 同）
        if (s == 0) {                                 // UNKNOWN
            if (dd < d_safe) {
                return c_unknown + w_obs * (std::exp(d_safe - dd) - 1.0);
            }
            return c_unknown;
        }
        if (dd < d_safe) {                            // FREE
            return c_free + w_obs * (std::exp(d_safe - dd) - 1.0);
        }
        return c_free;
    };

    int sv[3], gv[3];
    to_voxel(st(0), st(1), st(2), sv);
    to_voxel(gl(0), gl(1), gl(2), gv);

    const double INF = 1e30;
    std::vector<int> path_f;

    {
        py::gil_scoped_release release;  // release GIL during the search
        bool no_path = false;

        // nearest clear voxel near goal (matches _nearest_clear, radius 6)
        if (!in_bounds(gv[0], gv[1], gv[2]) || voxel_cost(gv[0], gv[1], gv[2]) >= INF) {
            int best[3] = {-1, -1, -1};
            double best_d2 = INF;
            const int radius = 6;
            for (int i = gv[0] - radius; i <= gv[0] + radius; i++)
                for (int j = gv[1] - radius; j <= gv[1] + radius; j++)
                    for (int k = gv[2] - radius; k <= gv[2] + radius; k++) {
                        if (!in_bounds(i, j, k)) continue;
                        if (voxel_cost(i, j, k) >= INF) continue;
                        double d2 = (double)(i - gv[0]) * (i - gv[0])
                                  + (double)(j - gv[1]) * (j - gv[1])
                                  + (double)(k - gv[2]) * (k - gv[2]);
                        if (d2 < best_d2) { best_d2 = d2; best[0] = i; best[1] = j; best[2] = k; }
                    }
            if (best[0] < 0) no_path = true;
            else { gv[0] = best[0]; gv[1] = best[1]; gv[2] = best[2]; }
        }

        if (!no_path && (!in_bounds(sv[0], sv[1], sv[2]) || voxel_cost(sv[0], sv[1], sv[2]) >= INF))
            no_path = true;

        if (!no_path) {
            auto flat = [&](int i, int j, int k) { return (i * ny + j) * nz + k; };
            int sf = flat(sv[0], sv[1], sv[2]);
            int gf = flat(gv[0], gv[1], gv[2]);

            // 26 neighbors + lengths
            const int nd[26][3] = {
                {-1,-1,-1},{-1,-1,0},{-1,-1,1},{-1,0,-1},{-1,0,0},{-1,0,1},{-1,1,-1},{-1,1,0},{-1,1,1},
                {0,-1,-1},{0,-1,0},{0,-1,1},{0,0,-1},{0,0,1},{0,1,-1},{0,1,0},{0,1,1},
                {1,-1,-1},{1,-1,0},{1,-1,1},{1,0,-1},{1,0,0},{1,0,1},{1,1,-1},{1,1,0},{1,1,1}
            };

            const int total = nx * ny * nz;
            std::vector<double> g(total, INF);
            std::vector<std::int32_t> came(total, -1);
            std::vector<std::int8_t> closed(total, 0);

            g[sf] = 0.0;
            double h0 = h_weight * std::sqrt(
                (double)(sv[0]-gv[0])*(sv[0]-gv[0]) + (double)(sv[1]-gv[1])*(sv[1]-gv[1]) + (double)(sv[2]-gv[2])*(sv[2]-gv[2])) * res;

            using Node = std::tuple<double, double, int>;
            std::priority_queue<Node, std::vector<Node>, std::greater<Node>> heap;
            heap.push({h0, 0.0, sf});

            int expanded = 0;
            bool found = false;
            while (!heap.empty()) {
                auto [f, gs, cur] = heap.top(); heap.pop();
                if (closed[cur]) continue;
                if (gs > g[cur]) continue;
                closed[cur] = 1;
                if (++expanded > max_nodes) break;
                if (cur == gf) { found = true; break; }

                int ci = cur / (ny * nz);
                int cj = (cur / nz) % ny;
                int ck = cur % nz;
                bool is_start = (cur == sf) && use_vdir;

                for (int m = 0; m < 26; m++) {
                    int ni = ci + nd[m][0], nj = cj + nd[m][1], nk = ck + nd[m][2];
                    if (!in_bounds(ni, nj, nk)) continue;
                    double cc = voxel_cost(ni, nj, nk);
                    if (cc >= INF) continue;
                    double elen = std::sqrt((double)nd[m][0]*nd[m][0] + (double)nd[m][1]*nd[m][1] + (double)nd[m][2]*nd[m][2]);
                    double ng = gs + cc * elen * res;
                    if (is_start) {
                        double cosang = (nd[m][0] * vd[0] + nd[m][1] * vd[1] + nd[m][2] * vd[2]) / elen;
                        ng += w_dir * (1.0 - cosang) * res;
                    }
                    int nf = flat(ni, nj, nk);
                    if (ng < g[nf]) {
                        g[nf] = ng;
                        came[nf] = cur;
                        double h = h_weight * std::sqrt(
                            (double)(ni-gv[0])*(ni-gv[0]) + (double)(nj-gv[1])*(nj-gv[1]) + (double)(nk-gv[2])*(nk-gv[2])) * res;
                        heap.push({ng + h, ng, nf});
                    }
                }
            }

            if (found) {
                std::vector<int> tmp;
                int cur = gf;
                while (cur != sf && cur != -1) {
                    tmp.push_back(cur);
                    cur = came[cur];
                }
                if (cur == -1) no_path = true;
                else { std::reverse(tmp.begin(), tmp.end()); path_f = tmp; }
            } else {
                no_path = true;
            }
        }
    }  // GIL re-acquired here

    py::array_t<double> out({(py::ssize_t)path_f.size(), (py::ssize_t)3});
    auto o = out.mutable_unchecked<2>();
    for (size_t t = 0; t < path_f.size(); t++) {
        int f = path_f[t];
        int i = f / (ny * nz), j = (f / nz) % ny, k = f % nz;
        o(t, 0) = ox + (i + 0.5) * res;
        o(t, 1) = oy + (j + 0.5) * res;
        o(t, 2) = oz + (k + 0.5) * res;
    }
    return out;
}

void register_fast_planning(py::module_& m) {
    m.def("bspline_optimize", &bspline_optimize,
          py::arg("ctrl0"), py::arg("goal"), py::arg("dist"), py::arg("origin"),
          py::arg("res"), py::arg("dt"),
          py::arg("w_goal"), py::arg("w_obs"), py::arg("w_smooth"), py::arg("w_dyn"),
          py::arg("d_safe"), py::arg("d_min"), py::arg("w_hard"),
          py::arg("v_max"), py::arg("a_max"),
          py::arg("iters"), py::arg("lr"), py::arg("eps"), py::arg("max_step"),
          py::arg("lock_first"), py::arg("smooth_skip"),
          "B-spline trajectory optimization (GIL released during compute).");
    m.def("astar_plan", &astar_plan,
          py::arg("occ"), py::arg("dist"), py::arg("origin"), py::arg("res"),
          py::arg("start"), py::arg("goal"),
          py::arg("c_free"), py::arg("c_unknown"), py::arg("w_obs"),
          py::arg("d_safe"), py::arg("d_min"), py::arg("h_weight"), py::arg("max_nodes"),
          py::arg("w_dir") = 0.0, py::arg("v_dir") = py::array_t<double>(),
          "3D 26-connected A* path planning (GIL released during compute).");
}
