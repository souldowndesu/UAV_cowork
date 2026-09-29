// fast_kernels.cpp -- C++ acceleration kernel (pybind11).
//
// Exposes the only real hot spot of Phase 1: batched 3D DDA (Amanatides-Woo)
// ray casting, point cloud -> occupancy map (plan doc section 14). This is the
// hardest loop to vectorize and one of the main compute sources (section 55),
// so it runs in C++ instead of a per-ray Python loop.
//
// Build: see build_extension.bat (VS BuildTools + venv python).

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>

namespace py = pybind11;

// Occupancy-map states (must match nav/occupancy.py)
static const std::int8_t UNKNOWN = 0;
static const std::int8_t FREE = 1;
static const std::int8_t OCCUPIED = 2;

// ---------------------------------------------------------------------------
// Single-ray 3D DDA (Amanatides-Woo): append traversed voxels (inclusive of
// both endpoints) to `out`. Inputs are already in grid coordinates (voxel
// units).
// ---------------------------------------------------------------------------
static void dda3d(double gx0, double gy0, double gz0,
                  double gx1, double gy1, double gz1,
                  std::vector<int>& out) {
    int ix = static_cast<int>(std::floor(gx0));
    int iy = static_cast<int>(std::floor(gy0));
    int iz = static_cast<int>(std::floor(gz0));
    int exi = static_cast<int>(std::floor(gx1));
    int eyi = static_cast<int>(std::floor(gy1));
    int ezi = static_cast<int>(std::floor(gz1));

    double dx = gx1 - gx0;
    double dy = gy1 - gy0;
    double dz = gz1 - gz0;

    int stepX = (dx > 0.0) ? 1 : -1;
    int stepY = (dy > 0.0) ? 1 : -1;
    int stepZ = (dz > 0.0) ? 1 : -1;

    const double inf = 1e30;
    double tDeltaX = (std::fabs(dx) < 1e-12) ? inf : std::fabs(1.0 / dx);
    double tDeltaY = (std::fabs(dy) < 1e-12) ? inf : std::fabs(1.0 / dy);
    double tDeltaZ = (std::fabs(dz) < 1e-12) ? inf : std::fabs(1.0 / dz);

    // t at first boundary crossing along each axis
    double tMaxX = (std::fabs(dx) < 1e-12) ? inf
        : ((stepX > 0) ? (std::floor(gx0) + 1.0 - gx0) : (gx0 - std::floor(gx0))) * tDeltaX;
    double tMaxY = (std::fabs(dy) < 1e-12) ? inf
        : ((stepY > 0) ? (std::floor(gy0) + 1.0 - gy0) : (gy0 - std::floor(gy0))) * tDeltaY;
    double tMaxZ = (std::fabs(dz) < 1e-12) ? inf
        : ((stepZ > 0) ? (std::floor(gz0) + 1.0 - gz0) : (gz0 - std::floor(gz0))) * tDeltaZ;

    out.push_back(ix);
    out.push_back(iy);
    out.push_back(iz);

    int guard = 0;
    const int max_steps = 4096;
    while (!(ix == exi && iy == eyi && iz == ezi)) {
        if (++guard > max_steps) break;
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                ix += stepX; tMaxX += tDeltaX;
            } else {
                iz += stepZ; tMaxZ += tDeltaZ;
            }
        } else {
            if (tMaxY < tMaxZ) {
                iy += stepY; tMaxY += tDeltaY;
            } else {
                iz += stepZ; tMaxZ += tDeltaZ;
            }
        }
        out.push_back(ix);
        out.push_back(iy);
        out.push_back(iz);
    }
}

// ---------------------------------------------------------------------------
// raycast_batch: write LiDAR rays into the occupancy map (in place).
//   origins/hits : (N,3) float64 world-frame (NED)
//   occ          : (nx,ny,nz) int8 (modified in place)
//   origin       : (3,) world-frame low corner
//   res          : voxel size (meters)
//   max_range    : rays longer than this (meters) are skipped
// ---------------------------------------------------------------------------
void raycast_batch(py::array_t<double, py::array::c_style> origins,
                   py::array_t<double, py::array::c_style> hits,
                   py::array_t<std::int8_t, py::array::c_style> occ,
                   py::array_t<double, py::array::c_style> origin,
                   double res, double max_range) {
    auto o = origins.unchecked<2>();
    auto h = hits.unchecked<2>();
    auto m = occ.mutable_unchecked<3>();
    auto og = origin.unchecked<1>();

    const int nrays = static_cast<int>(o.shape(0));
    const int nx = static_cast<int>(m.shape(0));
    const int ny = static_cast<int>(m.shape(1));
    const int nz = static_cast<int>(m.shape(2));
    const double ox = og(0), oy = og(1), oz = og(2);
    const double max_range2 = max_range * max_range;

    py::gil_scoped_release release;
    std::vector<std::int8_t> batch(nx * ny * nz, UNKNOWN);
    std::vector<int> cells;
    cells.reserve(256);

    for (int r = 0; r < nrays; ++r) {
        double sx = o(r, 0), sy = o(r, 1), sz = o(r, 2);
        double ex = h(r, 0), ey = h(r, 1), ez = h(r, 2);
        double lx = ex - sx, ly = ey - sy, lz = ez - sz;
        double len2 = lx * lx + ly * ly + lz * lz;
        if (!std::isfinite(len2) || len2 > max_range2) continue;

        double gx0 = (sx - ox) / res, gy0 = (sy - oy) / res, gz0 = (sz - oz) / res;
        double gx1 = (ex - ox) / res, gy1 = (ey - oy) / res, gz1 = (ez - oz) / res;

        cells.clear();
        dda3d(gx0, gy0, gz0, gx1, gy1, gz1, cells);
        const int n = static_cast<int>(cells.size()) / 3;
        if (n <= 1) {
            // Start and hit share a voxel: likely a self-echo; skip so we never
            // mark the vehicle's own cell occupied.
            continue;
        }
        // Along the ray (excluding start) -> free; endpoint -> occupied.
        for (int t = 0; t < n - 1; ++t) {
            int ix = cells[t * 3], iy = cells[t * 3 + 1], iz = cells[t * 3 + 2];
            if (ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz)
                batch[(ix*ny+iy)*nz+iz] = std::max(batch[(ix*ny+iy)*nz+iz], FREE);
        }
        {
            int ix = cells[(n - 1) * 3], iy = cells[(n - 1) * 3 + 1], iz = cells[(n - 1) * 3 + 2];
            if (ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz)
                batch[(ix*ny+iy)*nz+iz] = OCCUPIED;
        }
    }
    for (int i=0;i<nx;++i) for(int j=0;j<ny;++j) for(int k=0;k<nz;++k) {
        auto value=batch[(i*ny+j)*nz+k];
        if(value!=UNKNOWN) m(i,j,k)=value;
    }
}

void register_fast_planning(py::module_& m);

PYBIND11_MODULE(_fast, m) {
    m.attr("source_hash") = NAV_SOURCE_HASH;
    m.doc() = "Phase-1 navigation C++ kernels (3D DDA ray casting, B-spline, A*)";
    m.def("raycast_batch", &raycast_batch,
          py::arg("origins"), py::arg("hits"), py::arg("occ"),
          py::arg("origin"), py::arg("res"), py::arg("max_range"),
          "Batched 3D DDA ray casting into the occupancy map (in place).");
    register_fast_planning(m);
}
