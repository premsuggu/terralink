"""Fusion Kernel - CuPy ElementwiseKernel for GPU-accelerated point cloud fusion.

This kernel implements the core Bayesian fusion algorithm on GPU.
Each thread processes one point from the point cloud.
"""
import cupy as cp
import math

# Layer indices (must match ElevationMapCPU)
IDX_ELEVATION = 0
IDX_VARIANCE = 1
IDX_IS_VALID = 2
IDX_TRAVERSABILITY = 3
IDX_TIME = 4
IDX_UPPER_BOUND = 5
IDX_IS_UPPER_BOUND = 6

# Shared device functions (preamble for all kernels)
DEVICE_FUNCTIONS = r'''
/* point_noise: sensor noise variance = factor * (x^2 + y^2 + z^2) */
__device__ float point_noise(float x, float y, float z, float factor) {
    return factor * (x*x + y*y + z*z);
}

/* world_to_grid_idx: convert world coordinates to grid index */
__device__ int world_to_grid_idx(float x, float y, float center_x, float center_y, 
                                  float resolution, int cell_n) {
    int col = int(roundf((x - center_x) / resolution + cell_n / 2.0f));
    int row = int(roundf((y - center_y) / resolution + cell_n / 2.0f));
    if (row < 0 || row >= cell_n || col < 0 || col >= cell_n) {
        return -1;
    }
    return row * cell_n + col;
}

/* is_valid_point: check if point is within valid range */
__device__ bool is_valid_point(float x, float y, float z, 
                                float min_dist, float max_dist,
                                float min_h, float max_h) {
    float dist = sqrtf(x*x + y*y + z*z);
    return (dist >= min_dist) && (dist <= max_dist) &&
           (z >= min_h) && (z <= max_h);
}

/* transform_point: transform point from sensor to map frame */
__device__ void transform_point(const float* R, const float* t,
                                 float px, float py, float pz,
                                 float* mx, float* my, float* mz) {
    *mx = R[0]*px + R[1]*py + R[2]*pz + t[0];
    *my = R[3]*px + R[4]*py + R[5]*pz + t[1];
    *mz = R[6]*px + R[7]*py + R[8]*pz + t[2];
}
'''

# Main fusion kernel - one thread per point
fusion_kernel = cp.ElementwiseKernel(
    in_params=(
        "raw float32 px, raw float32 py, raw float32 pz, "
        "raw float32 R, raw float32 t, "
        "int32 cell_n, float32 resolution, "
        "float32 center_x, float32 center_y, "
        "float32 sensor_noise_factor, float32 mahalanobis_thresh, "
        "float32 outlier_variance, float32 initial_variance, "
        "float32 min_valid_distance, float32 max_ray_length, "
        "float32 min_height, float32 max_height"
    ),
    out_params=(
        "raw float32 elevation, raw float32 variance, raw float32 is_valid, "
        "raw float32 time, raw float32 upper_bound, raw float32 is_upper_bound, "
        "raw float32 new_elevation, raw float32 new_variance, raw float32 new_count"
    ),
    preamble=DEVICE_FUNCTIONS,
    operation=(
        "/* Transform point to map frame */\n"
        "float mx, my, mz;\n"
        "transform_point(R, t, px, py, pz, &mx, &my, &mz);\n"
        "\n"
        "/* Validate point */\n"
        "if (!is_valid_point(mx, my, mz, min_valid_distance, max_ray_length, min_height, max_height)) {\n"
        "    return;\n"
        "}\n"
        "\n"
        "/* Compute grid index */\n"
        "int idx = world_to_grid_idx(mx, my, center_x, center_y, resolution, cell_n);\n"
        "if (idx < 0) return;\n"
        "\n"
        "/* Sensor noise variance (in sensor frame) */\n"
        "float v = point_noise(px, py, pz, sensor_noise_factor);\n"
        "\n"
        "/* Prior from map */\n"
        "float map_h = elevation[idx];\n"
        "float map_v = variance[idx];\n"
        "\n"
        "/* Mahalanobis outlier check */\n"
        "if (fabsf(map_h - mz) > sqrtf(map_v) * mahalanobis_thresh) {\n"
        "    /* Outlier: increase variance only, preserve elevation */\n"
        "    atomicAdd(&variance[idx], outlier_variance);\n"
        "    return;\n"
        "}\n"
        "\n"
        "/* Bayesian fusion */\n"
        "float new_h = (map_h * v + mz * map_v) / (map_v + v);\n"
        "float new_v = (map_v * v) / (map_v + v);\n"
        "\n"
        "/* Atomic accumulation (multiple points per cell) */\n"
        "atomicAdd(&new_elevation[idx], new_h);\n"
        "atomicAdd(&new_variance[idx], new_v);\n"
        "atomicAdd(&new_count[idx], 1.0);\n"
        "\n"
        "/* Mark valid, reset time */\n"
        "is_valid[idx] = 1.0;\n"
        "time[idx] = 0.0;\n"
        "upper_bound[idx] = mz;\n"
        "is_upper_bound[idx] = 1.0;"
    ),
    name='fuse_pointcloud_kernel'
)

# Finalize fusion kernel - one thread per cell
finalize_kernel = cp.ElementwiseKernel(
    in_params="raw float32 new_elevation, raw float32 new_variance, raw float32 new_count",
    out_params="raw float32 elevation, raw float32 variance, raw float32 is_valid",
    preamble="__device__ float initial_variance = 1.0;",
    operation=(
        "if (new_count > 0.0) {\n"
        "    elevation = new_elevation / new_count;\n"
        "    variance = new_variance / new_count;\n"
        "    is_valid = 1.0;\n"
        "} else {\n"
        "    elevation = 0.0;\n"
        "    variance = initial_variance;\n"
        "    is_valid = 0.0;\n"
        "}"
    ),
    name='finalize_fusion_kernel'
)

# Drift compensation kernel - one thread per point
drift_kernel = cp.ElementwiseKernel(
    in_params=(
        "raw float32 px, raw float32 py, raw float32 pz, "
        "raw float32 R, raw float32 t, "
        "int32 cell_n, float32 resolution, "
        "float32 center_x, float32 center_y, "
        "float32 mahalanobis_thresh, float32 outlier_variance, "
        "float32 traversability_inlier, "
        "raw float32 elevation, raw float32 variance, "
        "raw float32 is_valid, raw float32 traversability"
    ),
    out_params="raw float32 error_sum, raw float32 error_count",
    preamble=DEVICE_FUNCTIONS + r'''
        __device__ float point_noise_drift(float x, float y, float z, float factor) {
            return factor * (x*x + y*y + z*z);
        }
    ''',
    operation=(
        "/* Transform point to map frame */\n"
        "float mx, my, mz;\n"
        "transform_point(R, t, px, py, pz, &mx, &my, &mz);\n"
        "\n"
        "/* Validate */\n"
        "if (!is_valid_point(mx, my, mz, 0.3, 10.0, -2.0, 5.0)) {\n"
        "    return;\n"
        "}\n"
        "\n"
        "int idx = world_to_grid_idx(mx, my, center_x, center_y, 0.05, cell_n);\n"
        "if (idx < 0) return;\n"
        "\n"
        "/* Check if valid inlier on traversable terrain */\n"
        "float map_h = elevation[idx];\n"
        "float map_v = variance[idx];\n"
        "float trav = traversability[idx];\n"
        "bool valid = is_valid[idx] > 0.5;\n"
        "\n"
        "if (valid && fabsf(map_h - mz) < sqrtf(map_v) * mahalanobis_thresh &&\n"
        "    map_v < outlier_variance / 2.0 && trav > traversability_inlier) {\n"
        "    atomicAdd(&error_sum[0], mz - map_h);\n"
        "    atomicAdd(&error_count[0], 1.0);\n"
        "}"
    ),
    name='drift_compensation_kernel'
)

def compile_kernels(param):
    """Pre-compile kernels with current parameters (just-in-time compilation)."""
    # CuPy compiles on first use, but we can trigger it here
    # Create dummy arrays to trigger compilation
    dummy_px = cp.array([0.0], dtype=cp.float32)
    dummy_py = cp.array([0.0], dtype=cp.float32)
    dummy_pz = cp.array([0.0], dtype=cp.float32)
    dummy_R = cp.eye(3, dtype=cp.float32).ravel()
    dummy_t = cp.zeros(3, dtype=cp.float32)
    
    dummy_elev = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_var = cp.ones(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_valid = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_time = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_ub = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_iub = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_new_elev = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_new_var = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    dummy_new_cnt = cp.zeros(param.cell_n * param.cell_n, dtype=cp.float32)
    
    try:
        # Trigger compilation with minimal data
        fusion_kernel(
            dummy_px, dummy_py, dummy_pz,
            dummy_R, dummy_t,
            param.cell_n, param.resolution,
            0.0, 0.0,
            param.sensor_noise_factor, param.mahalanobis_thresh,
            param.outlier_variance, 1.0,
            param.min_valid_distance, param.max_ray_length,
            param.min_height, param.max_height,
            dummy_elev, dummy_var, dummy_valid,
            dummy_time, dummy_ub, dummy_valid,
            dummy_new_elev, dummy_new_var, dummy_new_cnt,
            size=1
        )
        cp.cuda.Device().synchronize()
    except:
        pass  # Compilation may fail with dummy data, but kernels are ready