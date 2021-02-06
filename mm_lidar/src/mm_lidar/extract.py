import numpy as np
import IPython


N_BUCKETS = 10
MAX_DIST = 2.0
MIN_ANGLE = -np.pi / 2.0
MAX_ANGLE = np.pi / 2.0


def parse_scan(scan):
    """Get ranges and angles from a scan."""
    n = len(scan.ranges)
    ranges = np.array(scan.ranges)
    angles = np.array([scan.angle_min + i*scan.angle_increment for i in range(n)])
    valid = (angles >= MIN_ANGLE) & (angles <= MAX_ANGLE)
    return ranges[valid], angles[valid]


# def reshape_returns(ranges, angles, n_buckets):
#     """Reshape lidar returns into `n_buckets` segments."""
#     # pad up so the arrays are divisible by n_buckets
#     # n = ranges.shape[0]
#     # n_pad = n_buckets - (n % n_buckets)
#     # ranges = np.concatenate((ranges, np.zeros(n_pad)))
#     # angles = np.concatenate((angles, np.zeros(n_pad)))
#
#     # hack: remove last element so arrays are of length 1080, divisible by 9 or
#     # 10
#     range_buckets = ranges[:-1].reshape((n_buckets, -1))
#     angle_buckets = angles[:-1].reshape((n_buckets, -1))
#     return range_buckets, angle_buckets


def extract_positions(scan):
    """Get positions of scan in the sensor frame."""
    ranges, angles = parse_scan(scan)
    valid_mask = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    positions = np.vstack((np.cos(angles), np.sin(angles))) * ranges
    return positions, valid_mask


def filter_points_by_distance(p_ob_bs, p_ow_ws, valid_mask,
                              n_buckets=N_BUCKETS,
                              max_dist=MAX_DIST):
    # compute distances of detections to the base
    dists_squared = np.sum(p_ob_bs**2, axis=0)
    dists_squared[~valid_mask] = np.inf

    # split into n_buckets and find the minimum distance in each
    dists_squared_re = dists_squared.reshape((n_buckets, -1))
    min_idx = np.argmin(dists_squared_re, axis=1)
    min_dists_squared = dists_squared_re[range(n_buckets), min_idx]
    valid_min_dist_mask = min_dists_squared < max_dist**2

    # get the corresponding minimum distance world points
    p_ow_ws_re = p_ow_ws.reshape((2, n_buckets, -1))
    p_ow_ws_closest = p_ow_ws_re[:, valid_min_dist_mask, min_idx[valid_min_dist_mask]]

    return p_ow_ws_closest
