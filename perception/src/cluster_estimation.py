import numpy as np
import sklearn.mixture
from sklearn.preprocessing import StandardScaler

#use like: cluster_estimation(load_from_file(path))
def load_points_from_file(path, with_tags=False): 
    """load a file return a list of points. Also returns a list of tags if with_tags=True"""
    tags = []
    points = []
    with open(path, "r") as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            if "|" in line:
                coord_part, tag_part = line.split("|", 1)
                line_tags = tag_part.replace(",", " ").split()
            else:
                coord_part = line
                line_tags = []
            parts = coord_part.replace(",", " ").split()

            try:
                point = [float(x) for x in parts]
            except ValueError:
                raise ValueError(f"Line {line_num}: could not parse {line!r}")
            if points and len(point) != len(points[0]):
                raise ValueError(f"Not all points are the same dimension. Line {line_num}: expected {len(points[0])} values, got {len(point)}")
            points.append(point)
            tags.append(line_tags)
    if with_tags:
        return points, tags
    else:
        return points

def _group_value(point_tags, key):
    """Return the value of a `key=value` tag for one point, or None if absent."""
    prefix = key + "="
    for t in point_tags:
        if t.startswith(prefix):
            return t[len(prefix):]
    return None

def bucket_points_by_tag(points, tags, key, include_untagged=False):
    """Sort points into separate buckets by their `key=value` tag.
 
    Returns {value: [points]}. Points lacking the key are dropped unless
    include_untagged is True (then collected under '<untagged>').
    """
    buckets = {}
    for point, point_tags in zip(points, tags):
        value = _group_value(point_tags, key)
        if value is None:
            if not include_untagged:
                continue
            value = "<untagged>"
        buckets.setdefault(value, []).append(point)
    return buckets

def cluster_estimation(extracted_points: list):
    success, clusterInstance = ClusterEstimation.create(
        min_activation_threshold=5,  # Requires at least 5 points to start
        min_new_points_to_run=1,
        max_num_components=10,
        random_state=42,
        min_points_per_cluster=2,
    )

    if success and clusterInstance:
        did_run, clusters = clusterInstance.run(extracted_points)
        return clusters

    else:
        return None

def cluster_by_tag(points, tags, key, include_untagged=True):
    """Cluster each tag bucket seperately

    Points are sorted into buckets by their `key=value` tag, then
    cluster_estimation() is run independently on each bucket.
 
    Each bucket gets its own fresh ClusterEstimation instance

    Returns {value: clusters}, where clusters is the estimation output for
    that bucket (an empty list if the bucket was too small to run).
    """

    buckets = bucket_points_by_tag(points, tags, key, include_untagged)
    results = {}
    for value, bucket_points in buckets.items():
        results[value] = cluster_estimation(bucket_points)
    return results



class ClusterEstimation:
    _WEIGHT_DROP_THRESHOLD = (
        0.5  # Set low to keep sparse clusters. Points per cluster are very different
    )
    _MAX_COVARIANCE_THRESHOLD = 100  # Cluster Size can be in a large range of sizes

    @classmethod
    def create(
        cls,
        min_activation_threshold,
        min_new_points_to_run,
        max_num_components,
        random_state,
        min_points_per_cluster,
    ):

        if min_activation_threshold > max_num_components or max_num_components < 1:
            return False, None

        return True, cls(
            min_activation_threshold,
            min_new_points_to_run,
            max_num_components,
            random_state,
            min_points_per_cluster,
        )

    def __init__(
        self,
        min_activation_threshold,
        min_new_points_to_run,
        max_num_components,
        random_state,
        min_points_per_cluster,
    ):
        self._vgmm = sklearn.mixture.BayesianGaussianMixture(
            covariance_type="spherical",
            n_components=max_num_components,
            init_params="k-means++",
            weight_concentration_prior=0.001,  # Lower --> Accepts clusters with fewer points
            mean_precision_prior=1e-3,
            max_iter=3000,
            random_state=random_state,
        )
        self._scaler = StandardScaler()
        self._all_points = []
        self._current_bucket = []
        self._max_num_components = max_num_components
        self._min_activation_threshold = min_activation_threshold
        self._min_new_points_to_run = min_new_points_to_run
        self._min_points_per_cluster = min_points_per_cluster
        self._has_ran_once = False

    def run(self, detections, run_override=False):
        self._current_bucket = detections
        if not self._decide_to_run(run_override):
            return False, []

        raw_data = np.array(self._all_points)

        self._vgmm.n_components = min(self._max_num_components, len(raw_data))

        scaled_data = self._scaler.fit_transform(raw_data)

        self._vgmm.fit(scaled_data)

        if not self._vgmm.converged_:
            return False, []

        real_means = self._scaler.inverse_transform(self._vgmm.means_)

        model_output = list(
            zip(real_means, self._vgmm.weights_, self._vgmm.covariances_)
        )

        model_output = self._filter_by_points_ownership(model_output, scaled_data)

        model_output = self._sort_by_weights(model_output)

        if not model_output:
            return True, []

        viable_clusters = [model_output[0]]
        for i in range(1, len(model_output)):
            ratio = model_output[i][1] / (model_output[i - 1][1] + 1e-9)
            if ratio < self._WEIGHT_DROP_THRESHOLD:
                break
            viable_clusters.append(model_output[i])

        return True, self._filter_by_covariances(viable_clusters)

    def _decide_to_run(self, run_override):
        count_all = len(self._all_points)
        count_current = len(self._current_bucket)

        if not run_override:
            if count_all + count_current < self._min_activation_threshold:
                return False
            if self._has_ran_once and count_current < self._min_new_points_to_run:
                return False

        if count_all + count_current == 0:
            return False

        self._all_points.extend(self._current_bucket)
        self._current_bucket = []
        self._has_ran_once = True
        return True

    def _filter_by_points_ownership(self, model_output, scaled_data):
        cluster_assignment = self._vgmm.predict(scaled_data)
        unique, counts = np.unique(cluster_assignment, return_counts=True)
        cluster_counts = dict(zip(unique, counts))

        filtered_output = []
        for i, cluster_data in enumerate(model_output):
            if cluster_counts.get(i, 0) >= self._min_points_per_cluster:
                filtered_output.append(cluster_data)
        return filtered_output

    def _filter_by_covariances(self, model_output):
        if not model_output:
            return []
        min_cov = min(item[2] for item in model_output)
        threshold = min_cov * self._MAX_COVARIANCE_THRESHOLD
        return [c for c in model_output if c[2] <= threshold]

    def _sort_by_weights(self, model_output):
        return sorted(model_output, key=lambda x: x[1], reverse=True)