# Copyright 2024 Proxima Technology Inc, TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
from typing import Callable
from typing import Literal

from matplotlib.patches import Ellipse  # type: ignore
import matplotlib.pyplot as plt  # type: ignore
import numpy as np
import plotly.express as px  # type: ignore
import plotly.graph_objs as go  # type: ignore
from scipy.spatial import ConvexHull  # type: ignore
from scipy.spatial import Delaunay
from sklearn.neighbors import KernelDensity  # type: ignore
from sklearn.preprocessing import StandardScaler  # type: ignore
from utils.data_collection_utils import ControlType
from utils.parameter_change_utils import ChangeParam  # type: ignore
from utils.parameter_change_utils import test_dir_name

KinematicKey = Literal["speed", "acc", "steer"]
"""The key of kinematic states."""


class KinematicStates:
    """The class for kinematic states, which includes speed, acceleration, and steering angle."""

    def __init__(self, speed: np.ndarray, acc: np.ndarray, steer: np.ndarray):
        self.speed = speed
        self.acc = acc
        self.steer = steer

    def to_ndarray(self, field_list: list[str] = ["speed", "acc", "steer"]) -> np.ndarray:
        return np.stack(
            arrays=[getattr(self, field) for field in field_list],
            axis=1,
        )

    def plot(
        self,
        x_axis: KinematicKey,
        y_axis: KinematicKey,
        z_axis: KinematicKey | None = None,
    ) -> go.Figure:
        """Plot interactive scatter image of kinematic states in 2D or 3D."""
        if z_axis is None:
            fig = px.scatter(x=getattr(self, x_axis), y=getattr(self, y_axis))
            fig.update_layout(xaxis_title=x_axis, yaxis_title=y_axis)
        else:
            fig = px.scatter_3d(
                x=getattr(self, x_axis),
                y=getattr(self, y_axis),
                z=getattr(self, z_axis),
            )
            fig.update_layout(
                scene={"xaxis_title": x_axis, "yaxis_title": y_axis, "zaxis_title": z_axis}
            )
        fig.update_traces(marker={"size": 1})
        return fig


def load_kinematic_states(
    control_type: ControlType | None = None,
    change_param: ChangeParam | None = None,
    index: int | None = None,
    load_dir: str | None = None,
    verbose: bool = False,
) -> KinematicStates:
    """Load kinematic states from the specified directory."""
    if load_dir is None:
        load_dir = test_dir_name(
            control_type=control_type,
            change_param=change_param,
            index=index,
        )
    if verbose:
        print(f"loaded directory: {load_dir}")

    # cSpell:ignore usecols
    observed_speed_array = np.loadtxt(
        fname=load_dir + "/kinematic_state.csv", delimiter=",", usecols=47
    )
    observed_acc_array = np.loadtxt(
        fname=load_dir + "/acceleration.csv",
        delimiter=",",
        usecols=3,
    )
    observed_steer_array = np.loadtxt(
        fname=load_dir + "/steering_status.csv",
        delimiter=",",
        usecols=2,
    )
    kinematic_states = KinematicStates(
        speed=observed_speed_array, acc=observed_acc_array, steer=observed_steer_array
    )
    return kinematic_states


def load_kinematic_states_and_result(
    load_dir: str, mem_diff=False, max_lateral_deviation: float = 5.0
) -> tuple[KinematicStates, float, float]:
    """Return the KinematicStates object and the lateral error of the simulation result.

    This read the given directory (the directory where the kinematic states are stored)
    To read the lateral deviation, however, it goes to read the appropriate directory next to the given directory.

    Returns:
    * KinematicStates: KinematicStates object
    * float: the lateral error of the simulation result. this is not for trained data
    * float: the lateral error of the simulation result. this is for trained data
    """
    kinematic_states = load_kinematic_states(
        control_type=None,
        change_param=None,
        index=None,
        load_dir=load_dir,
    )

    load_dir_for_result = load_dir.replace(ControlType.pp_eight.value, "pp_aided")
    if mem_diff:
        load_dir_for_result += "_mem_diff"
    load_dir_for_trained_result = load_dir_for_result.replace("_aided_sim_", "_aided_sim_trained_")

    with open(f"{load_dir_for_result}/auto_test_performance_result.json", "r") as f:
        data = json.load(f)

    with open(f"{load_dir_for_trained_result}/auto_test_performance_result.json", "r") as f:
        data_trained = json.load(f)

    total_abs_max_lateral_deviation = min(
        data.get("total_abs_max_lateral_deviation"), max_lateral_deviation
    )
    trained_total_abs_max_lateral_deviation = min(
        data_trained.get("total_abs_max_lateral_deviation"), max_lateral_deviation
    )
    return (
        kinematic_states,
        total_abs_max_lateral_deviation,
        trained_total_abs_max_lateral_deviation,
    )


class CalcMethodOfScalarIndex:
    name: Literal["minimum", "percentage"]
    """Which scalar index is to be calculated."""

    threshold: float
    """This is used only when the name is `percentage`."""

    def __init__(self, name: Literal["minimum", "percentage"], threshold: float = 0.0):
        self.name = name
        self.threshold = threshold

    def description(self) -> str:
        if self.name == "minimum":
            return "minimum value of KDE"
        if self.name == "percentage":
            return "percentage of KDE below threshold"


class Shape:
    """The class for shape functions, just a namespace."""

    # config for ellipse
    ELLIPSE_CENTER = [6.0, 0.0]

    ELLIPSE_WIDTH = 10.0

    ELLIPSE_HEIGHT = 2.0

    @staticmethod
    def trivial(_x: float, _y: float) -> bool:
        """Trivial shape function."""
        return True

    @staticmethod
    def ellipse(x: float, y: float) -> bool:
        """Represent the interior of an ellipse."""
        # read config
        center = Shape.ELLIPSE_CENTER
        width = Shape.ELLIPSE_WIDTH
        hight = Shape.ELLIPSE_HEIGHT

        return (x - center[0]) ** 2 / (width / 2) ** 2 + (y - center[1]) ** 2 / (
            hight / 2
        ) ** 2 <= 1

    @staticmethod
    def triangle(A: list[float], B: list[float], C: list[float], x: float, y: float) -> bool:
        """Represent the interior of a triangle.

        Determine if the given point is inside the triangle formed by connecting the given three points.
        """

        def sign(p, q, r):
            """Check relation of `p`, `q`, `r`."""
            return (p[0] - r[0]) * (q[1] - r[1]) - (q[0] - r[0]) * (p[1] - r[1])

        P = [x, y]
        b1 = sign(P, A, B) < 0
        b2 = sign(P, B, C) < 0
        b3 = sign(P, C, A) < 0

        # P is inside triangle ABC if all sides are on the same side
        return b1 == b2 == b3

    @staticmethod
    def convex_hull(points: np.ndarray, x: float, y: float) -> bool:
        """Determine if a point is within the convex hull.

        Parameters:
        points (ndarray): An array of points forming the convex hull (Nx2).
        x (float): The x-coordinate of the point to be checked.
        y (float): The y-coordinate of the point to be checked.

        Returns:
        bool: True if the point is within the convex hull, False otherwise.
        """
        hull = ConvexHull(points)
        delaunay = Delaunay(points[hull.vertices])
        return delaunay.find_simplex([x, y]) >= 0


def get_search_points(
    n_points: int,
    x_range: list[float],
    y_range: list[float],
    z_range: list[float] = [],
):
    """Help for density estimation."""
    x = np.linspace(x_range[0], x_range[1], num=n_points)
    y = np.linspace(y_range[0], y_range[1], num=n_points)
    if z_range != []:
        z = np.linspace(z_range[0], z_range[1], num=n_points)
        return np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    return np.array(np.meshgrid(x, y)).T.reshape(-1, 2)


class ScottCoef:
    """Multiple coefficient for bandwidth of Scott's algorithm."""

    def __init__(self, val: float):
        self.val = val


def calc_scalar_indexes(
    kinematic_states: KinematicStates,
    method: CalcMethodOfScalarIndex,
    fst: Literal["speed", "acc", "steer"],
    snd: Literal["speed", "acc", "steer"],
    fst_range: list[float],
    snd_range: list[float],
    bandwidths: list[float] | list[ScottCoef],
    shape: Callable[..., bool] = Shape.trivial,
) -> np.ndarray:
    """Return an ndarray of scalar indexes calculated by the kernel density estimation.

    This takes as a parameter the method (the minimum value in a particular region) used to compute the scalar index of kernel density.
    """
    array = np.array([])

    if method.name == "minimum":
        for bandwidth in bandwidths:
            min_point, min_val = calc_minimum_density_point(
                kinematic_states=kinematic_states,
                fst=fst,
                snd=snd,
                fst_range=fst_range,
                snd_range=snd_range,
                shape=shape,
                bandwidth=bandwidth,
            )
            array = np.append(array, min_val)

    if method.name == "percentage":
        for bandwidth in bandwidths:
            index = calc_percentage_of_density_below_threshold(
                kinematic_states=kinematic_states,
                threshold=method.threshold,
                fst=fst,
                snd=snd,
                fst_range=fst_range,
                snd_range=snd_range,
                shape=shape,
                bandwidth=bandwidth,
            )
            array = np.append(array, index)
        pass

    return array


# cSpell:ignore silverman
def kde_score_func(
    kinematic_states: KinematicStates,
    fst: KinematicKey,
    snd: KinematicKey,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
    verbose: bool = False,
) -> Callable[..., float]:
    """Return the function which is a variation of `KernelDensity.score_samples` including step of standardization."""
    # standardize the kinematic states
    scaler = StandardScaler()
    scaler.fit(kinematic_states.to_ndarray([fst, snd]))
    standardized_2d_kinematic_states = scaler.transform(kinematic_states.to_ndarray([fst, snd]))

    if not isinstance(bandwidth, ScottCoef):
        # perform density estimation with standardization
        if verbose:
            print(f"info: bandwidth is set to {bandwidth}")
        standardized_kde = KernelDensity(kernel="gaussian", bandwidth=bandwidth).fit(
            standardized_2d_kinematic_states
        )
    else:
        scott_kde = KernelDensity(kernel="gaussian", bandwidth="scott").fit(
            standardized_2d_kinematic_states
        )
        scott_bandwidth = scott_kde.bandwidth_
        if verbose:
            print(f"info: bandwidth is set to {scott_bandwidth * bandwidth.val}")
        standardized_kde = KernelDensity(
            kernel="gaussian", bandwidth=scott_bandwidth * bandwidth.val
        ).fit(standardized_2d_kinematic_states)

    return lambda arr: standardized_kde.score_samples(scaler.transform(arr))


def get_kde_scores(
    kinematic_states: KinematicStates,
    fst: KinematicKey,
    snd: KinematicKey,
    fst_range: list[float],
    snd_range: list[float],
    shape: Callable[..., bool] = Shape.trivial,
    point_number: int = 30,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
) -> tuple[np.ndarray, np.ndarray]:
    kde_score = kde_score_func(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        bandwidth=bandwidth,
    )
    grids = get_search_points(point_number, fst_range, snd_range)
    filtered_grids = np.array([point for point in grids if shape(point[0], point[1])])
    kernel_densities = np.exp(kde_score(filtered_grids))
    return kernel_densities, filtered_grids


def plot_kernel_density(
    kinematic_states: KinematicStates,
    fst: KinematicKey,
    snd: KinematicKey,
    fst_range: list[float],
    snd_range: list[float],
    point_number: int = 30,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
) -> None:
    kde_score = kde_score_func(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        bandwidth=bandwidth,
    )
    grids = get_search_points(point_number, fst_range, snd_range)
    kernel_log_densities = kde_score(grids)
    Z = np.exp(kernel_log_densities).reshape(point_number, point_number).T

    x_range = np.linspace(fst_range[0], fst_range[1], point_number)
    y_range = np.linspace(snd_range[0], snd_range[1], point_number)
    fig = go.Figure(data=[go.Surface(z=Z, x=x_range, y=y_range)])

    # cSpell:ignore zaxis
    fig.update_layout(
        title="3D Surface Plot of Kernel Density",
        scene={
            "xaxis": {"title": fst},
            "yaxis": {"title": snd},
            "zaxis": {"title": "kernel density"},
        },
    )
    fig.show()


def calc_minimum_density_point(
    kinematic_states: KinematicStates,
    fst: KinematicKey,
    snd: KinematicKey,
    fst_range: list[float],
    snd_range: list[float],
    shape: Callable[..., bool] = Shape.trivial,
    point_number: int = 30,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
) -> tuple[np.ndarray, float]:
    kernel_densities, filtered_grids = get_kde_scores(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        fst_range=fst_range,
        snd_range=snd_range,
        shape=shape,
        point_number=point_number,
        bandwidth=bandwidth,
    )
    min_value: float = kernel_densities.min()
    min_index = kernel_densities.argmin()  # type: ignore
    min_element = filtered_grids[min_index]
    return min_element.reshape(1, -1)[0], min_value


def calc_percentage_of_density_below_threshold(
    kinematic_states: KinematicStates,
    threshold: float,
    fst: KinematicKey,
    snd: KinematicKey,
    fst_range: list[float],
    snd_range: list[float],
    shape: Callable[..., bool] = Shape.trivial,
    point_number: int = 30,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
) -> float:
    """Return the percentage of points whose density is below the threshold."""
    kernel_densities, _ = get_kde_scores(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        fst_range=fst_range,
        snd_range=snd_range,
        shape=shape,
        point_number=point_number,
        bandwidth=bandwidth,
    )
    return (kernel_densities < threshold).sum() / len(kernel_densities)


def kde_data(
    kinematic_states: KinematicStates,
    fst: KinematicKey,
    snd: KinematicKey,
    fst_range: list[float],
    snd_range: list[float],
    n_points: int = 100,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    kde_score = kde_score_func(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        bandwidth=bandwidth,
    )

    x = np.linspace(fst_range[0], fst_range[1], n_points)
    y = np.linspace(snd_range[0], snd_range[1], n_points)
    X, Y = np.meshgrid(x, y)

    grids = np.column_stack((X.flatten(), Y.flatten())).reshape((n_points**2, 2))
    Z = np.exp(kde_score(grids)).reshape(n_points, n_points)
    return X, Y, Z


def default_range(axis: KinematicKey) -> list[float]:
    """Return the default range of the given axis."""
    if axis == "speed":
        return [0.0, 12.0]
    elif axis == "acc":
        return [-1.5, 1.5]
    elif axis == "steer":
        return [-0.3, 0.3]
    else:
        raise ValueError("Invalid axis")


def visualize_speed_acc(
    fig: plt.Figure,
    ax: plt.Axes,
    kinematic_states: KinematicStates,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
) -> tuple[plt.Figure, plt.Axes]:
    """Visualizes the speed and acceleration of kinematic states using kernel density estimation.

    This function plots a heatmap of the kernel density estimation of speed and acceleration
    values from the given kinematic states. It also overlays an ellipse and marks the minimum
    density point within the elliptical region.

    Parameters:
    fig (plt.Figure): The figure object to be used for plotting.
    ax (plt.Axes): The axes object to be used for plotting.
    kinematic_states (KinematicStates): The kinematic states containing speed and acceleration data.

    Returns:
    tuple[plt.Figure, plt.Axes]: The modified figure and axes objects.
    """
    # config for kernel density estimation
    fst: KinematicKey = "speed"
    snd: KinematicKey = "acc"
    fst_range = default_range(fst)
    snd_range = default_range(snd)

    ellipse: Ellipse = Ellipse(
        xy=Shape.ELLIPSE_CENTER,
        width=Shape.ELLIPSE_WIDTH,
        height=Shape.ELLIPSE_HEIGHT,
        angle=0,
        edgecolor="orange",
        fc="None",
        lw=2,
    )

    # kernel density estimation
    X, Y, Z = kde_data(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        fst_range=fst_range,
        snd_range=snd_range,
        n_points=100,
        bandwidth=bandwidth,
    )

    # plot heatmap
    # cSpell:ignore pcolormesh viridis
    mesh_acc = ax.pcolormesh(X, Y, Z, cmap="viridis", shading="auto")
    fig.colorbar(mesh_acc, ax=ax)

    # plot ellipse
    ax.add_patch(ellipse)

    # Find the minimum value of kernel density in the elliptic region
    min_element, min_val = calc_minimum_density_point(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        fst_range=fst_range,
        snd_range=snd_range,
        shape=lambda x, y: Shape.ellipse(x, y),
        bandwidth=bandwidth,
    )
    rounded_min_val = round(min_val, 4)
    rounded_min_element = [round(min_element[0], 4), round(min_element[1], 4)]

    # plot the minimum point as a white point
    ax.plot(min_element[0], min_element[1], marker="o", color="white", markersize=2)

    ax.set_xlabel(fst, fontsize=12)
    ax.set_ylabel(snd, fontsize=12)
    ax.set_title(f"kernel density: min_density={rounded_min_val} at {rounded_min_element}")

    return fig, ax


def visualize_speed_steer(
    fig: plt.Figure,
    ax: plt.Axes,
    kinematic_states: KinematicStates,
    bandwidth: float | Literal["scott", "silverman"] | ScottCoef = "scott",
) -> tuple[plt.Figure, plt.Axes]:
    """Visualizes the speed and steering angle of kinematic states using kernel density estimation.

    This function plots a heatmap of the kernel density estimation of speed and steering angle
    values from the given kinematic states. It also overlays the convex hull and marks the minimum
    density point within the convex hull.

    Parameters:
    fig (plt.Figure): The figure object to be used for plotting.
    ax (plt.Axes): The axes object to be used for plotting.
    kinematic_states (KinematicStates): The kinematic states containing speed and steering angle data.

    Returns:
    tuple[plt.Figure, plt.Axes]: The modified figure and axes objects.
    """
    # vertexes of convex hull
    POINTS = np.array(
        [
            [1.0, 0.15],
            [4.0, 0.15],
            [11.0, 0.0],
            [1.0, -0.15],
            [4, -0.15],
        ]
    )

    fst: KinematicKey = "speed"
    snd: KinematicKey = "steer"
    fst_range = default_range(fst)
    snd_range = default_range(snd)

    # calculate convex hull
    hull = ConvexHull(POINTS)

    X_STEER, Y_STEER, Z_STEER = kde_data(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        fst_range=fst_range,
        snd_range=snd_range,
        n_points=200,
        bandwidth=bandwidth,
    )

    # plot heatmap
    mesh_steer = ax.pcolormesh(X_STEER, Y_STEER, Z_STEER, cmap="viridis", shading="auto")
    fig.colorbar(mesh_steer, ax=ax)

    # plot edges of the convex hull
    for simplex in hull.simplices:
        ax.plot(POINTS[simplex, 0], POINTS[simplex, 1], color="orange")

    # calculate the minimum value of kernel density in the convex hull
    min_element, min_val = calc_minimum_density_point(
        kinematic_states=kinematic_states,
        fst=fst,
        snd=snd,
        fst_range=fst_range,
        snd_range=snd_range,
        shape=lambda x, y: Shape.convex_hull(POINTS, x, y),
        bandwidth=bandwidth,
    )
    rounded_min_element = [round(min_element[0], 4), round(min_element[1], 4)]

    # plot the minimum point as a white point
    ax.plot(min_element[0], min_element[1], marker="o", color="white", markersize=2)

    ax.set_xlabel(fst, fontsize=12)
    ax.set_ylabel(snd, fontsize=12)
    ax.set_title(f"kernel density: min_density={min_val: .4f} at {rounded_min_element}")
    return fig, ax


def plot_histogram(data: np.ndarray, xlabel: str, title="histogram"):
    bins = int(np.sqrt(data.size))
    plt.hist(data, bins=bins, edgecolor="black")
    plt.xlabel(xlabel)
    plt.ylabel("count")
    plt.title(title)
    plt.grid(True)
    plt.show()


def plot_histogram_of_kde_scores(
    load_dir_list: list[str],
    fst: Literal["speed", "acc", "steer"],
    snd: Literal["speed", "acc", "steer"],
    shape: Callable[..., bool],
    bandwidth: float | ScottCoef,
):
    for load_dir in load_dir_list:
        kinematic_states = load_kinematic_states(load_dir=load_dir)
        data_for_histogram, _ = get_kde_scores(
            kinematic_states=kinematic_states,
            fst=fst,
            snd=snd,
            fst_range=default_range(fst),
            snd_range=default_range(snd),
            shape=shape,
            point_number=30,
            bandwidth=bandwidth,
        )
        plot_histogram(data=data_for_histogram, xlabel="KDE score", title=load_dir)


def correlation_between_scalar_indexes_and_lateral_errors(
    load_dir_list: list[str],
    method: CalcMethodOfScalarIndex,
    fst: Literal["speed", "acc", "steer"],
    snd: Literal["speed", "acc", "steer"],
    fst_range: list[float],
    snd_range: list[float],
    bandwidths: list[float] | list[ScottCoef],
    shape: Callable[..., bool] = Shape.trivial,
) -> np.ndarray:
    accumulation = np.array([])
    for load_dir in load_dir_list:
        # The one that was additionally learned is named "trained"
        kinematic_states, result, trained_result = load_kinematic_states_and_result(load_dir)
        scalar_indexes = calc_scalar_indexes(
            kinematic_states=kinematic_states,
            method=method,
            fst=fst,
            snd=snd,
            fst_range=fst_range,
            snd_range=snd_range,
            bandwidths=bandwidths,
            shape=shape,
        )

        # cSpell:ignore corcoef #
        # Add lateral errors to the end of the vector of scalar indexes for calculating the correlation coefficient with numpy.corcoef
        scalar_indexes = np.append(arr=scalar_indexes, values=[result, trained_result])

        if accumulation.size == 0:
            accumulation = accumulation.reshape(scalar_indexes.shape[0], -1)
        accumulation = np.hstack((accumulation, scalar_indexes.reshape(-1, 1)))

    return np.corrcoef(accumulation)[-1, :-2]


def list_of_scalar_index_and_error(
    load_dir_list: list[str],
    method: CalcMethodOfScalarIndex,
    fst: Literal["speed", "acc", "steer"],
    snd: Literal["speed", "acc", "steer"],
    fst_range: list[float],
    snd_range: list[float],
    bandwidth: float | ScottCoef,
    shape: Callable[..., bool] = Shape.trivial,
) -> np.ndarray:
    """Produce a data of scatter plot of the lateral deviation of the run results and the computed scalar indexes."""
    accumulation = np.array([])

    if method.name == "minimum":
        for load_dir in load_dir_list:
            # The one that was additionally learned is named "trained"
            kinematic_states, result, trained_result = load_kinematic_states_and_result(load_dir)
            min_point, min_val = calc_minimum_density_point(
                kinematic_states=kinematic_states,
                fst=fst,
                snd=snd,
                fst_range=fst_range,
                snd_range=snd_range,
                shape=shape,
                bandwidth=bandwidth,
            )
            accumulation = np.append(accumulation, [trained_result, min_val])

    if method.name == "percentage":
        for load_dir in load_dir_list:
            # The one that was additionally learned is named "trained"
            kinematic_states, result, trained_result = load_kinematic_states_and_result(load_dir)
            index = calc_percentage_of_density_below_threshold(
                kinematic_states=kinematic_states,
                threshold=method.threshold,
                fst=fst,
                snd=snd,
                fst_range=fst_range,
                snd_range=snd_range,
                shape=shape,
                bandwidth=bandwidth,
            )
            accumulation = np.append(accumulation, [trained_result, index])
    return accumulation.reshape(-1, 2)


def plot_scatter_acc(bandwidth: float | ScottCoef, method: CalcMethodOfScalarIndex) -> None:
    data = list_of_scalar_index_and_error(
        load_dir_list=[
            f"test_param_search_{i}_test_vehicle/test_pure_pursuit_figure_eight_sim_test_vehicle_3th"
            for i in range(0, 65)
        ],
        method=method,
        fst="speed",
        snd="acc",
        fst_range=default_range("speed"),
        snd_range=default_range("acc"),
        bandwidth=bandwidth,
        shape=lambda x, y: Shape.ellipse(x, y),
    )
    plt.scatter(data[:, 0], data[:, 1])

    plt.xlabel("Trained lateral error")
    plt.ylabel(method.description())

    plt.title("Kernel density and lateral error")

    plt.show()


def plot_scatter_steer(bandwidth: float | ScottCoef, method: CalcMethodOfScalarIndex) -> None:
    coef = 4.76 / 2.79
    POINTS = np.array(
        [
            [1.0 + 0.3 * 5, coef * (0.15)],
            [4.0, coef * (0.15)],
            [11.0, 0.0],
            [1.0 + 0.3 * 5, coef * -(0.15)],
            [4, coef * -(0.15)],
        ]
    )
    data = list_of_scalar_index_and_error(
        load_dir_list=[
            f"test_param_search_{i}_test_vehicle/test_pure_pursuit_figure_eight_sim_test_vehicle_2th"
            for i in range(0, 65)
        ],
        method=method,
        fst="speed",
        snd="steer",
        fst_range=default_range("speed"),
        snd_range=default_range("steer"),
        bandwidth=bandwidth,
        shape=lambda x, y: Shape.convex_hull(POINTS, x, y),
    )
    plt.scatter(data[:, 0], data[:, 1])

    plt.xlabel("Trained lateral error")
    plt.ylabel(method.description())

    plt.title("Kernel density and lateral error")

    plt.show()
