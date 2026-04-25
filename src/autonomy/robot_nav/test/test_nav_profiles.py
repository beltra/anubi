from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
CONFIG_DIR = PACKAGE_ROOT / "config"


PROFILE_FILES = {
    "default_safe": "nav2_params.yaml",
    "narrow_gap_1p0m": "nav2_params_narrow_gap_1p0m.yaml",
    "forward_static": "nav2_params_forward_static.yaml",
}


ALL_NAV2_YAMLS = [
    "nav2_params.yaml",
    "nav2_params_narrow_gap_1p0m.yaml",
    "nav2_params_forward_static.yaml",
    "nav2_params_2d.yaml",
    "nav2_params_navfn.yaml",
]


def load_yaml(filename):
    with (CONFIG_DIR / filename).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def params(config, *keys):
    value = config
    for key in keys:
        value = value[key]
    return value["ros__parameters"]


def test_profile_files_exist():
    for filename in PROFILE_FILES.values():
        assert (CONFIG_DIR / filename).is_file()


def test_stvl_profiles_have_lifecycle_smoother_and_map_backed_global_costmap():
    for profile, filename in PROFILE_FILES.items():
        config = load_yaml(filename)

        smoother = params(config, "smoother_server")
        assert smoother["smoother_plugins"] == ["simple_smoother"]

        lifecycle_nodes = params(config, "lifecycle_manager")["node_names"]
        assert "smoother_server" in lifecycle_nodes

        global_costmap = params(config, "global_costmap", "global_costmap")
        assert global_costmap["rolling_window"] is False, profile
        assert global_costmap["plugins"] == ["static_layer", "voxel_layer", "inflation_layer"]
        assert global_costmap["static_layer"]["plugin"] == "nav2_costmap_2d::StaticLayer"
        assert global_costmap["static_layer"]["map_subscribe_transient_local"] is True
        assert global_costmap["voxel_layer"]["z_voxels"] <= 16
        assert global_costmap["voxel_layer"]["cloud"]["topic"] == "/lidar/points_undistorted"


def test_default_safe_preserves_omni_lattice_stack():
    config = load_yaml("nav2_params.yaml")

    planner = params(config, "planner_server")["GridBased"]
    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerLattice"

    controller = params(config, "controller_server")
    follow_path = controller["FollowPath"]
    assert controller["costmap_update_timeout"] == 0.30
    assert controller["speed_limit_topic"] == "speed_limit"
    assert follow_path["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert follow_path["primary_controller"] == "nav2_mppi_controller::MPPIController"
    assert follow_path["motion_model"] == "Omni"


def test_forward_static_is_diffdrive_smac2d_with_pointcloud_costmaps():
    config = load_yaml("nav2_params_forward_static.yaml")

    planner = params(config, "planner_server")["GridBased"]
    assert planner["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert planner["motion_model_for_search"] == "VON_NEUMANN"

    controller = params(config, "controller_server")
    follow_path = controller["FollowPath"]
    assert controller["costmap_update_timeout"] == 0.30
    assert controller["speed_limit_topic"] == "speed_limit"
    assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
    assert follow_path["motion_model"] == "DiffDrive"
    assert follow_path["vy_max"] == 0.0
    assert "PreferForwardCritic" in follow_path["critics"]

    local_costmap = params(config, "local_costmap", "local_costmap")
    assert local_costmap["plugins"] == ["stvl_layer", "inflation_layer"]
    assert local_costmap["stvl_layer"]["lidar_mark"]["topic"] == "/lidar/points_undistorted"

    rendered = (CONFIG_DIR / "nav2_params_forward_static.yaml").read_text(encoding="utf-8")
    assert "/front_camera/scan" not in rendered
    assert "/back_camera/scan" not in rendered


def test_all_nav2_yamls_have_route_server_scaffold():
    for filename in ALL_NAV2_YAMLS:
        config = load_yaml(filename)
        route_server = params(config, "route_server")
        assert route_server["graph_filepath"] == ""
        assert route_server["graph_file_loader"] == "GeoJsonGraphFileLoader"
        assert (
            route_server["GeoJsonGraphFileLoader"]["plugin"]
            == "nav2_route::GeoJsonGraphFileLoader"
        )
        assert route_server["edge_cost_functions"] == ["DistanceScorer", "CostmapScorer"]
        assert route_server["DistanceScorer"]["plugin"] == "nav2_route::DistanceScorer"
        assert route_server["CostmapScorer"]["plugin"] == "nav2_route::CostmapScorer"
        assert route_server["operations"] == ["ReroutingService"]
        assert route_server["ReroutingService"]["plugin"] == "nav2_route::ReroutingService"


def test_route_server_args_propagate_through_launch_and_scripts():
    nav2_launch = (PACKAGE_ROOT / "launch" / "nav2.launch.py").read_text(encoding="utf-8")
    sim_launch = (
        WORKSPACE_ROOT / "src" / "autonomy" / "robot_sim" / "launch" / "sim_bringup.launch.py"
    ).read_text(encoding="utf-8")
    real_launch = (
        WORKSPACE_ROOT / "src" / "platform" / "go2_bringup" / "launch" / "go2_bringup.launch.py"
    ).read_text(encoding="utf-8")
    sim_script = (WORKSPACE_ROOT / "scripts" / "run_go2_sim.sh").read_text(encoding="utf-8")
    real_script = (WORKSPACE_ROOT / "scripts" / "run_go2_real.sh").read_text(encoding="utf-8")

    for content in [nav2_launch, sim_launch, real_launch, sim_script, real_script]:
        assert "use_route_server" in content
        assert "route_graph_file" in content

    for script in [sim_script, real_script]:
        assert 'use_route_server:="$USE_ROUTE_SERVER"' in script
        assert 'if [[ -n "$ROUTE_GRAPH_FILE" ]]; then' in script
        assert 'CMD+=(route_graph_file:="$ROUTE_GRAPH_FILE")' in script
