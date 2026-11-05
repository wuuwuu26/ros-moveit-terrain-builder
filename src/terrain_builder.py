#!/usr/bin/env python3
"""
Automated Terrain Reconstruction from Contour Topographic Map

Builds a 3D voxel terrain by stacking small blocks layer-by-layer using a
Franka Emika Panda robotic arm in MoveIt/RViz simulation.

The system:
1. Processes a contour map → 2D heightmap matrix (via contour_to_heightmap.py)
2. Constructs the terrain layer-by-layer from bottom to top
3. Uses dynamic collision column management to avoid knocking over placed blocks
4. Uses Cartesian paths for precise vertical placement

Grid mapping:
  - Row i → World X:  origin_x + i * grid_spacing
  - Col j → World Y:  origin_y + j * grid_spacing
  - Layer h → World Z: (h - 0.5) * block_size

Block parameters:
  - Cube size: L = 0.04m
  - Grid spacing: 0.04m (center-to-center, blocks touch — Minecraft-style)

Run with:
  roslaunch panda_moveit_config terrain_builder_demo.launch
"""

from __future__ import print_function

import sys
import os
import copy
import time
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, sin, cos, sqrt
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState

# Import quaternion_from_euler from tf, with fallback
try:
    from tf.transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(roll, pitch, yaw):
        cy = cos(yaw * 0.5); sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5); sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5); sr = sin(roll * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]

# TF2 for fast end-effector tracking (no polling latency)
import tf2_ros

# Import the contour processing module
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
import contour_to_heightmap as contour_proc


# ---------------------------------------------------------------------------
# Terrain Builder
# ---------------------------------------------------------------------------

class TerrainBuilder(object):
    """Layer-by-layer terrain construction using a Panda robot arm."""

    def __init__(self, grid_size=8, num_levels=8):
        """Initialize MoveIt interfaces and compute the heightmap."""
        super(TerrainBuilder, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("terrain_builder", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)

        # Direct joint state publisher for fake gripper control
        self._joint_pub = rospy.Publisher(
            "/move_group/fake_controller_joint_states",
            JointState, queue_size=10)

        # Configure planner for speed with smooth motion (interpolate)
        self.move_group.set_num_planning_attempts(1)
        self.move_group.set_planning_time(2.0)
        self.move_group.allow_replanning(True)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()

        rospy.loginfo("Planning frame: %s", self.planning_frame)
        rospy.loginfo("End effector:  %s", self.eef_link)
        rospy.loginfo("Groups:        %s", self.robot.get_group_names())

        # -----------------------------------------------------------------
        # Terrain parameters
        # -----------------------------------------------------------------
        self.grid_size = grid_size
        self.num_levels = num_levels
        self.block_size = 0.02        # L (cube side, meters)
        self.grid_spacing = 0.02      # center-to-center (blocks touch — Minecraft-style)
        self.approach_height = 0.10   # vertical clearance above placement
        self.table_z = 0.05           # table surface Z (raised for IK reachability)

        # Fixed grasp orientation — ensures vertical approach and prevents
        # the block from rotating relative to the gripper during transport
        self.GRASP_ROLL = pi
        self.GRASP_PITCH = 0.0
        self.GRASP_YAW = pi / 4.0
        # Pre-compute EEF Z-axis direction in world frame for this orientation
        q = quaternion_from_euler(self.GRASP_ROLL, self.GRASP_PITCH,
                                  self.GRASP_YAW)
        self._grasp_zx = 2.0 * (q[0] * q[2] + q[1] * q[3])
        self._grasp_zy = 2.0 * (q[1] * q[2] - q[0] * q[3])
        self._grasp_zz = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])

        # Terrain origin: world (X,Y) of grid cell (0,0)
        # With 32×32 at 0.02m spacing: span = 31 × 0.02 = 0.62m
        # Positioned close enough that farthest cell (x=0.87) is within Panda reach (~0.85m)
        terrain_span = (grid_size - 1) * self.grid_spacing
        self.origin_x = 0.25          # world X of grid row 0 (forward)
        self.origin_y = -terrain_span / 2.0  # center in Y

        # Supply position: block sits on the table surface
        self.supply_x = 0.30
        self.supply_y = 0.35
        self.supply_z = self.table_z + self.block_size / 2.0

        # -----------------------------------------------------------------
        # State tracking
        # -----------------------------------------------------------------
        # heightmap[i,j] = total blocks needed at cell
        self.heightmap = None
        # current_height[i,j] = blocks already placed at cell
        self.current_heights = None
        # Total blocks placed so far
        self.blocks_placed = 0
        self.total_blocks = 0
        # Supply block counter (for naming)
        self._supply_counter = 0
        # Column collision object names
        self._col_names = {}
        # MarkerArray publisher for colored block visualization
        self._marker_pub = rospy.Publisher(
            "/terrain_markers", MarkerArray, queue_size=100)
        self._terrain_markers = []   # accumulated Marker messages
        self._marker_id_counter = 0  # unique marker IDs
        self._supply_marker_ns_id = None  # (ns, id) of current supply marker
        # Separate publisher for the block held during transport
        self._held_marker_pub = rospy.Publisher(
            "/held_block_marker", MarkerArray, queue_size=10)
        # TF2-based tracking — avoids get_current_pose() latency.
        # Uses the TF tree that robot_state_publisher updates at high frequency.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # Polling-based tracking (timer reference)
        self._z_offset = 0.103       # panda_link8 to gripper center
        self._tracking_timer = None  # rospy.Timer, started when carrying
        self._block_held = False
        self._held_marker = None

    # -------------------------------------------------------------------
    # Visualization (colored markers)
    # -------------------------------------------------------------------

    def _color_by_height(self, layer):
        """
        Return the RGB color for a block at the given terrain layer (1-based).

        Uses a natural elevation gradient: earth → grass → rock → snow.
        Colors are linearly interpolated between the 8 canonical stops so
        intermediate layer counts (e.g., num_levels=6) also get smooth blends.

        :param layer: Integer layer number, 1 = base, N = peak
        :return: RGB tuple (r, g, b)
        """
        # Canonical color stops at layers 1 through 8
        stops = {
            1: (0.35, 0.20, 0.10),   # dark brown / earth
            2: (0.45, 0.30, 0.15),   # brown
            3: (0.55, 0.40, 0.20),   # light brown / tan
            4: (0.50, 0.55, 0.25),   # yellow-green / grass
            5: (0.35, 0.60, 0.30),   # green
            6: (0.20, 0.50, 0.25),   # dark green / forest
            7: (0.60, 0.60, 0.65),   # gray / rock
            8: (0.95, 0.95, 0.95),   # white / snow
        }
        # Clamp to valid range
        layer = max(1, min(layer, max(stops.keys())))

        # Exact match
        if layer in stops:
            return stops[layer]

        # Linear interpolation between surrounding stops
        lower = int(layer)
        upper = lower + 1
        t = layer - lower  # fractional part
        r = stops[lower][0] + t * (stops[upper][0] - stops[lower][0])
        g = stops[lower][1] + t * (stops[upper][1] - stops[lower][1])
        b = stops[lower][2] + t * (stops[upper][2] - stops[lower][2])
        return (r, g, b)

    def _make_cube_marker(self, name, x, y, z, r, g, b, a=1.0):
        """
        Create a colored cube Marker at the given world position.

        Returns a Marker message (not published — caller accumulates and publishes).
        """
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "terrain"
        marker.id = self._marker_id_counter
        self._marker_id_counter += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        s = self.block_size * 1.02  # slightly oversize to cover collision obj
        marker.scale.x = s
        marker.scale.y = s
        marker.scale.z = s
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.lifetime = rospy.Duration(0)  # persistent
        return marker

    def _publish_markers(self):
        """Publish all accumulated terrain markers as a MarkerArray."""
        ma = MarkerArray(markers=self._terrain_markers)
        self._marker_pub.publish(ma)

    # -------------------------------------------------------------------
    # Block tracking — marker follows EEF during transport via polling
    # -------------------------------------------------------------------

    def _start_tracking(self, marker):
        """Begin tracking — marker follows EEF via fast TF polling at 50Hz."""
        self._held_marker = marker
        self._block_held = True
        # Faster timer (50Hz) for smoother tracking with low-latency TF lookups
        self._tracking_timer = rospy.Timer(
            rospy.Duration(0.02), self._tracking_cb)

    def _snap_marker(self):
        """Force one immediate marker update at the current EEF pose."""
        if not self._block_held or self._held_marker is None:
            return
        self._tracking_cb(None)

    def _stop_tracking(self):
        """Stop polling and delete ALL held markers from RViz."""
        self._block_held = False
        if self._tracking_timer is not None:
            self._tracking_timer.shutdown()
            self._tracking_timer = None
        # DELETEALL — robust cleanup of any lingering held-block markers
        delete_all = Marker()
        delete_all.header.frame_id = self.planning_frame
        delete_all.action = Marker.DELETEALL
        self._held_marker_pub.publish(MarkerArray(markers=[delete_all]))
        self._held_marker = None

    def _tracking_cb(self, event):
        """
        Timer callback: update held marker at gripper center via TF lookup.

        Uses tf2_ros.Buffer.lookup_transform() which reads directly from the
        TF tree (populated by robot_state_publisher). This is much faster than
        get_current_pose() because it's a simple buffer read — no service
        calls, no planning scene monitor polling.
        """
        if not self._block_held or self._held_marker is None:
            return
        try:
            # Fast TF lookup — latest available transform
            transform = self._tf_buffer.lookup_transform(
                self.planning_frame,
                self.eef_link,
                rospy.Time(0))  # latest
            # Block center = EEF position + z_offset along EEF Z-axis direction
            q = transform.transform.rotation
            zx = 2.0 * (q.x * q.z + q.y * q.w)
            zy = 2.0 * (q.y * q.z - q.x * q.w)
            zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            m = self._held_marker
            m.header.stamp = rospy.Time.now()
            m.pose.position.x = transform.transform.translation.x + self._z_offset * zx
            m.pose.position.y = transform.transform.translation.y + self._z_offset * zy
            m.pose.position.z = transform.transform.translation.z + self._z_offset * zz
            self._held_marker_pub.publish(MarkerArray(markers=[m]))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            # TF not available yet (e.g., during startup) — skip this frame
            pass
        except Exception:
            pass

    # -------------------------------------------------------------------
    # Gravity drop animation
    # -------------------------------------------------------------------

    def _animate_fall(self, name, start_z, end_x, end_y, end_z, r, g, b):
        """
        Animate a block falling straight down with gravitational acceleration
        from start_z to end_z at position (end_x, end_y).

        The block drops vertically — X and Y stay fixed at the target cell.
        """
        fall_dist = start_z - end_z
        if fall_dist <= 0.001:
            # No fall needed — place directly
            return

        g = 9.81  # gravitational acceleration (m/s²)
        t_total = sqrt(2.0 * fall_dist / g)  # total fall time
        dt = 0.03  # 30Hz animation
        steps = max(3, int(t_total / dt))
        dt = t_total / steps

        # Create a temporary falling marker
        fall_marker = Marker()
        fall_marker.header.frame_id = self.planning_frame
        fall_marker.ns = "falling"
        fall_marker.id = self._marker_id_counter
        self._marker_id_counter += 1
        fall_marker.type = Marker.CUBE
        fall_marker.action = Marker.ADD
        fall_marker.scale.x = self.block_size
        fall_marker.scale.y = self.block_size
        fall_marker.scale.z = self.block_size
        fall_marker.color.r = r
        fall_marker.color.g = g
        fall_marker.color.b = b
        fall_marker.color.a = 1.0
        fall_marker.lifetime = rospy.Duration(2.0)  # auto-expire after animation

        for step in range(steps + 1):
            t = step * dt
            z = start_z - 0.5 * g * t * t
            fall_marker.header.stamp = rospy.Time.now()  # resets lifetime each frame
            fall_marker.pose.position.x = end_x
            fall_marker.pose.position.y = end_y
            fall_marker.pose.position.z = max(z, end_z)
            fall_marker.pose.orientation.w = 1.0
            self._marker_pub.publish(MarkerArray(markers=[fall_marker]))
            rospy.sleep(dt)

    # -------------------------------------------------------------------
    # Heightmap generation
    # -------------------------------------------------------------------

    def generate_heightmap(self, shape_type="mountain"):
        """
        Generate the heightmap from a synthetic contour map.

        :param shape_type: "mountain" (concentric ellipses) or "pyramid"
        """
        rospy.loginfo("Generating contour heightmap (%dx%d, max %d levels)...",
                       self.grid_size, self.grid_size, self.num_levels)

        self.heightmap = contour_proc.generate_sample_heightmap(
            grid_size=self.grid_size,
            num_levels=self.num_levels,
            shape_type=shape_type,
            output_dir=SCRIPT_DIR,
        )
        self.total_blocks = int(self.heightmap.sum())
        self.current_heights = np.zeros((self.grid_size, self.grid_size),
                                         dtype=np.int32)

        rospy.loginfo("Heightmap: %d total blocks across %d filled cells",
                       self.total_blocks,
                       int((self.heightmap > 0).sum()))

    def load_heightmap(self, npy_path):
        """Load a pre-computed heightmap from a .npy file."""
        self.heightmap = np.load(npy_path)
        self.grid_size = self.heightmap.shape[0]
        self.total_blocks = int(self.heightmap.sum())
        self.current_heights = np.zeros((self.grid_size, self.grid_size),
                                         dtype=np.int32)
        rospy.loginfo("Loaded heightmap from %s: %d blocks", npy_path,
                       self.total_blocks)

    # -------------------------------------------------------------------
    # Scene setup
    # -------------------------------------------------------------------

    def setup_scene(self):
        """Initialize the planning scene with the table and supply block."""
        rospy.loginfo("Setting up planning scene...")
        self.scene.remove_world_object()
        rospy.sleep(0.5)

        # Add table surface (centered under terrain + supply area)
        # Table center at (0.56, 0.0) sits under the 32×32 grid spanning
        # x: 0.25–0.87, y: -0.31–0.31 with supply at x=0.30, y=0.35
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = self.planning_frame
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0.56
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = self.table_z - 0.015
        self.scene.add_box("table", table_pose, size=(1.0, 0.9, 0.03))
        # Add a grass green table marker for visualization
        table_marker = self._make_cube_marker(
            "table", 0.56, 0.0, self.table_z - 0.015,
            0.30, 0.45, 0.20)  # grass green
        table_marker.ns = "table"
        table_marker.scale.x = 1.0
        table_marker.scale.y = 0.9
        table_marker.scale.z = 0.03
        self._terrain_markers.append(table_marker)
        self._publish_markers()
        rospy.sleep(0.3)
        rospy.loginfo("Table added.")

        # Place initial supply block
        self._spawn_supply_block()
        rospy.loginfo("Scene ready.")

    def _spawn_supply_block(self):
        """Create a new block (collision + visible marker) at the supply position."""
        name = "supply_%d" % self._supply_counter
        self._supply_counter += 1

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = self.supply_x
        pose.pose.position.y = self.supply_y
        pose.pose.position.z = self.supply_z

        self.scene.add_box(name, pose, size=(self.block_size,) * 3)
        self._wait_for_scene(name, known=True)

        # Add a visible supply marker (neutral gray) to the terrain topic
        sm = Marker()
        sm.header.frame_id = self.planning_frame
        sm.ns = "supply"
        sm.id = self._marker_id_counter
        self._marker_id_counter += 1
        sm.type = Marker.CUBE
        sm.action = Marker.ADD
        sm.pose.position.x = self.supply_x
        sm.pose.position.y = self.supply_y
        sm.pose.position.z = self.supply_z
        sm.pose.orientation.w = 1.0
        sm.scale.x = self.block_size
        sm.scale.y = self.block_size
        sm.scale.z = self.block_size
        sm.color.r = 0.5; sm.color.g = 0.5; sm.color.b = 0.5; sm.color.a = 1.0
        sm.lifetime = rospy.Duration(0)  # persistent until deleted
        self._supply_marker_ns_id = (sm.ns, sm.id)  # remember for deletion
        self._marker_pub.publish(MarkerArray(markers=[sm]))
        return name

    # -------------------------------------------------------------------
    # Column collision management
    # -------------------------------------------------------------------

    def _col_name(self, i, j):
        """Get the collision object name for grid column (i,j)."""
        return "col_%d_%d" % (i, j)

    def _cell_xy(self, i, j):
        """Get world (x, y) position of grid cell (i,j) center."""
        return (self.origin_x + i * self.grid_spacing,
                self.origin_y + j * self.grid_spacing)

    def _remove_column(self, i, j):
        """Temporarily remove the collision column at (i,j)."""
        name = self._col_name(i, j)
        if name in self.scene.get_known_object_names():
            self.scene.remove_world_object(name)
            self._wait_for_scene(name, known=False)

    def _update_column(self, i, j, height_in_blocks):
        """
        Add or update the collision column at (i,j) to the given height.

        The column is a single tall box from z=0 to z=height_in_blocks * L.
        If height is 0, the column is removed.

        Collision columns are narrower than visual blocks (50% width) to
        give the motion planner clearance between touching blocks.
        """
        name = self._col_name(i, j)
        cx, cy = self._cell_xy(i, j)
        block_h = height_in_blocks * self.block_size

        # Remove old column first
        if name in self.scene.get_known_object_names():
            self.scene.remove_world_object(name)
            self._wait_for_scene(name, known=False)

        if block_h > 0:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = self.planning_frame
            pose.pose.orientation.w = 1.0
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = self.table_z + block_h / 2.0  # center of column on table
            # Half-width columns create gaps for the planner between
            # visually-touching blocks (Minecraft look, robot-safe paths)
            col_w = self.block_size * 0.5
            self.scene.add_box(name, pose, size=(col_w, col_w, block_h))
            self._wait_for_scene(name, known=True)

    # -------------------------------------------------------------------
    # Robot motion primitives
    # -------------------------------------------------------------------

    GRIP_OPEN = 0.035   # finger joint position when fully open
    GRIP_CLOSED = 0.006  # finger joint position when gripping a 0.02m block

    def _publish_finger(self, position):
        """Publish a single finger joint state to the fake controller."""
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = ["panda_finger_joint1"]
        js.position = [position]
        js.velocity = [0.0]
        self._joint_pub.publish(js)

    def open_gripper(self):
        """Open the Panda hand via direct fake-controller joint command."""
        for _ in range(3):
            self._publish_finger(self.GRIP_OPEN)
            rospy.sleep(0.03)

    def close_gripper(self):
        """Close the Panda hand via direct fake-controller joint command."""
        for _ in range(3):
            self._publish_finger(self.GRIP_CLOSED)
            rospy.sleep(0.03)

    def _wait_for_scene(self, name, known=True, attached=False, timeout=2.0):
        """Wait for scene update (reduced timeout for speed)."""
        start = rospy.get_time()
        while (rospy.get_time() - start < timeout) and not rospy.is_shutdown():
            is_attached = len(self.scene.get_attached_objects([name]).keys()) > 0
            is_known = name in self.scene.get_known_object_names()
            if (attached == is_attached) and (known == is_known):
                return True
            rospy.sleep(0.05)
        return False

    def move_to_position(self, x, y, z):
        """
        Move the end effector to a target XYZ position using position-only IK.
        This leaves orientation free, giving the IK solver 3 extra DOF and
        dramatically improving reachability compared to full pose constraints.
        """
        # Position-only target: solver is free to choose orientation
        self.move_group.set_position_target([x, y, z])
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def center_block_above(self, cx, cy, z_target):
        """
        Move so the BLOCK is centered above (cx, cy, z_target).

        Uses full-pose IK with a fixed top-down grasp orientation (same as
        the reference). The flange position is computed from the known
        orientation so the block lands exactly at the target.

        Falls back to position-only IK if the full pose is unreachable.
        """
        # Compute flange position that places the block at (cx, cy, z_target)
        flange_x = cx - self._z_offset * self._grasp_zx
        flange_y = cy - self._z_offset * self._grasp_zy
        flange_z = z_target - self._z_offset * self._grasp_zz

        # Full-pose IK with fixed top-down orientation
        q = quaternion_from_euler(self.GRASP_ROLL, self.GRASP_PITCH,
                                  self.GRASP_YAW)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = flange_x
        pose.position.y = flange_y
        pose.position.z = flange_z
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.move_group.set_pose_target(pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if not success:
            # Fallback: position-only IK for hard-to-reach cells
            rospy.logwarn("Full-pose IK failed for (%.2f, %.2f, %.2f), "
                          "falling back to position-only", cx, cy, z_target)
            return self.move_to_position(cx, cy, z_target)
        return True

    def cartesian_move(self, dz):
        """Move end effector vertically by dz meters (positive=up)."""
        current = self.move_group.get_current_pose().pose
        target = copy.deepcopy(current)
        target.position.z += dz

        (plan, fraction) = self.move_group.compute_cartesian_path(
            [target], 0.005, True
        )
        if plan and len(plan.joint_trajectory.points) > 0:
            return self.move_group.execute(plan, wait=True)
        return False

    def cartesian_move_to_z(self, target_z):
        """
        Move the end effector to an absolute Z via Cartesian straight-line path.

        Unlike cartesian_move(dz), this targets a precise Z coordinate regardless
        of where motion planning tolerances left the arm. Critical for accurate
        pick/place where the block must be reached exactly.

        :param target_z: Absolute flange Z position in the planning frame
        :return: True on success
        """
        current = self.move_group.get_current_pose().pose
        delta_z = abs(target_z - current.position.z)

        # Skip if already within 1 mm tolerance
        if delta_z <= 0.001:
            rospy.loginfo("Already at Z=%.3f m (within 1mm), skipping move.",
                          target_z)
            return True

        target = copy.deepcopy(current)
        target.position.z = target_z

        rospy.loginfo("Cartesian move to Z=%.3f m (delta: %.3f m)...",
                       target_z, delta_z)
        (plan, fraction) = self.move_group.compute_cartesian_path(
            [target], 0.005, True
        )
        if fraction < 1.0:
            rospy.logwarn("Cartesian path only %.1f%% feasible!",
                          fraction * 100)

        if plan and len(plan.joint_trajectory.points) > 0:
            return self.move_group.execute(plan, wait=True)
        return False

    # -------------------------------------------------------------------
    # Pick and place
    # -------------------------------------------------------------------

    def pick_from_supply(self):
        """
        Pick a block from the supply area.

        Uses position-only IK to approach, then a Cartesian descent to grasp.
        The collision object is REMOVED during transport and a colored marker
        follows the EEF via a polling timer (no TF dependency, works with any
        arm orientation).

        Returns the supply block name, or None on failure.
        """
        supply_name = "supply_%d" % (self._supply_counter - 1)

        # Ensure supply block exists
        if supply_name not in self.scene.get_known_object_names():
            supply_name = self._spawn_supply_block()

        # Step 1: Move so the gripper is centered above the supply block.
        # center_block_above already accounts for _z_offset internally, so we
        # pass the desired block-center Z directly (supply_z + approach_height).
        pre_z = self.supply_z + self.approach_height
        if not self.center_block_above(self.supply_x, self.supply_y, pre_z):
            rospy.logerr("Failed to reach pre-grasp above supply!")
            return None

        # Step 2: Open gripper, descend to exact grasp position using absolute
        # Z Cartesian move (NOT relative — that accumulates IK tolerance error).
        self.open_gripper()
        rospy.sleep(0.05)
        # Flange Z that puts the gripper center exactly at the supply block center
        grasp_flange_z = self.supply_z - self._z_offset * self._grasp_zz
        if not self.cartesian_move_to_z(grasp_flange_z):
            rospy.logerr("Failed to descend to grasp position!")
            return None

        # Step 3: Close gripper to grasp
        self.close_gripper()
        rospy.sleep(0.05)

        # Step 4: Create held marker at grasp position FIRST (no visual gap).
        # Use TF for fast, accurate pose without get_current_pose() latency.
        try:
            transform = self._tf_buffer.lookup_transform(
                self.planning_frame, self.eef_link, rospy.Time(0))
            q = transform.transform.rotation
            zx = 2.0 * (q.x * q.z + q.y * q.w)
            zy = 2.0 * (q.y * q.z - q.x * q.w)
            zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            # Neutral gray for held marker (layer color applied at placement)
            held_marker = self._make_cube_marker(
                supply_name,
                transform.transform.translation.x + self._z_offset * zx,
                transform.transform.translation.y + self._z_offset * zy,
                transform.transform.translation.z + self._z_offset * zz,
                0.5, 0.5, 0.5)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Fallback to get_current_pose if TF not available
            current = self.move_group.get_current_pose().pose
            q = current.orientation
            zx = 2.0 * (q.x * q.z + q.y * q.w)
            zy = 2.0 * (q.y * q.z - q.x * q.w)
            zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            held_marker = self._make_cube_marker(
                supply_name,
                current.position.x + self._z_offset * zx,
                current.position.y + self._z_offset * zy,
                current.position.z + self._z_offset * zz,
                0.5, 0.5, 0.5)
        held_marker.ns = "held"
        self._start_tracking(held_marker)

        # Step 5: THEN remove collision object + supply marker
        self.scene.remove_world_object(supply_name)
        self._wait_for_scene(supply_name, known=False)
        if self._supply_marker_ns_id is not None:
            delete = Marker()
            delete.header.frame_id = self.planning_frame
            delete.ns = self._supply_marker_ns_id[0]
            delete.id = self._supply_marker_ns_id[1]
            delete.action = Marker.DELETE
            self._marker_pub.publish(MarkerArray(markers=[delete]))
            self._supply_marker_ns_id = None

        # Step 6: Retreat up, then snap marker to final position
        self.cartesian_move(self.approach_height + 0.05)
        self._snap_marker()

        return supply_name

    def place_block_at(self, block_name, i, j, layer):
        """
        Place a block at grid cell (i,j).

        The held marker follows the EEF during the move (polling-based).
        Once above the target, the gripper opens, tracking stops, and the
        block's collision object + static terrain marker are created at
        the exact target position.

        :param block_name: Name of the carried block
        :param i, j: Grid cell indices
        :param layer: Layer number (1-based)
        :return: True on success
        """
        cx, cy = self._cell_xy(i, j)
        place_z = self.table_z + (layer - 0.5) * self.block_size
        drop_z = place_z + self.approach_height + 0.04

        # Step 1: Temporarily remove collision column at (i,j)
        self._remove_column(i, j)

        # Step 2: Move so the BLOCK is centered above the target cell
        if not self.center_block_above(cx, cy, drop_z):
            rospy.logerr("Failed to reach drop position at (%d,%d)!", i, j)
            self._stop_tracking()
            self._update_column(i, j, self.current_heights[i, j])
            return False

        # Step 3: Capture block's current position via fast TF lookup, then release.
        # Use TF to get accurate block position without get_current_pose() latency.
        try:
            transform = self._tf_buffer.lookup_transform(
                self.planning_frame, self.eef_link, rospy.Time(0))
            q = transform.transform.rotation
            zx = 2.0 * (q.x * q.z + q.y * q.w)
            zy = 2.0 * (q.y * q.z - q.x * q.w)
            zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            release_x = transform.transform.translation.x + self._z_offset * zx
            release_y = transform.transform.translation.y + self._z_offset * zy
            release_z = transform.transform.translation.z + self._z_offset * zz
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            current = self.move_group.get_current_pose().pose
            q = current.orientation
            zx = 2.0 * (q.x * q.z + q.y * q.w)
            zy = 2.0 * (q.y * q.z - q.x * q.w)
            zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            release_x = current.position.x + self._z_offset * zx
            release_y = current.position.y + self._z_offset * zy
            release_z = current.position.z + self._z_offset * zz

        self.open_gripper()
        rospy.sleep(0.05)
        self._stop_tracking()

        # Step 4: Animate block falling with gravitational acceleration
        r, g, b = self._color_by_height(layer)
        self._animate_fall(block_name, release_z, cx, cy, place_z, r, g, b)

        # Step 5: Add collision object at the exact target position
        final_pose = geometry_msgs.msg.PoseStamped()
        final_pose.header.frame_id = self.planning_frame
        final_pose.pose.orientation.w = 1.0
        final_pose.pose.position.x = cx
        final_pose.pose.position.y = cy
        final_pose.pose.position.z = place_z
        self.scene.add_box(block_name, final_pose,
                           size=(self.block_size,) * 3)
        self._wait_for_scene(block_name, known=True)

        # Step 6: Update column collision at new height
        self.current_heights[i, j] += 1
        self._update_column(i, j, self.current_heights[i, j])
        self.blocks_placed += 1

        # Step 7: Add a colored terrain marker at the final position
        marker = self._make_cube_marker(block_name, cx, cy, place_z, r, g, b)
        self._terrain_markers.append(marker)
        self._publish_markers()

        # Step 8: Retreat upward to clear the column before next pick cycle.
        # Without this, the arm stays at drop_z inside the collision column,
        # causing planning failures (block "stuck" above the target) on the
        # next iteration.
        self.cartesian_move(self.approach_height + 0.05)

        return True

    # -------------------------------------------------------------------
    # Terrain construction
    # -------------------------------------------------------------------

    def build_terrain(self):
        """
        Build the full terrain layer-by-layer.

        Within each layer, cells are built back-to-front (high row index to
        low) so the robot approaches from the clear front side, avoiding
        same-layer neighbor collisions with tight-packed blocks.
        """
        if self.heightmap is None:
            rospy.logerr("No heightmap! Call generate_heightmap() or load_heightmap() first.")
            return

        max_h = int(self.heightmap.max())
        rospy.loginfo("=" * 60)
        rospy.loginfo("TERRAIN CONSTRUCTION STARTING")
        rospy.loginfo("  Grid:    %d x %d  | Spacing: %.2f m",
                       self.grid_size, self.grid_size, self.grid_spacing)
        rospy.loginfo("  Layers:  %d (1 to %d)", max_h, max_h)
        rospy.loginfo("  Blocks:  %d total", self.total_blocks)
        rospy.loginfo("=" * 60)

        # Pre-build layer lists — cells sorted BACK-TO-FRONT for clearance
        layer_blocks = []
        for layer in range(1, max_h + 1):
            cells = [(i, j) for i in range(self.grid_size)
                     for j in range(self.grid_size)
                     if self.heightmap[i, j] >= layer]
            # Sort: highest row index first (back of terrain → front)
            cells.sort(key=lambda ij: -ij[0])
            layer_blocks.append((layer, cells))
            rospy.loginfo("  Layer %d: %d blocks (back→front order)",
                           layer, len(cells))

        start_time = time.time()

        for layer, cells in layer_blocks:
            rospy.loginfo("")
            rospy.loginfo("-" * 50)
            rospy.loginfo("BUILDING LAYER %d/%d (%d blocks)",
                           layer, max_h, len(cells))
            rospy.loginfo("-" * 50)

            for idx, (i, j) in enumerate(cells):
                # Pick block from supply
                block_name = self.pick_from_supply()
                if block_name is None:
                    rospy.logerr("Supply pick failed! Aborting layer.")
                    return

                # Place at target cell
                success = self.place_block_at(block_name, i, j, layer)
                if not success:
                    rospy.logerr("Placement failed at cell (%d,%d)! Continuing...", i, j)

                # Spawn next supply block for next iteration
                self._spawn_supply_block()

                # Progress report every 10 blocks
                if self.blocks_placed % 10 == 0:
                    elapsed = time.time() - start_time
                    rate = self.blocks_placed / max(elapsed, 0.1)
                    remaining = (self.total_blocks - self.blocks_placed) / max(rate, 0.01)
                    rospy.loginfo("PROGRESS: %d/%d blocks (%.1f%%) | %.1f blk/s | ~%.0fs left",
                                   self.blocks_placed, self.total_blocks,
                                   100.0 * self.blocks_placed / self.total_blocks,
                                   rate, remaining)

        elapsed = time.time() - start_time
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("TERRAIN COMPLETE!")
        rospy.loginfo("  %d blocks in %.1fs (%.1f blk/s)",
                       self.blocks_placed, elapsed,
                       self.blocks_placed / max(elapsed, 0.1))
        rospy.loginfo("=" * 60)

    # -------------------------------------------------------------------
    # Main
    # -------------------------------------------------------------------

    def run(self):
        """Main entry point."""
        rospy.loginfo("Waiting for initialization (2s)...")
        rospy.sleep(2)

        # Generate heightmap from synthetic contour map
        self.generate_heightmap(shape_type="mountain")

        # Set up the scene
        self.setup_scene()

        # Build the terrain
        rospy.sleep(0.5)
        self.build_terrain()

        rospy.loginfo("Demo complete. Press Ctrl-C to exit.")
        rospy.spin()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    try:
        builder = TerrainBuilder(grid_size=32, num_levels=8)
        builder.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Stopped by user.")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
