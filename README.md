# NeuroSweep

**An autonomous cleaning-robot research simulation built on ROS 2 Humble + Webots.**
NeuroSweep pairs real-time SLAM mapping with a custom, position-aware Bayesian
dirt-estimation layer, so the robot maintains a live probabilistic model of
*where the floor is likely dirty* rather than cleaning blindly on a fixed pattern.

![NeuroSweep live dirt heatmap over the SLAM map in RViz](media/mappingdirt.png)
*Live dirt heatmap: blue where the robot has recently cleaned, red where dirt
probability has accumulated. Rendered over the SLAM occupancy grid in RViz.*

> Status: **Simulation, work-in-progress.** SLAM mapping and the Bayesian dirt
> heatmap are implemented and working. Autonomous navigation (Nav2) is the next
> stage (see [Roadmap](#roadmap)). This is an active learning/research project,
> not a finished product.

---

## What actually works today

| Component | Status | What it does |
|---|---|---|
| Webots + TurtleBot3 Burger sim | Working | Physics-based robot, differential drive, 360° LiDAR (LDS-01) |
| SLAM (slam_toolbox) | Working | Builds a live 2D occupancy-grid map and publishes the robot's pose |
| Bayesian dirt heatmap (`neuro_analytics`) | Working | Position-aware dirt-probability grid, cools where the robot drives |
| Teleoperation | Working | Manual driving via `teleop_twist_keyboard` |
| Autonomous cleaning (Nav2) | **Planned** | Robot picks the dirtiest cell and drives there itself (roadmap) |
| Vision / semantic context (YOLO) | **Planned** | Adjust dirt priors by room type (roadmap) |

I want to be precise about scope: the heatmap is a **first-pass probabilistic
model**. It estimates dirt from *time and location*, not yet from a physical dirt
sensor. Making the update driven by real sensor evidence, and closing the loop with
autonomous navigation, are the next steps I'm working on.

---

## The core idea

A standard robot vacuum cleans on a fixed schedule and a fixed pattern — it has no
notion of *where* dirt is more likely. NeuroSweep treats floor cleanliness as a
**hidden state to be estimated**:

- The floor is divided into a grid (mirroring the SLAM map exactly).
- Each cell holds a **dirt probability** between 0 (clean) and 1 (dirty).
- Cells the robot hasn't visited slowly accumulate dirt probability over time
  (dust settling).
- Cells the robot physically drives over are "cleaned" — their probability decays
  toward zero.

The result is a live **heatmap**: blue where recently cleaned, red where dirt
probability has built up. This is the map a future navigation layer would use to
prioritise the dirtiest areas.

---

## How the Bayesian dirt model works

The dirt grid is sized to match the SLAM occupancy grid (same resolution, size, and
origin), so every dirt cell corresponds to a real physical location in the map.

**Accumulation (unvisited cells get dirtier over time):**

```
P ← P + (P_max − P) · (1 − e^(−λ·Δt))
```

Each tick, a cell's dirt probability rises *toward* a ceiling `P_max`. The rise is
fast when a cell is clean and slows as it approaches "dirty" — a bounded
exponential approach, so probability never overshoots 1. `λ` (lambda) sets how fast
dust accumulates.

**Cleaning (cells under the robot are serviced):**

```
P ← P · e^(−μ·Δt)
```

The node reads the robot's **real position from the TF transform tree**
(`map → base_link`, published by SLAM), converts that position into a grid cell,
and decays the dirt on all cells within the robot's radius. `μ` (mu) sets how
fast cleaning removes dirt.

This is what makes the model *position-aware*: it isn't a timer or random noise —
the cleaning is tied to where the robot actually is, read live from SLAM's pose
estimate. Localisation (where am I → the TF transform) and mapping (what does the
world look like → the occupancy grid) are the two halves of SLAM, and this node
uses both.

The heatmap is published as a `nav_msgs/OccupancyGrid` on `/neuro_sweep/heatmap`,
so it renders directly in RViz.

---

## System architecture

```
Webots (TurtleBot3 + LiDAR)
        │  /scan, wheel odometry
        ▼
   slam_toolbox ──► /map (occupancy grid)
        │          └► TF: map → base_link (robot pose)
        ▼
  neuro_analytics (dirt_inference_node)
        │  reads /map (to size grid) + TF (robot pose)
        │  runs Bayesian accumulate/clean update
        ▼
   /neuro_sweep/heatmap ──► RViz (live dirt heatmap)
```

---

## Running it

Requires ROS 2 Humble, Webots R2025a, `slam_toolbox`, and this workspace built.

```bash
# Terminal 1 — simulation
ros2 launch webots_ros2_turtlebot robot_launch.py

# Terminal 2 — SLAM
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3 — RViz (Fixed Frame = map; add /map, /scan, and /neuro_sweep/heatmap)
source /opt/ros/humble/setup.bash
rviz2

# Terminal 4 — Bayesian dirt heatmap
cd ~/neurosweep_ws
source install/setup.bash
ros2 run neuro_analytics heatmap_node

# Terminal 5 — drive the robot
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

In RViz, set the `/neuro_sweep/heatmap` display to **Color Scheme: costmap** and
**Alpha: 1.0** to see the blue-to-red dirt gradient.

![The ROS 2 node running on Linux](media/codeworkinglinux.png)
*The `dirt_inference_node` running, logging the peak dirt cell and the live
cleaning cell as the robot drives.*

### Tunable parameters

| Parameter | Meaning | Default |
|---|---|---|
| `accumulation_rate` (λ) | How fast unvisited cells get dirty | 0.02 |
| `cleaning_rate` (μ) | How fast the robot cleans a cell | 2.0 |
| `robot_radius_m` | Cleaning footprint radius (m) | 0.35 |
| `tick_period_s` | Update interval (s) | 0.5 |

---

## Roadmap

The next stage closes the perception → action loop:

1. **Autonomous navigation (Nav2).** Take the highest-probability cell from the
   heatmap, send it as a goal via `nav2_simple_commander`, and let Nav2's global
   planner + local controller drive the robot there on a costmap that keeps it
   clear of walls, with recovery behaviours if it gets stuck. The cleaning update
   already fires on arrival, so this becomes a loop that autonomously services the
   dirtiest area first. The main engineering work is costmap tuning and keeping the
   SLAM pose stable enough for reliable navigation.
2. **Evidence-driven dirt updates.** Replace the time-based accumulation with a
   proper Bayesian update conditioned on a (simulated) dirt-detection signal.
3. **Semantic priors (vision).** Use object detection to raise dirt priors in
   high-traffic areas (e.g. under a table).

---

## Tech stack

Ubuntu 22.04 · ROS 2 Humble · Webots R2025a · slam_toolbox · Python · NumPy · tf2

## Author

Faysal Ali Shah — Electrical Engineering, NUST CEME.
Built as a self-directed robotics learning project.
