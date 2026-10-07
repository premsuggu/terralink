#!/usr/bin/env python3
"""
Generates the comprehensive, professional, two-column academic-style project report
for the TerraLink UAV-UGV collaborative navigation system.
Outputs: media/reports/TerraLink_Project_Report.docx
"""
import os
import docx
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.section import WD_SECTION
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

def create_report():
    doc = docx.Document()
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Define Color Palette (Clean Academic / IEEE Style)
    C_NAVY = RGBColor(15, 41, 66)      # Primary Headers #0F2942
    C_STEEL = RGBColor(31, 58, 82)     # Secondary Headers #1F3A52
    C_CHARCOAL = RGBColor(30, 41, 59)  # Body text #1E293B
    C_MUTED = RGBColor(100, 116, 139)  # Subtitles/Metadata #64748B
    C_BLACK = RGBColor(0, 0, 0)

    # Base Page Setup (Letter, 0.75 in margins)
    for sec in doc.sections:
        sec.top_margin = Inches(0.75)
        sec.bottom_margin = Inches(0.75)
        sec.left_margin = Inches(0.75)
        sec.right_margin = Inches(0.75)

    # Configure Normal Style
    style_normal = doc.styles['Normal']
    font_normal = style_normal.font
    font_normal.name = 'Times New Roman'
    font_normal.size = Pt(9.5)
    font_normal.color.rgb = C_CHARCOAL
    style_normal.paragraph_format.line_spacing = 1.12
    style_normal.paragraph_format.space_after = Pt(3.5)
    style_normal.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY

    # Helper: Set XML borders for academic booktabs-style tables
    def apply_academic_table_borders(table):
        tblPr = table._tbl.tblPr
        tblBorders = OxmlElement('w:tblBorders')
        
        top = OxmlElement('w:top')
        top.set(qn('w:val'), 'single')
        top.set(qn('w:sz'), '10')
        top.set(qn('w:color'), '0F2942')
        tblBorders.append(top)
        
        bottom = OxmlElement('w:bottom')
        bottom.set(qn('w:val'), 'single')
        bottom.set(qn('w:sz'), '10')
        bottom.set(qn('w:color'), '0F2942')
        tblBorders.append(bottom)
        
        insideH = OxmlElement('w:insideH')
        insideH.set(qn('w:val'), 'single')
        insideH.set(qn('w:sz'), '4')
        insideH.set(qn('w:color'), 'CBD5E1')
        tblBorders.append(insideH)
        
        for side in ['left', 'right', 'insideV']:
            el = OxmlElement(f'w:{side}')
            el.set(qn('w:val'), 'none')
            tblBorders.append(el)
            
        tblPr.append(tblBorders)

    def set_cell_background(cell, fill_hex):
        tcPr = cell._tc.get_or_add_tcPr()
        shd = OxmlElement('w:shd')
        shd.set(qn('w:val'), 'clear')
        shd.set(qn('w:color'), 'auto')
        shd.set(qn('w:fill'), fill_hex)
        tcPr.append(shd)

    def add_p(text, bold_prefix="", italic_prefix="", space_after=3.5, align=WD_ALIGN_PARAGRAPH.JUSTIFY):
        p = doc.add_paragraph()
        p.alignment = align
        p.paragraph_format.space_after = Pt(space_after)
        p.paragraph_format.line_spacing = 1.12
        if bold_prefix:
            r_b = p.add_run(bold_prefix)
            r_b.font.name = 'Times New Roman'
            r_b.font.bold = True
            r_b.font.size = Pt(9.5)
            r_b.font.color.rgb = C_CHARCOAL
        if italic_prefix:
            r_i = p.add_run(italic_prefix)
            r_i.font.name = 'Times New Roman'
            r_i.font.italic = True
            r_i.font.size = Pt(9.5)
            r_i.font.color.rgb = C_CHARCOAL
        r_t = p.add_run(text)
        r_t.font.name = 'Times New Roman'
        r_t.font.size = Pt(9.5)
        r_t.font.color.rgb = C_CHARCOAL
        return p

    def add_sec_head(title):
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.LEFT
        p.paragraph_format.space_before = Pt(9.0)
        p.paragraph_format.space_after = Pt(3.0)
        p.paragraph_format.keep_with_next = True
        run = p.add_run(title)
        run.font.name = 'Times New Roman'
        run.font.size = Pt(10.5)
        run.font.bold = True
        run.font.color.rgb = C_NAVY
        return p

    def add_subsec_head(title):
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.LEFT
        p.paragraph_format.space_before = Pt(6.0)
        p.paragraph_format.space_after = Pt(2.0)
        p.paragraph_format.keep_with_next = True
        run = p.add_run(title)
        run.font.name = 'Times New Roman'
        run.font.size = Pt(9.5)
        run.font.bold = True
        run.font.italic = True
        run.font.color.rgb = C_STEEL
        return p

    def add_fig_caption(label, caption):
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_before = Pt(2.0)
        p.paragraph_format.space_after = Pt(6.0)
        r_lbl = p.add_run(label + " ")
        r_lbl.font.name = 'Times New Roman'
        r_lbl.font.bold = True
        r_lbl.font.size = Pt(8.5)
        r_lbl.font.color.rgb = C_CHARCOAL
        r_cap = p.add_run(caption)
        r_cap.font.name = 'Times New Roman'
        r_cap.font.italic = True
        r_cap.font.size = Pt(8.5)
        r_cap.font.color.rgb = C_CHARCOAL

    def add_tbl_caption(label, caption):
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_before = Pt(6.0)
        p.paragraph_format.space_after = Pt(2.0)
        r_lbl = p.add_run(label + ": ")
        r_lbl.font.name = 'Times New Roman'
        r_lbl.font.bold = True
        r_lbl.font.size = Pt(8.5)
        r_lbl.font.color.rgb = C_CHARCOAL
        r_cap = p.add_run(caption)
        r_cap.font.name = 'Times New Roman'
        r_cap.font.bold = False
        r_cap.font.size = Pt(8.5)
        r_cap.font.color.rgb = C_CHARCOAL

    # =========================================================================
    # SECTION 1: HEADER BLOCK (Single Column)
    # =========================================================================
    p_title = doc.add_paragraph()
    p_title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p_title.paragraph_format.space_before = Pt(0)
    p_title.paragraph_format.space_after = Pt(4)
    r_title = p_title.add_run("TerraLink: A Collaborative Heterogeneous UAV-UGV Navigation Framework via 2.5D Elevation Mapping and Bounded 3D Volumetric Verification")
    r_title.font.name = 'Times New Roman'
    r_title.font.size = Pt(17.5)
    r_title.font.bold = True
    r_title.font.color.rgb = C_NAVY

    p_meta = doc.add_paragraph()
    p_meta.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p_meta.paragraph_format.space_after = Pt(10)
    r_meta1 = p_meta.add_run("Autonomous Multi-Robot Systems Technical Report\n")
    r_meta1.font.name = 'Times New Roman'
    r_meta1.font.bold = True
    r_meta1.font.size = Pt(10)
    r_meta1.font.color.rgb = C_CHARCOAL
    r_meta2 = p_meta.add_run("TerraLink Core Development & Verification Line — ROS 2 Humble & Ignition Gazebo Fortress\nWorkspace Repository: ")
    r_meta2.font.name = 'Times New Roman'
    r_meta2.font.size = Pt(9)
    r_meta2.font.color.rgb = C_MUTED
    r_meta3 = p_meta.add_run("premsuggu/terralink")
    r_meta3.font.name = 'Times New Roman'
    r_meta3.font.size = Pt(9)
    r_meta3.font.italic = True
    r_meta3.font.color.rgb = C_STEEL

    # Abstract Box
    p_abs = doc.add_paragraph()
    p_abs.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    p_abs.paragraph_format.left_indent = Inches(0.4)
    p_abs.paragraph_format.right_indent = Inches(0.4)
    p_abs.paragraph_format.space_after = Pt(4)
    p_abs.paragraph_format.line_spacing = 1.1
    r_absh = p_abs.add_run("Abstract— ")
    r_absh.font.name = 'Times New Roman'
    r_absh.font.bold = True
    r_absh.font.size = Pt(9.0)
    r_abst = p_abs.add_run(
        "Autonomous navigation across unstructured, occluded environments—such as collapsed structures, "
        "subterranean passages, and dense construction sites—demands heterogeneous robotic cooperation. While unmanned aerial "
        "vehicles (UAVs) provide rapid global overhead surveillance, ground robots (UGVs) execute physical transport and inspection. "
        "However, conventional 2.5D elevation grids fail catastrophically under semi-enclosed structures: overhead depth sensors "
        "capture ceiling or overhang returns, collapsing vertical traversability into lethal obstacle steps and starving ground planners "
        "of viable routes. In this work, we present TerraLink, a complete, from-scratch ROS 2 Humble framework designed for Ignition Gazebo "
        "Fortress. TerraLink couples a high-throughput 2.5D GPU/CPU elevation mapping engine (emap) with an anomaly-guided bounded 3D volumetric "
        "verifier (nav). We introduce a topological connected-component anomaly detector (Version 3) that isolates genuine overhang signatures "
        "from planar partition walls via elevation differentials. When potential occlusions are detected, a hierarchical Probabilistic Roadmap (PRM) "
        "routes a tentative path charged with an anomaly cost penalty (20×). Ground truth vertical clearance is verified on-the-fly through an in-process "
        "octree (BoundedVoxelMap) with radial spatial eviction (12m radius). Verified headroom verdicts are persistently retained in a bidirectional "
        "memory loop (ResolvedRegionStore) to eliminate query oscillations. A deterministic, opt-in A* grid planner with explicit free/blocked/uncertain cell "
        "classes, route memory and a replay-based test workflow was added as a second global planner; in the tunnel world it reached the goal about 16 s after "
        "first movement versus about 28 s for PRM (three runs each, simulation only, tunnel world only here; the maze is covered in Section VII-C). The system has been validated across 426 automated unit tests "
        "(382 nav, 44 emap) and simulation benchmarks in Ignition Fortress. The UGV autonomously negotiates an occluded tunnel corridor with zero operator "
        "intervention. Furthermore, we document the critical physical, simulation, and coordination defects encountered during "
        "development and provide the exact mathematical and algorithmic solutions that ensured end-to-end stability."
    )
    r_abst.font.name = 'Times New Roman'
    r_abst.font.size = Pt(9.0)

    p_kw = doc.add_paragraph()
    p_kw.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    p_kw.paragraph_format.left_indent = Inches(0.4)
    p_kw.paragraph_format.right_indent = Inches(0.4)
    p_kw.paragraph_format.space_after = Pt(12)
    r_kwh = p_kw.add_run("Index Terms— ")
    r_kwh.font.name = 'Times New Roman'
    r_kwh.font.bold = True
    r_kwh.font.size = Pt(9.0)
    r_kwt = p_kw.add_run("Heterogeneous Robotics, UAV-UGV Collaboration, 2.5D Elevation Mapping, OctoMap, PRM and A* Path Planning, Nav2, Anomaly Detection.")
    r_kwt.font.name = 'Times New Roman'
    r_kwt.font.italic = True
    r_kwt.font.size = Pt(9.0)

    # =========================================================================
    # SECTION 2: TWO-COLUMN BODY
    # =========================================================================
    sec2 = doc.add_section(WD_SECTION.CONTINUOUS)
    sec2.top_margin = Inches(0.75)
    sec2.bottom_margin = Inches(0.75)
    sec2.left_margin = Inches(0.75)
    sec2.right_margin = Inches(0.75)

    sectPr2 = sec2._sectPr
    cols = sectPr2.xpath('./w:cols')
    if cols:
        cols[0].set(qn('w:num'), '2')
        cols[0].set(qn('w:space'), '720') # 0.5 in gap
    else:
        col_elem = OxmlElement('w:cols')
        col_elem.set(qn('w:num'), '2')
        col_elem.set(qn('w:space'), '720')
        sectPr2.append(col_elem)

    # --- SECTION I: INTRODUCTION ---
    add_sec_head("I. INTRODUCTION & PROBLEM FORMULATION")
    add_p(
        "Heterogeneous autonomous robotic systems combining aerial and terrestrial platforms provide superior operational "
        "capabilities compared to homogeneous fleets in complex, unstructured scenarios such as post-disaster reconnaissance, "
        "industrial inspection, and civil construction. Unmanned aerial vehicles (UAVs) swiftly traverse three-dimensional space, bypassing "
        "ground obstacles to build broad topological maps. Conversely, unmanned ground vehicles (UGVs) possess substantial payload capacity, "
        "extended operational endurance, and close-proximity manipulation or sensing capability. Harmonizing these complementary assets "
        "requires a robust, shared spatial perception and navigation pipeline."
    )
    add_p(
        "Historically, robotic field mapping has relied extensively on 2.5D elevation grids. Representing spatial terrain as a single "
        "height scalar h(x, y) over a discretized horizontal plane offers remarkable computational efficiency, bounded memory scaling, "
        "and direct applicability to traversability analysis. However, 2.5D elevation models embody an inherent structural flaw: they cannot "
        "represent multi-layered vertical geometry. When a downward-facing UAV sensor scans an overhead overhang, ceiling, bridge, or tunnel "
        "entrance, the sensor rays intersect the topmost opaque surface. The elevation grid records the roof's altitude as the ground elevation. "
        "Subsequent analytical slope and step-height filters score the perimeter of the overhang as an impassable vertical cliff (LETHAL). "
        "Consequently, terrestrial path planners operating over the 2.5D traversability layer treat the entire subterranean or covered passageway "
        "as a solid barrier, permanently abandoning viable transit paths even when adequate ground-level headroom exists."
    )
    add_p(
        "Existing exploration methodologies attempt to circumvent this by tracking unobserved cells through frontier exploration (Yamauchi, 1997). "
        "However, frontier logic operates on the absence of measurements (is_valid = False). In the case of an opaque roof, the elevation map does "
        "not suffer from missing data; rather, it possesses highly confident but misleading measurements of the ceiling structure. Full 3D volumetric "
        "representations (such as global OctoMaps) can resolve multi-level surfaces but incur prohibitive memory footprints and ray-casting latencies "
        "that hinder fast global UGV path planning."
    )
    add_p(
        "To resolve this dilemma, this project introduces the TerraLink framework. The core contributions of this work are:",
        bold_prefix="Contributions: "
    )
    add_p(
        "1) A ground-up, highly optimized 2.5D elevation mapping package (emap) targeting Ignition Gazebo Fortress and ROS 2 Humble, featuring "
        "recursive Bayesian Gaussian fusion, GPU acceleration (CuPy), vertical drift compensation, and analytical traversability scoring.\n"
        "2) A novel Version 3 connected-component anomaly detection algorithm (nav.anomaly) that isolates occluded passage candidates from "
        "ordinary dividing walls by evaluating spatial component area and vertical elevation offsets.\n"
        "3) A hybrid 2.5D/3D navigation paradigm that combines cheap global 2.5D PRM routing with localized, bounded 3D volumetric verification "
        "(nav.voxel_map) executed in-process with automated spatial leaf eviction.\n"
        "4) A bidirectional memory loop (nav.resolved_regions) that caches verified volumetric verdicts to guarantee planner convergence without "
        "query oscillation or redundant inspections.\n"
        "5) Exhaustive empirical verification in simulation, identifying and correcting five subtle physics, timing, and behavioral failure modes.\n"
        "6) An opt-in deterministic A* global planner (nav.astar_planner, nav.global_planner) that treats suspected passages and unobserved space as explicit "
        "uncertain cell classes, with route memory, exploration and stuck feedback, plus an offline map-replay workflow that tests planning without re-running "
        "the UAV scan."
    )

    # --- SECTION II: ARCHITECTURE ---
    add_sec_head("II. SYSTEM ARCHITECTURE & PLATFORM INTEGRATION")
    add_subsec_head("A. Simulation & Middleware Architecture")
    add_p(
        "The TerraLink framework is architected natively for Ignition Gazebo Fortress (version 6.18) paired with ROS 2 Humble running on Ubuntu 22.04 LTS "
        "within a WSL2 environment. Actuation and sensor telemetry are interfaced via ros_gz_bridge. Simulation rendering operates under software "
        "OpenGL rasterization (LIBGL_ALWAYS_SOFTWARE=1) to prevent Mesa D3D12 texture translation aborts during camera image generation."
    )
    
    # Embedded Architecture Figure
    fig1_path = os.path.join(repo_root, 'media/figures/system_architecture_diagram.png')
    if os.path.exists(fig1_path):
        doc.add_picture(fig1_path, width=Inches(3.3))
        add_fig_caption("Fig. 1.", "High-level architectural pipeline of the TerraLink collaborative UAV-UGV framework.")

    add_subsec_head("B. Heterogeneous Robotic Platforms")
    add_p(
        "The system incorporates two distinct simulated agents:\n"
        "• Quadrotor UAV (iris_quad): Powered by Ignition's MulticopterVelocityControl and OdometryPublisher plugins. The UAV carries a rigidly attached "
        "downward-looking RGB-D depth camera (320×240 resolution, 10 Hz, 20m range, 0.08m vertical chassis offset) publishing sensor_msgs/PointCloud2.\n"
        "• Ground UGV (nav_ugv): A custom differential-drive mobile robot. Its sensor suite comprises a 360° planar 2D LiDAR (10 Hz, 12m range), an angled "
        "forward-downward depth camera (pitch 0.6 rad, 8m range, 0.05m chassis elevation) for near-field subterranean inspection, and an elevated "
        "forward video camera (pitch 0.15 rad, 0.22m elevation) dedicated to visual recording and monitoring."
    )
    add_p(
        "Mechanical Redesign: During initial high-deceleration trials (linear acceleration 3.5 m/s²), the single-caster chassis experienced severe dynamic "
        "pitching (19.5° pitch angle), causing the driven wheels to lose ground traction. The DiffDrive plugin integrated free-spinning wheel revolutions "
        "as phantom translation, causing massive odometric divergence. The chassis was mechanically redesigned to feature dual passive casters (front and rear, "
        "r=0.03m, mu=0.001), establishing 4-point ground support and restoring physical stability across all braking maneuvers.",
        bold_prefix="Chassis Stabilization: "
    )

    # --- SECTION III: 2.5D ELEVATION MAPPING ---
    add_sec_head("III. AERIAL PERCEPTION & 2.5D ELEVATION MAPPING")
    add_subsec_head("A. Core Elevation Grid Data Structure")
    add_p(
        "The primary spatial representation in emap is an ElevationMap: a square grid of dimension cell_n × cell_n at resolution δ (default 0.1m). "
        "The map maintains four contiguous floating-point layers stored within a single NumPy tensor of shape (4, cell_n, cell_n):\n"
        "1) Layer 0 (elevation): Estimated terrain height h in meters.\n"
        "2) Layer 1 (variance): Height estimation uncertainty σ² in m².\n"
        "3) Layer 2 (is_valid): Observation indicator (1.0 if observed, else 0.0).\n"
        "4) Layer 3 (traversability): Analytical driving cost score (0.0 to 1.0)."
    )
    add_p(
        "To reconcile fast local reactive maneuvers with long-term global mission planning, elevation_mapping_node instantiates a dual-map architecture: "
        "a rolling local map (20m length) that shifts to remain centered beneath the moving UAV, and a persistent global map (40m length) fixed at world "
        "origin that accumulates observations indefinitely. Map shifting is achieved vectorially via circular buffer rolling (np.roll) along spatial axes (1, 2) "
        "followed by directional edge-blanking (_reset_region), guaranteeing O(1) recentering without memory reallocation."
    )

    add_subsec_head("B. Recursive Bayesian Gaussian Fusion")
    add_p(
        "Incoming point clouds are transformed into the world coordinate frame (iris_quad/odom) via tf2_ros buffer lookups. For each point p = (x, y, z), "
        "the sensor range r = ||p - p_sensor|| is evaluated. Points closer than min_valid_distance (0.2m) or farther than max_valid_range (19.8m) are filtered out. "
        "The measurement uncertainty σ_meas² is modeled as quadratic with respect to distance:"
    )
    add_p("σ_meas² = k_noise · r²", bold_prefix="Noise Model: ", align=WD_ALIGN_PARAGRAPH.CENTER)
    add_p(
        "where k_noise = 0.01. Each cell's state is modeled as a 1D Gaussian belief N(h_prior, σ_prior²). Outliers are detected via a Mahalanobis distance check: "
        "if |h_prior - z| > 2.0 · σ_prior², the point is rejected from elevation fusion, and cell uncertainty is penalized via scatter-addition (np.add.at) "
        "by σ_outlier² (1.0 m²). For surviving inliers, the prior belief is merged with the measurement via Bayesian variance-weighted update equations:"
    )
    add_p("h_new = (h_prior · σ_meas² + z · σ_prior²) / (σ_prior² + σ_meas²)", align=WD_ALIGN_PARAGRAPH.CENTER)
    add_p("σ_new² = (σ_prior² · σ_meas²) / (σ_prior² + σ_meas²)", align=WD_ALIGN_PARAGRAPH.CENTER)
    add_p(
        "Where multiple inlier rays strike the identical cell within a single point cloud frame, scatter-addition accumulation computes the exact multi-ray "
        "mean, preventing last-write-wins race corruption. A GPU-accelerated CuPy kernel (fusion_gpu.py) implements the identical arithmetic pipeline, "
        "achieving a 25% throughput gain on large point clouds."
    )

    add_subsec_head("C. Analytical Traversability Classification")
    add_p(
        "Departing from opaque, learned neural networks (such as Direction 1's CNN filter), emap implements an analytical, explainable traversability engine. "
        "Terrain drivability is derived from three physical metrics:\n"
        "1) Surface Slope: Computed via central finite differences:\n"
        "   S = sqrt((∂h/∂x)² + (∂h/∂y)²)\n"
        "2) Local Step Discontinuity: Computed using 3×3 morphologic filters:\n"
        "   Δh_step = max_3×3(h) - min_3×3(h)\n"
        "3) Surface Roughness: Quantified directly via height variance σ²."
    )
    add_p(
        "Cells are classified into discrete operational tiers: LETHAL (0.0) if S > 0.35, Δh_step > 0.15m, or σ² > 0.05 m²; DIFFICULT (0.3) if exceeding "
        "half-threshold values; and EASY (1.0) otherwise. Unobserved cells (is_valid = 0) default safely to EASY to prevent unmapped terrain from generating "
        "artificial boundary repulsion."
    )

    fig2_path = os.path.join(repo_root, 'media/figures/elevation_map_render.png')
    if os.path.exists(fig2_path):
        doc.add_picture(fig2_path, width=Inches(3.3))
        add_fig_caption("Fig. 2.", "Rendered 2.5D elevation grid with analytical traversability coloring and 3D surface projection.")

    add_subsec_head("D. Odometry Drift Compensation & Safety Watchdog")
    add_p(
        "In physical deployments, barometric and IMU vertical odometry drifts over time. emap incorporates an autonomous Z-drift compensator (drift.py). "
        "Residuals between newly transformed points and high-confidence global map cells (σ² < 0.05) are aggregated. The median residual across inliers is extracted, "
        "and the estimated vertical bias is updated via a damped gain controller (gain α = 0.3, residual clamp 1.0m):\n"
        "Z_bias ← Z_bias + α · median(z_meas - h_map)\n"
        "Crucially, incoming points are corrected against existing bias prior to computing residuals, establishing an asymptotically stable fixed-point convergence."
    )
    add_p(
        "Additionally, because Ignition's MulticopterVelocityControl plugin lacks an internal command timeout (perpetually executing the last received Twist), "
        "a dedicated safety node (cmd_vel_watchdog.py) monitors publisher liveness. If operator commands cease for >1.0s, the watchdog injects a zero-velocity "
        "Twist, preventing quadrotor runaway climb."
    )

    # --- SECTION IV: TOPOLOGICAL ANOMALY DETECTION ---
    add_sec_head("IV. SPATIAL MASKS & ANOMALY DETECTION")
    add_subsec_head("A. Spatial Mask Formulations")
    add_p(
        "Downstream navigation nodes in the nav package ingest the published /elevation_map GridMap to derive binary operational masks:\n"
        "• Walkable Mask (Mw): Confirmed traversable territory:\n"
        "  Mw(x, y) = is_valid(x, y) ∧ (traversability(x, y) ≠ LETHAL)\n"
        "• Frontier Mask (Mf): Unobserved space bordering known walkable ground (Yamauchi boundary exploration):\n"
        "  Mf(x, y) = ¬is_valid(x, y) ∧ (∃ q ∈ N4(x, y) : Mw(q))"
    )

    add_subsec_head("B. The Ceiling Occlusion Dilemma")
    add_p(
        "When an overhead UAV surveys a roofed structure (e.g., a culvert or arched tunnel), the camera observes the roof's upper surface. "
        "Because measurements exist, is_valid = True. Because the roof elevation stands ~0.7m above the room floor, traversability scores the step discontinuity "
        "as LETHAL. Thus, Mw = False and Mf = False. The tunnel entrance is categorized as an impassable obstacle indistinguishable from a solid concrete wall."
    )

    add_subsec_head("C. Version 3 Anomaly Detection Algorithm")
    add_p(
        "To overcome this structural failure without compromising obstacle detection, we designed an anomaly detector (nav.anomaly). Its design evolved "
        "across three iterative generations based on empirical simulation findings:\n"
        "• Version 1 (Local Scanlines): Evaluated 1D step-edges along coordinate axes. Produced high false-positive rates due to depth discretization noise.\n"
        "• Version 2 (Topological Gap Analysis): Utilized connected-component labeling and KD-tree nearest-gap analysis. While robust against sensor noise, "
        "it failed on ordinary thin partition walls separating two rooms, flagging 8 spurious corridor cells across a solid dividing wall.\n"
        "• Version 3 (Component Topology with Elevation Differentials): Restores structural discernment by enforcing a tri-partite role model. Walkable components "
        "are labeled via 8-connectivity and partitioned by surface area:\n"
        "  - Rooms (A ≥ 3.0 m²): Genuine large navigable regions.\n"
        "  - Islands (0.15 ≤ A < 3.0 m²): Moderate elevated surfaces (e.g., tunnel roofs).\n"
        "  - Noise (A < 0.15 m²): Small sensor artifacts or vehicle footprints."
    )
    add_p(
        "For each island, the algorithm evaluates its boundary proximity against candidate rooms using KD-trees. An anomaly corridor is confirmed if and only if:\n"
        "1) The island connects to at least two distinct rooms.\n"
        "2) The connecting straight-line segment is thinly lethal (width ≤ 0.6m).\n"
        "3) The mean elevation of the island differs significantly from both rooms: |h_island - h_room| ≥ 0.15m.\n"
        "Because an ordinary room-dividing wall possesses no intermediate island component, it is structurally barred from being flagged, eliminating false-positive "
        "corridors while reliably identifying true occluded passages."
    )

    # --- SECTION V: 3D VOLUMETRIC VERIFICATION ---
    add_sec_head("V. BOUNDED 3D VOLUMETRIC VERIFICATION")
    add_subsec_head("A. Local 3D Volumetric OctoMap")
    add_p(
        "While Version 3 anomaly detection identifies potential corridors, 2.5D data cannot confirm whether vertical clearance exists beneath the roof. "
        "TerraLink introduces an in-process 3D occupancy map (nav.voxel_map.BoundedVoxelMap) backed by the octomap C++ binding. The voxel map fuses incoming "
        "point clouds from both the UAV and the UGV's front-facing depth camera via ray-casting (insertPointCloud), explicitly marking traversed interior volume "
        "as confirmed free space."
    )
    add_p(
        "Memory Bounding: Standard global octrees suffer unbounded growth. BoundedVoxelMap enforces a strict spatial cutoff (max_radius_m = 12.0m) centered on the UGV's "
        "current pose. A periodic background thread scans octree leaves and executes radial eviction (evict_outside_radius). During development, an insidious "
        "upstream binding defect was identified: invoking deleteNode(coord) with default parameters (depth=1) purged major subtrees, collapsing a 1478-leaf tree "
        "to 53 leaves. Explicitly enforcing depth=0 restored surgical single-leaf eviction, ensuring bounded memory with zero data loss.",
        bold_prefix="Spatial Eviction Safeguard: "
    )

    fig3_path = os.path.join(repo_root, 'media/figures/voxel_map_render.png')
    if os.path.exists(fig3_path):
        doc.add_picture(fig3_path, width=Inches(3.3))
        add_fig_caption("Fig. 3.", "Volumetric 3D occupancy reconstruction (BoundedVoxelMap) isolating tunnel arch, roof, and navigable floor clearance.")

    add_subsec_head("B. Volumetric Headroom Scanning")
    add_p(
        "Before the UGV commits to an anomaly segment, waypoint_follower queries voxel_map_node over a verification topic. The function check_region_headroom "
        "discretizes the candidate path segment and scans vertical ray columns from z_min (-0.3m) to z_max (1.5m) at step_m (0.05m):\n"
        "• passable: A contiguous run of free voxels exceeding the required clearance (h_robot + margin = 0.25m + 0.15m = 0.40m) is verified.\n"
        "• blocked: No valid clearance exists, and all vertical samples are confidently resolved (occupied or free).\n"
        "• unknown: Insufficient clearance is found, but at least one voxel remains unobserved."
    )

    add_subsec_head("C. Bidirectional Memory Loop (ResolvedRegionStore)")
    add_p(
        "To prevent repetitive sensor checks and query churn, verified verdicts are dispatched to planner_node over resolved_region_report. The module "
        "nav.resolved_regions.ResolvedRegionStore records settled bounding boxes. Upon receiving subsequent /elevation_map messages, planner_node re-applies "
        "these verdicts: PASSABLE regions force Mw = True and Ma = False, permanently converting the corridor into standard drivable terrain, whereas BLOCKED "
        "regions force Mw = False and trigger an immediate global replan."
    )

    # --- SECTION VI: PRM & EXECUTION ---
    add_sec_head("VI. GLOBAL PLANNING (PRM AND A*) & MOTION CONTROL")
    add_subsec_head("A. Hierarchical PRM Planner")
    add_p(
        "Global path planning is performed on demand by nav.prm_planner.plan. Standard PRM implementations fail in closed-loop navigation when the robot's own chassis "
        "is fused into the elevation map, falsely marking the start cell as lethal. TerraLink resolves this by clearing a footprint disk (radius 0.3m) around the "
        "start pose prior to sampling."
    )
    add_p(
        "The planner samples N = 400 random nodes across Mw. Edge connectivity is evaluated across candidate pairs within connect_radius_m (3.0m) using Bresenham "
        "line-of-sight rasterization over the composite traverse mask (Mw ∪ Mf ∪ Ma). Edges traversing anomaly cells are charged an exponential cost multiplier (20×). "
        "Dijkstra's algorithm extracts the optimal path: confirmed routes are strictly preferred, but anomaly corridors are returned as tentative last resorts "
        "when no alternative exists."
    )

    add_subsec_head("B. Deterministic A* Planner (opt-in, planner_type:=astar)")
    add_p(
        "Random sampling can miss a 1 m tunnel and gives different routes for nearly identical maps. The A* planner (nav.astar_planner) searches an 8-connected "
        "cost grid with an octile heuristic, no diagonal corner cutting and a deterministic tie-break, so equal maps give equal paths and a path is found "
        "whenever one exists at grid resolution. Obstacles are inflated by robot radius plus margin, minus a 0.1 m rim already contained in emap's lethal "
        "mask. Cells are free, blocked or uncertain (suspected passage from nav.anomaly, or unobserved space next to free ground). Search runs in passes so "
        "a confirmed route always beats a guess: strict (free cells only), then optimistic (uncertain cells added with a penalty), each at full then relaxed "
        "inflation. Paths are shortened by supercover line of sight, cost-aware on free ground, and a standoff waypoint is placed 1 m before an uncertain "
        "stretch where the UGV should stop and look."
    )
    add_p(
        "A shared decision layer (nav.global_planner) adds route memory (a stored route is kept unless it became invalid, a confirmed route replaced a tentative "
        "one, or a new route is at least 15% cheaper), exploration toward suspected passages and then frontier viewpoints, an anomaly hold against detector "
        "flicker, a wait-for-mapping gate, and stuck feedback that penalizes the route stretch ahead of a stalled robot. All of these are off by default "
        "except route memory. The same class is used by the ROS node and by the offline replay tool, so offline results reflect the live planner.",
        bold_prefix="Decisions around the planner: "
    )
    add_p(
        "A real finding: a 0.15 m wall is only one or two 0.1 m cells wide, so part of it is unobserved rather than lethal and appears as a thin frontier line. "
        "Both planners treated it as a possible passage and repeatedly drove at the wall. Dropping unobserved strips thinner than the robot diameter "
        "(frontier_min_width_m = 0.44 m, automatic for A*) removed this; without it A* reached the goal in 0 of 3 runs. A second finding concerned clearance: the "
        "live detector flags only about 3x3 cells at each tunnel mouth, narrower than the robot, so demanding full clearance there returned no path where PRM's "
        "zero-width line of sight succeeded; uncertain cells are therefore exempt from the clearance test in the optimistic pass (uncertain_waive_m = 0.5 m).",
        bold_prefix="Lessons from real maps: "
    )

    add_subsec_head("C. Motion Execution & Preemption Race Prevention")
    add_p(
        "The node waypoint_follower receives PRM paths and interfaces with Nav2's bt_navigator via sequential /goal_pose dispatches. During live trials, a severe "
        "behavior-tree race condition was identified: because waypoint 0 corresponds to the UGV's current position, publishing waypoint 0 followed immediately "
        "by waypoint 1 caused Nav2's controller_server to report success on waypoint 0 within 19ms, preempting and aborting navigation toward waypoint 1. "
        "This left the UGV stranded in an idle state. The helper _skip_reached_leading_waypoints was introduced, actively filtering all leading waypoints within "
        "reached_radius (0.4m) so only genuinely unreached targets are ever published."
    )
    add_p(
        "Supervisory Watchdog: To handle Nav2 controller exhaustion near physical walls ('Controller patience exceeded'), waypoint_follower incorporates a progress "
        "watchdog (_made_progress). If spatial displacement Δd < 0.15m over 20s, stuck recovery resets goal tracking and requests an immediate fresh plan. "
        "With a deterministic planner the fresh plan can be identical, which the follower used to treat as no change, so a goal dropped while Nav2 was still "
        "starting was never resent; stuck recovery now resends the current goal when the plan is unchanged.",
        bold_prefix="Stuck Recovery: "
    )

    # --- SECTION VII: EXPERIMENTAL RESULTS ---
    add_sec_head("VII. EXPERIMENTAL RESULTS & RUNTIME ANALYSIS")
    add_subsec_head("A. Quantitative Performance Benchmarks")
    add_p(
        "The TerraLink framework was rigorously evaluated in Ignition Gazebo Fortress across two primary environments: room_maze.world (standard open/partitioned maze) "
        "and tunnel_test.world (two 6×4m rooms divided by a wall and connected exclusively by a 1.5m arched tunnel). Table I summarizes the framework's operational "
        "parameters, and Table II details benchmark metrics before and after key optimizations."
    )

    # TABLE I: Parameters
    add_tbl_caption("TABLE I", "TERRALINK SYSTEM SPECIFICATIONS & OPERATIONAL PARAMETERS")
    tbl1 = doc.add_table(rows=1, cols=3)
    tbl1.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr1 = tbl1.rows[0].cells
    hdr1[0].text = "Subsystem"
    hdr1[1].text = "Parameter Name"
    hdr1[2].text = "Operational Value"
    for c in hdr1:
        set_cell_background(c, "F1F5F9")
        c.paragraphs[0].runs[0].font.bold = True
        c.paragraphs[0].runs[0].font.size = Pt(8.5)
        c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    
    params_data = [
        ("emap 2.5D Grid", "Resolution (δ) / Grid Dimensions", "0.10 m / 400 × 400 cells (Global)"),
        ("emap 2.5D Grid", "Update Rate / Default Far Clip", "10 Hz / 19.8 m (Thresholded)"),
        ("Traversability", "Max Slope / Step / Roughness", "0.35 rad / 0.15 m / 0.05 m²"),
        ("nav.anomaly (v3)", "Room Min Area / Island Range", "3.0 m² / [0.15, 3.0) m²"),
        ("nav.anomaly (v3)", "Max Gap / Lethal Band Width", "1.5 m / 0.6 m (Bresenham)"),
        ("nav.anomaly (v3)", "Min Elevation Differential", "0.15 m (Island vs Room)"),
        ("nav.voxel_map", "Voxel Resolution / Eviction Radius", "0.05 m / 12.0 m (Radial Sphere)"),
        ("nav.voxel_map", "Required Headroom Clearance", "0.40 m (0.25m chassis + 0.15m margin)"),
        ("nav.prm_planner", "Node Samples (N) / Connect Radius", "400 nodes / 3.0 m"),
        ("nav.prm_planner", "Anomaly Edge Penalty Factor", "20.0× Distance Weight"),
        ("nav.astar_planner", "Robot radius / Margin / Mask rim", "0.22 m / 0.05 m / 0.10 m"),
        ("nav.astar_planner", "Standoff / Uncertain waiver", "1.0 m / 0.5 m (optimistic pass)"),
        ("nav.global_planner", "Plan switch margin / Stuck window", "15% cost / 18 s"),
        ("nav.walkability", "Frontier min width (A* auto)", "0.44 m (robot diameter)"),
        ("UGV Actuation", "Cruise Speed / Accel Limit", "0.60 m/s / 3.50 m/s² (DiffDrive)"),
        ("Safety Watchdog", "cmd_vel Timeout / Stuck Radius", "1.0 s silence / 0.15 m over 20 s"),
    ]
    for sub, pnm, val in params_data:
        row = tbl1.add_row().cells
        row[0].text = sub
        row[1].text = pnm
        row[2].text = val
        for i, c in enumerate(row):
            c.paragraphs[0].runs[0].font.size = Pt(8.0)
            if i == 2:
                c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    apply_academic_table_borders(tbl1)

    # TABLE II: Benchmarks
    add_tbl_caption("TABLE II", "COMPUTATIONAL PROFILING & OPTIMIZATION IMPACT")
    tbl2 = doc.add_table(rows=1, cols=4)
    tbl2.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr2 = tbl2.rows[0].cells
    hdr2[0].text = "Metric / Component"
    hdr2[1].text = "Baseline (Pre-Fix)"
    hdr2[2].text = "Optimized (Final)"
    hdr2[3].text = "Performance Delta"
    for c in hdr2:
        set_cell_background(c, "F1F5F9")
        c.paragraphs[0].runs[0].font.bold = True
        c.paragraphs[0].runs[0].font.size = Pt(8.5)
        c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER

    bench_data = [
        ("emap CPU Utilization", "220% CPU (Full cloud)", "40% CPU (stride=4)", "-81.8% CPU Load"),
        ("voxel_map CPU Load", "200% CPU (All rays)", "23% CPU (stride=4)", "-88.5% CPU Load"),
        ("System Load Average", "22.4 (Physics stall)", "9.2 - 11.8 (Stable)", "Physics preserved"),
        ("Launch to First Plan", "90 - 100 s (Corner dwell)", "33 - 39 s (Reordered)", "2.6× faster start"),
        ("Total Mission Duration", "214 s (Wandering/stall)", "133 - 153 s (Direct)", "33% transit reduction"),
        ("Plan Query Latency", "460+ s (Deadlock fail)", "14.8 s (Footprint clear)", "Immediate resolution"),
        ("OctoMap Eviction Latency", "Subtree collapse (d=1)", "0.08 s / cycle (d=0)", "Lossless bounding"),
    ]
    for mtr, bsl, opt, dlt in bench_data:
        row = tbl2.add_row().cells
        row[0].text = mtr
        row[1].text = bsl
        row[2].text = opt
        row[3].text = dlt
        for i, c in enumerate(row):
            c.paragraphs[0].runs[0].font.size = Pt(8.0)
            if i in [1, 2, 3]:
                c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    apply_academic_table_borders(tbl2)

    add_subsec_head("B. End-to-End Tunnel Transit Validation")
    add_p(
        "The complete autonomous pipeline was validated across three successive live runs in tunnel_test.world under full software rendering. "
        "The UAV executed an autonomous takeoff, dwelling sequentially at Room A center (-3.0, 0.0), directly over the tunnel (0.0, 0.0), and Room B center (3.0, 0.0). "
        "Within 35 seconds, emap assembled a valid 2.5D global elevation grid. Version 3 anomaly detection flagged the two tunnel mouths at x = ±0.75m. "
        "planner_node produced an anomaly-tentative path, waypoint_follower requested 3D volumetric clearance, and voxel_map_node confirmed PASSABLE headroom (0.7m clearance). "
        "The UGV traversed through the occluded tunnel, arriving within 0.10m of the red goal marker with continuous forward velocity."
    )

    fig4_path = os.path.join(repo_root, 'media/figures/experimental_sequence.png')
    if os.path.exists(fig4_path):
        doc.add_picture(fig4_path, width=Inches(3.3))
        add_fig_caption("Fig. 4.", "Experimental sequence of the autonomous tunnel traversal: (a) Start at green marker, (b) Mid-tunnel transit beneath ceiling occlusion, (c) Convergence at red goal marker, and (d) Onboard video camera perspective.")

    add_subsec_head("C. Planner Comparison: PRM versus A*")
    add_p(
        "Both planners were run in tunnel_test.world with goal (3, 0), a 6.0 m straight-line start-to-goal distance, headless, using the ground-truth "
        "odometry topic. Move-to-arrive is the time from first movement to within 0.3 m of the goal; stalled time counts 5 s windows with under 0.1 m net "
        "movement. In start-at-planning mode the UAV scan is skipped and the saved finished map is replayed, which isolates planning and following from "
        "scan timing (three runs per configuration). Table III summarises it; all PRM numbers include the flag and goal-resend fixes described above."
    )
    add_tbl_caption("TABLE III", "PRM VERSUS A* IN THE TUNNEL WORLD (SIMULATION, ONE WORLD, ONE GOAL)")
    tbl3 = doc.add_table(rows=1, cols=5)
    tbl3.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr3 = tbl3.rows[0].cells
    for c, t in zip(hdr3, ["Configuration", "Runs", "Reached", "Move-to-arrive", "Driven / stalled"]):
        c.text = t
        set_cell_background(c, "F1F5F9")
        c.paragraphs[0].runs[0].font.bold = True
        c.paragraphs[0].runs[0].font.size = Pt(8.5)
        c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    cmp_data = [
        ("Clean map: PRM", "3", "3/3", "28.0 s mean", "6.5 m / 5 s"),
        ("Clean map: A*, no thin-strip pruning", "3", "0/3", "-", "3.2-3.9 m / 115-130 s"),
        ("Clean map: A* (defaults)", "3", "3/3", "15.8 s mean", "5.8 m / 0 s"),
        ("Clean map: A* + exploration + stuck", "3", "3/3", "17.5 s mean", "5.8-6.5 m / 0 s"),
        ("Full run: PRM", "1", "yes", "112.5 s", "17.3 m / 45 s"),
        ("Full run: A* (defaults)", "2", "1/2", "74.2 s (one run stalled 0.306 m short)", "16.3 m / 0 s"),
        ("Full run: A* + hold 30 s + wait-for-mapping", "2", "2/2", "16.3 s and 28.8 s", "5.8-6.9 m / 0 s"),
    ]
    for row_vals in cmp_data:
        row = tbl3.add_row().cells
        for i, (c, v) in enumerate(zip(row, row_vals)):
            c.text = v
            c.paragraphs[0].runs[0].font.size = Pt(8.0)
            if i > 0:
                c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    apply_academic_table_borders(tbl3)
    add_p(
        "In the clean three-run comparison A* with pruning reached the goal about 44% sooner after first movement and drove about 11% less than PRM, and the "
        "run-to-run spread was small. In full mode, A* with the wait-for-mapping gate matches the clean behaviour, and total arrival time (97-110 s) was no "
        "worse than PRM's (98-147 s), but those rows have only one or two runs each. Exploration and stuck feedback showed no benefit on this static map. "
        "Not demonstrated: other worlds or goals, robustness of the 0.44 m pruning width to other wall thicknesses, and the goal-docking edge (one run stopped "
        "6 mm outside the arrival threshold). The default planner therefore remains PRM.",
        bold_prefix="Reading: "
    )
    add_p(
        "Step 08 added path-trace images and a second world. With the same logging (true UGV pose from the simulator and every plan the planner returned), "
        "three full runs per planner were recorded in tunnel_test.world and in room_maze.world (12 runs, UAV scan included). In the maze A* followed the same route "
        "in all three runs (17-18 s from first movement, 6.3-6.4 m, no stalls) while PRM reached the goal in two of three (one run stalled for 81 s; in the third "
        "its only plan crossed a wall and the UGV circled in front of it for about five minutes). In the tunnel PRM reached the goal in all three runs but slowly "
        "(63-242 s from first movement, 29-192 s stalled), while A* arrived in two of three runs in 17-31 s with no stalls and failed once, stuck at a thin wall seam "
        "south of the tunnel. The images are in media/figures/path_comparison/. Three runs per cell in simulation show behaviour, not statistics. The same work "
        "uncovered and fixed three defects: a launch argument that crashed the planner node (an integer where a double was expected), and a maze world and launch file "
        "that had not worked since the ground-truth odometry change (a missing transform source, then the spawn offset applied twice).",
        bold_prefix="Path traces in two worlds: "
    )
    add_p(
        "Measured on the saved 160x160 tunnel map (0.1 m cells, single CPU thread), one A* plan takes about 64 ms, mask construction 3.5 ms, and one PRM plan "
        "about 1.05 s. Three grid searches per plan account for about two thirds of the A* time. Mapping, voxel-map and Nav2 costs were not re-profiled for "
        "this table.",
        bold_prefix="Planner compute: "
    )

    add_subsec_head("D. Test Suite Verification")
    add_p(
        "To enforce architectural integrity without simulator overhead, the repository contains 25 test files spanning tests/emap and tests/nav "
        "(20 nav, 6 emap). The 382 nav tests run in under a minute and the 44 CPU emap tests in under 1 s (the GPU fusion test is skipped and the emap folder "
        "must be run file by file in this environment because a cupy import fails against the installed numpy). They cover coordinate math, Bayesian fusion "
        "invariants, Mahalanobis rejection, Bresenham and supercover rasterization, OctoMap eviction, Dijkstra cost weights, A* optimality and completeness "
        "cases, route memory, exploration, stuck feedback, and regression tests on real recorded maps, all without a simulator."
    )

    # --- SECTION VIII: PRACTICAL LESSONS ---
    add_sec_head("VIII. PRACTICAL LESSONS & ENGINEERING INSIGHTS")
    add_p(
        "Developing heterogeneous multi-robot systems within physics simulation frequently exposes edge-case behaviors where software abstractions "
        "diverge from physical reality. Below, we document the critical defects uncovered and resolved during this project:",
        bold_prefix="Defect Analysis: "
    )
    add_p(
        "1) Depth Camera Far-Clip Clamp Artifact: When sensor rays encounter no geometry, Ignition's depth camera produces +inf for most pixels but clamps "
        "a small fraction at the far-clip boundary (19.94m vs 20.0m). When transformed, these finite points generated artificial terrain towers reaching "
        "into the sky. Resolved by introducing max_valid_range = 19.8m in fusion.py.\n"
        "2) Phantom Geometry from Scene Lighting: Ignition's directional lights placed inside the operational bounding box were mistakenly rasterized as "
        "solid geometry by the Mesa software renderer. Relocating lights to z = 500m eliminated artificial ceiling hits.\n"
        "3) Differential Drive Odometry Divergence: Ignition's DiffDrive plugin publishes dead-reckoning odometry integrated from spawn. During wheel slippage, "
        "odom reported the UGV was at the goal while physics ground truth showed it motionless in Room A. Sourcing TF directly from OdometryPublisher restored "
        "absolute ground truth synchronization.\n"
        "4) In-Process OctoMap Subtree Purging: The PyPI octomap binding defaults deleteNode(coord) to depth=1, deleting major branch nodes and wiping out "
        "accumulated map memory. Enforcing depth=0 resolved the defect.\n"
        "5) Nav2 Velocity Smoother Topic Collision: Installed nav2_bringup remaps smoothed velocity directly to unnamespaced /cmd_vel, colliding with the UAV's "
        "flight controller. Implementing a targeted SetRemap inside launch files isolated ground and aerial twist channels.\n"
        "6) Failed Plans Overwrote Tentative-Plan Flags: the planner node published plan_has_frontier/anomaly = False for a request that found no path; the "
        "follower read that as 'plan is now confirmed' and stopped periodic replanning, a stall of about 45 s. Flags are now published only for valid plans.\n"
        "7) Thin Walls Are Unobserved, Not Lethal: a 0.15 m wall is one or two cells wide, so unscanned wall tops look like passages. Fixed with a local "
        "inscribed-disc width test on frontier cells.\n"
        "8) Diagnosing from the wrong evidence: an early conclusion that failed A* runs were not a planner problem was wrong; replaying PRM and A* on the same "
        "late snapshots showed A* returning no path where PRM crossed. Offline replay on recorded maps, with regression tests, is now the first diagnostic step."
    )

    # --- SECTION IX: CONCLUSION ---
    add_sec_head("IX. CONCLUSION & FUTURE WORK")
    add_p(
        "We have presented TerraLink, an open, modular ROS 2 Humble framework for heterogeneous UAV-UGV collaborative navigation in unstructured and "
        "occluded environments. By decoupling global 2.5D elevation planning from local bounded 3D volumetric verification, TerraLink resolves the classical "
        "overhang occlusion failure mode without incurring the computational or memory penalties of global 3D grids. Version 3 anomaly detection and "
        "bidirectional region caching guarantee efficient, oscillation-free transit through covered corridors. An opt-in deterministic A* planner with explicit uncertain-cell classes, route memory and an offline replay workflow "
        "reduced time and distance from first movement in the tunnel world, but all evidence is simulation, one main world and a small number of runs. Compared "
        "with published systems we are at best on par at the concept level for UAV-assisted ground navigation and clearly behind on exploration (TARE, FUEL, "
        "GBPlanner), on path execution (Nav2 Regulated Pure Pursuit instead of per-waypoint DWB goals), and on real-robot validation. Planned work: validate "
        "A* on the maze and construction-site maps, adopt continuous path following, fix the anomaly detector at the source, profile and reduce compute "
        "(replan only on change, lazy search passes, an adaptive UAV scan), and then integrate Direction 2 semantic vision "
        "(YOLOv8-seg/SAM 2.1) and deploy the framework onto physical robots."
    )

    # --- REFERENCES ---
    add_sec_head("REFERENCES")
    refs = [
        "[1] A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard, 'OctoMap: An efficient probabilistic 3D mapping framework based on octrees,' Autonomous Robots, vol. 34, no. 3, pp. 189–206, 2013.",
        "[2] P. Fankhauser, M. Bloesch, and M. Hutter, 'Probabilistic terrain mapping for mobile robots with uncertain localization,' IEEE Robotics and Automation Letters, vol. 3, no. 4, pp. 3019–3026, 2018.",
        "[3] B. Yamauchi, 'A frontier-based approach for autonomous exploration,' in Proc. IEEE Int. Symp. Computational Intelligence in Robotics and Automation (CIRA), 1997, pp. 146–151.",
        "[4] S. Macenski, F. Martín, R. White, and J. Clavero, 'The Marathon 2: A navigation system,' in Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS), 2020, pp. 2718–2725.",
        "[5] L. E. Kavraki, P. Svestka, J.-C. Latombe, and M. H. Overmars, 'Probabilistic roadmaps for path planning in high-dimensional configuration spaces,' IEEE Trans. Robot. Autom., vol. 12, no. 4, pp. 566–580, 1996.",
        "[6] E. Marder-Eppstein et al., 'The Office Marathon: Robust navigation in an office environment,' in Proc. IEEE Int. Conf. Robotics and Automation (ICRA), 2010, pp. 300–307.",
        "[7] N. Koenig and A. Howard, 'Design and use paradigms for Gazebo, an open-source multi-robot simulator,' in Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS), 2004, pp. 2149–2154.",
        "[8] J. Redmon and A. Farhadi, 'YOLOv3: An incremental improvement,' arXiv preprint arXiv:1804.02767, 2018.",
        "[9] A. Kirillov et al., 'Segment Anything,' in Proc. IEEE/CVF Int. Conf. Computer Vision (ICCV), 2023, pp. 4015–4026.",
        "[10] T. M. Howard, C. J. Green, and A. Kelly, 'State space sampling of feasible motions for high-performance mobile robot navigation in complex terrain,' J. Field Robot., vol. 25, no. 6-7, pp. 325–345, 2008.",
        "[11] P. E. Hart, N. J. Nilsson, and B. Raphael, 'A formal basis for the heuristic determination of minimum cost paths,' IEEE Trans. Syst. Sci. Cybern., vol. 4, no. 2, pp. 100–107, 1968.",
        "[12] T. Miki, L. Wellhausen, R. Grandia, F. Jenelten, T. Homberger, and M. Hutter, 'Elevation mapping for locomotion and navigation using GPU,' arXiv:2204.12876, 2022.",
        "[13] S. Macenski, S. Singh, F. Martín, and J. Ginés, 'Regulated pure pursuit for robot path tracking,' arXiv:2305.20026, 2023.",
        "[14] C. Cao, H. Zhu, H. Choset, and J. Zhang, 'TARE: A hierarchical framework for efficiently exploring complex 3D environments,' in Robotics: Science and Systems (RSS), 2021.",
        "[15] B. Zhou, Y. Zhang, X. Chen, and S. Shen, 'FUEL: Fast UAV exploration using incremental frontier structure and hierarchical planning,' arXiv:2010.11561, 2020."
    ]
    for r in refs:
        p_ref = doc.add_paragraph()
        p_ref.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
        p_ref.paragraph_format.left_indent = Inches(0.2)
        p_ref.paragraph_format.first_line_indent = Inches(-0.2)
        p_ref.paragraph_format.space_after = Pt(2.0)
        p_ref.paragraph_format.line_spacing = 1.05
        r_run = p_ref.add_run(r)
        r_run.font.name = 'Times New Roman'
        r_run.font.size = Pt(8.0)
        r_run.font.color.rgb = C_CHARCOAL

    output_dir = os.path.join(repo_root, "media", "reports")
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "TerraLink_Project_Report.docx")
    doc.save(output_path)
    print(f"Report successfully generated at: {output_path}")

if __name__ == '__main__':
    create_report()
