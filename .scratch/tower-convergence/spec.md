# Specification: Two-stage Tower Geometric Convergence (UpperTowerBox & LowerTowerFrustum)

## Problem Statement

During UAV LiDAR point cloud processing of high-voltage transmission lines, electric power transmission towers exhibit vastly different spatial structural profiles across different elevation layers. The upper tower head features wide horizontal crossarms and long drooping tension jumpers, where the physical span and insulator string lengths vary drastically across voltage levels (from 1.2m at 110kV up to 4.5m+ at 500kV and 8m+ at UHV). In contrast, the lower tower trunk transitions immediately into a bare four-legged lattice column, which then expands linearly towards the ground foundations.

Existing convergence logic suffers from two critical failure modes:
1. **Upper-Lower Step Mismatch & Lower Crossarm Diagonal Severing**: If the division line between upper and lower structures is positioned even slightly too high, the lower diagonal bracing angle steels of the lowest crossarms fall into the narrow waist envelope and are erroneously sheared off. Conversely, if positioned too low, the wide crossarm bounding box engulfs nearby hillside trees and dense canopy.
2. **Rigid Hardcoded Line-Direction Thickness**: Fixed meter limits on line-direction thickness either clip long insulator strings on 500kV towers or capture excessive mid-span hanging conductors on 110kV towers.
3. **Vegetation Infiltration at the Foundation**: The lower tapering frustum naturally expands to its maximum cross-sectional area at the ground, frequently encapsulating hillside terrain and steep-slope shrubs that lack physical structural connection to the steel tower.

## Solution

Formalize and implement the two-stage geometric convergence architecture (`UpperTowerBox` + `LowerTowerFrustum`), strictly abiding by ADR 0005 and domain model contracts in `CONTEXT.md`:
1. **WaistBoundaryElevation Anchor**: Anchor the vertical division elevation strictly 1.5m below the lowest candidate crossarm and tension jumper nadir, ensuring that all cantilever crossarms, diagonal bracing struts, and jumper loops remain completely enclosed within the `UpperTowerBox`.
2. **Voltage-Adaptive UpperTowerBox**: Construct an oriented 3D bounding box where the crossarm width along the transverse axis encloses all metallic cantilevers, and the line-direction thickness along the corridor axis dynamically scales with both crossarm span and total tower height, cleanly encapsulating insulators without over-capturing span conductors.
3. **LowerTowerFrustum with Pure Centroid Symmetry**: Project a four-sided truncated pyramid downwards from the waist boundary using the calibrated D15 pure-trunk median centroid and physical tower slope rate, preserving independent tolerance margins along the crossarm and line directions.
4. **Voxel Structural Connectivity Gating**: Constrain the interior of the lower frustum with 3D skeleton voxel connectivity growth seeded from the pure waist column, cleanly discarding unattached hillside shrubs and bare ground surface points.

## User Stories

1. As a transmission line algorithm engineer, I want the tower detection algorithm to converge using a dedicated `UpperTowerBox` for upper structures, so that multi-layer crossarms and drooping tension jumpers are 100% recalled without manual bounding box tuning.
2. As a LiDAR inspection developer, I want the transition elevation `WaistBoundaryElevation` to be positioned 1.5m below the lowest crossarm and jumper nadir, so that diagonal under-bracing struts on 220kV/500kV towers are never clipped by the narrow waist boundary.
3. As a point cloud processing technician, I want the line-direction thickness of `UpperTowerBox` to scale adaptively with tower height and crossarm span across 110kV, 220kV, and 500kV circuits, so that long insulator strings are fully preserved while mid-span conductors are cleanly excluded.
4. As a GIS data analyst, I want the lower tower structure to be bounded by a geometrically true `LowerTowerFrustum` aligned with the D15 pure-trunk median centroid, so that asymmetric conductors never pull the center of the tower frustum or shear off the opposing tower leg.
5. As a field inspection operator running data in steep mountainous terrain, I want points inside `LowerTowerFrustum` to be filtered via 3D skeleton voxel connectivity seeded from the clean tower waist, so that hillside vegetation and steep slope ground points enclosed in the wide foundation base are automatically rejected.
6. As a pipeline consumer, I want `detect_towers` to output standardized `TowerEntity` objects containing verified centroid coordinates, crossarm orientations, and bounding parameters, so that downstream wire catenary tracking receives reliable attachment priors.
7. As a QA automation engineer, I want regression tests to verify that both 500kV heavy towers (e.g. `68-69.las` Tower 2) and 220kV standard towers maintain zero missed steel leg points and zero tree false positives, so that updates never degrade production accuracy.
8. As a pipeline architect, I want the two-stage convergence implementation to execute within milliseconds without heavy iterative mesh decimation, so that overall corridor processing throughput remains optimal.

## Implementation Decisions

1. **Two-Stage Geometric Model**: Split tower spatial bounding into two decoupled entities: `UpperTowerBox` (an oriented 3D bounding box) and `LowerTowerFrustum` (a linear truncated pyramid frustum).
2. **Boundary Elevation Formula**: Calculate `WaistBoundaryElevation` as `min(crossarm_candidate_z) - 1.5m`. In the absence of candidate crossarms, fall back to a height-scaled waist ratio between 48% and 60% of total structural height.
3. **Line-Direction Adaptive Thickness**: Replace hardcoded thickness limits with an adaptive formula scaling with crossarm half-width and total tower height:
   $$W_{\text{line\_half}} = \frac{W_{\text{trunk\_line}}}{2} + \min(0.18 \times W_{\text{arm\_half}},\ 0.08 \times H)$$
   ensuring smooth scaling across 110kV to 500kV+ circuits.
4. **Frustum Slope & Centroid Alignment**: Center the frustum strictly on the multi-slice pure-trunk median centroid. Apply a height-scaled physical slope rate ($0.0811 \times k_{\text{height}}$) symmetrically in both horizontal dimensions, while maintaining independent margin parameters for structural flange clearance.
5. **Topological Seeded Voxel Connectivity**: Retain 3D voxel connectivity growth (voxel size 0.45m, query radius 0.85m) downward from the pure column waist seeds to validate steel continuity in `LowerTowerFrustum`, filtering out disconnected hillside ground canopy.
6. **Decoupled Downstream Responsibilities**: Stage 2 focuses strictly on tower body convergence and tree rejection; fine-grained separation of insulator attachment clips from wire catenaries is left to Stage 3 wire extraction and Stage 4 topology validation.

## Testing Decisions

- **Good Test Criteria**:
  - Tests must only evaluate external behavior across defined seams: input point coordinates and height metrics $\to$ output `TowerEntity` count, spatial center accuracy, and boolean classification masks (`is_tower`, `is_tower_arm`).
  - Tests must assert zero missed leg points on known asymmetric test cases and zero false-positive tower classifications on dense hillside trees.
  - Tests must run deterministically in-memory in under 1 second without disk dependencies.
- **Modules Tested**:
  - `modules.tower_detector` (`detect_towers` and helper geometric bounding functions).
  - `modules.pipeline_executor` (`PipelineExecutor.run` with `stop_after=PipelineStage.TOWER`).
- **Prior Art**:
  - `tests/test_lower_tower_capture.py`: verified multi-slice waist capture and slope envelope.
  - `tests/test_tower_center_localization.py`: verified pure-trunk centroid recalibration under asymmetric arm pulling.

## Out of Scope

- Modifying Stage 3 catenary curve fitting or tension jumper extraction algorithms.
- Changing LAS exporter format or color classification codes.
- Redesigning Stage 1 ground separation or DEM generation.

## Further Notes

- All changes conform to ADR 0005 (`docs/adr/0005-two-stage-tower-convergence-box-and-frustum.md`) and the domain vocabulary defined in `CONTEXT.md`.
