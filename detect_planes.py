from __future__ import annotations

import argparse
import importlib
import json
import math
import os
from dataclasses import dataclass, field
from typing import Any

import numpy as np


# Legacy prototype kept only as reference; the semantic pipeline is defined below.
def _legacy_load_point_cloud(file_path):
    """Carica la nuvola di punti, supportando nativamente file .las e .laz tramite laspy."""
    ext = os.path.splitext(file_path)[1].lower()
    if ext in ['.las', '.laz']:
        print(f"Caricamento file {ext} tramite laspy in corso...")
        las = laspy.read(file_path)
        # Estrazione coordinate
        coords = np.vstack((las.x, las.y, las.z)).transpose()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
        
        # Prova a estrarre i colori se presenti
        try:
            r = las.red / 65535.0
            g = las.green / 65535.0
            b = las.blue / 65535.0
            pcd.colors = o3d.utility.Vector3dVector(np.vstack((r, g, b)).transpose())
        except AttributeError:
            pass 
        return pcd
    else:
        print(f"Caricamento file {ext} tramite Open3D...")
        return o3d.io.read_point_cloud(file_path)

def _legacy_detect_and_classify_planes(file_path, knn, planes_only, hide_oblique):
    """Esegue l'estrazione iterativa dei piani: prima pavimenti, poi pareti."""
    if not os.path.exists(file_path):
        print(f"Errore: Il file {file_path} non esiste.")
        return

    # --- Pre-elaborazione ---
    pcd = _legacy_load_point_cloud(file_path)
    
    # Centriamo la nuvola per evitare problemi di precisione in virgola mobile (tipici nei file georeferenziati)
    center = pcd.get_center()
    pcd.translate(-center)
    
    print(f"Stima delle normali (knn={knn})...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    # Allineiamo le normali per garantire che la direzione Z sia corretta
    pcd.orient_normals_to_align_with_direction(np.array([0., 0., 1.]))

    # Lista in cui accumuleremo le geometrie da visualizzare
    geometries = [] if planes_only else [pcd]

    # ==========================================
    # PASSAGGIO 1: ESTRAZIONE DEI PIANI ORIZZONTALI
    # (Alta tolleranza, per assorbire ingombri a terra)
    # ==========================================
    print("\n[Passaggio 1] Ricerca dei piani orizzontali (pavimenti/tetti)...")
    oboxes_horiz = pcd.detect_planar_patches(
        normal_variance_threshold_deg=45.0, # Tollera pavimenti irregolari
        coplanarity_deg=80.0,               # Assorbe ostacoli e ingombri a terra
        outlier_ratio=0.90,                 # Molto permissivo per non spezzare il piano
        min_plane_edge_length=3.0,          # Solo i grandi ponti/pavimenti (> 3m)
        min_num_points=1000,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn)
    )
    
    horiz_count = 0
    for obox in oboxes_horiz:
        min_axis_idx = np.argmin(obox.extent)
        normal = obox.R[:, min_axis_idx]
        verticality = 1.0 - abs(normal[2])
        
        # Salviamo SOLO se la geometria estratta è realmente orizzontale
        if verticality < 0.2:
            mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.05])
            mesh.compute_vertex_normals()
            mesh.paint_uniform_color([0.0, 0.0, 1.0])  # Colore Blu
            geometries.append(mesh)
            horiz_count += 1
            
    print(f"-> Trovati {horiz_count} piani orizzontali.")

    # ==========================================
    # PASSAGGIO 2: ESTRAZIONE DELLE PARETI VERTICALI
    # (Vincoli rigidi, per evitare che i muri "scivolino" sulle curve dello scafo)
    # ==========================================
    print("\n[Passaggio 2] Ricerca delle pareti verticali (vincoli rigidi sulle curve)...")
    oboxes_vert = pcd.detect_planar_patches(
        normal_variance_threshold_deg=15.0, # Blocca lo scivolamento sulle pareti curve dello scafo
        coplanarity_deg=70.0,               # Permette di saltare tubi a muro ma senza esagerare
        outlier_ratio=0.85,                 
        min_plane_edge_length=1.5,          # Trova muri lunghi almeno 1.5m (elimina micro-frammenti)
        min_num_points=100,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn)
    )
    
    vert_count = 0
    oblique_count = 0
    for obox in oboxes_vert:
        min_axis_idx = np.argmin(obox.extent)
        normal = obox.R[:, min_axis_idx]
        verticality = 1.0 - abs(normal[2])
        
        # Salviamo la geometria se è verticale (tolleranza allargata a 0.6 per i muri un po' storti)
        if verticality > 0.6:  
            mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.05])
            mesh.compute_vertex_normals()
            mesh.paint_uniform_color([1.0, 0.0, 0.0])  # Colore Rosso
            geometries.append(mesh)
            vert_count += 1
            
        # Gestione dei piani obliqui/grigi (potrebbero essere artefatti o muri molto storti)
        elif verticality >= 0.2 and verticality <= 0.6:
            if not hide_oblique:
                mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.05])
                mesh.compute_vertex_normals()
                mesh.paint_uniform_color([0.5, 0.5, 0.5])  # Colore Grigio
                geometries.append(mesh)
                oblique_count += 1

    print(f"-> Trovate {vert_count} pareti verticali.")
    if not hide_oblique:
        print(f"-> Trovati {oblique_count} piani obliqui (grigi).")

    # --- Visualizzazione ---
    print("\nApertura del visualizzatore 3D in corso...")
    o3d.visualization.draw_geometries(geometries, window_name="Analisi Strutturale - Piani Estratti")


if False and __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Estrazione iterativa di pavimenti e pareti da nuvole di punti.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument("input_file", type=str, help="Percorso del file della nuvola di punti (.las, .laz, .ply, .pcd).")
    parser.add_argument("--knn", type=int, default=30, help="Punti vicini considerati per il calcolo delle normali.")
    parser.add_argument("--planes-only", action="store_true", help="Nasconde la nuvola di punti originale e mostra solo i piani 3D.")
    parser.add_argument("--hide-oblique", action="store_true", help="Nasconde completamente i piani grigi (piani non perfettamente orizzontali né verticali).")
    
    args = parser.parse_args()
    
    _legacy_detect_and_classify_planes(
        args.input_file, 
        args.knn, 
        args.planes_only, 
        args.hide_oblique
    )


EPSILON = 1e-9
HORIZONTAL_COLOR = np.array([0.05, 0.35, 0.95], dtype=float)
VERTICAL_COLOR = np.array([0.90, 0.10, 0.10], dtype=float)
OBLIQUE_COLOR = np.array([0.55, 0.55, 0.55], dtype=float)
INTERSECTION_COLOR = np.array([0.10, 0.85, 0.20], dtype=float)
OBLIQUE_INTERSECTION_COLOR = np.array([1.00, 0.65, 0.10], dtype=float)


def require_module(module_name: str, install_hint: str):
    try:
        return importlib.import_module(module_name)
    except ImportError as exc:
        raise SystemExit(
            f"Manca il modulo '{module_name}'. Installa le dipendenze richieste con: {install_hint}"
        ) from exc


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < EPSILON:
        return vector.copy()
    return vector / norm


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def angle_between_normals_deg(a: np.ndarray, b: np.ndarray) -> float:
    cosine = clamp(float(np.dot(normalize(a), normalize(b))), -1.0, 1.0)
    return math.degrees(math.acos(abs(cosine)))


def cross2d(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def polygon_area_2d(polygon: np.ndarray) -> float:
    if len(polygon) < 3:
        return 0.0
    x = polygon[:, 0]
    y = polygon[:, 1]
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))) * 0.5


def clean_polygon(polygon: np.ndarray, tol: float = 1e-6) -> np.ndarray:
    if len(polygon) < 3:
        return np.empty((0, 2), dtype=float)

    cleaned: list[np.ndarray] = []
    for point in polygon:
        if not cleaned or np.linalg.norm(point - cleaned[-1]) > tol:
            cleaned.append(point)

    if len(cleaned) >= 2 and np.linalg.norm(cleaned[0] - cleaned[-1]) <= tol:
        cleaned.pop()

    if len(cleaned) < 3:
        return np.empty((0, 2), dtype=float)

    reduced: list[np.ndarray] = []
    total = len(cleaned)
    for index in range(total):
        prev_point = cleaned[index - 1]
        curr_point = cleaned[index]
        next_point = cleaned[(index + 1) % total]
        edge_a = curr_point - prev_point
        edge_b = next_point - curr_point
        if abs(cross2d(edge_a, edge_b)) <= tol and np.dot(edge_a, edge_b) >= 0:
            continue
        reduced.append(curr_point)

    if len(reduced) < 3:
        return np.empty((0, 2), dtype=float)

    return np.asarray(reduced, dtype=float)


def merge_intervals(intervals: list[tuple[float, float]], tol: float = 1e-6) -> list[tuple[float, float]]:
    if not intervals:
        return []
    intervals = sorted(intervals, key=lambda item: item[0])
    merged = [intervals[0]]
    for start, end in intervals[1:]:
        last_start, last_end = merged[-1]
        if start <= last_end + tol:
            merged[-1] = (last_start, max(last_end, end))
        else:
            merged.append((start, end))
    return merged


def intersect_intervals(
    first: list[tuple[float, float]],
    second: list[tuple[float, float]],
    tol: float = 1e-6,
) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    i = 0
    j = 0
    while i < len(first) and j < len(second):
        start = max(first[i][0], second[j][0])
        end = min(first[i][1], second[j][1])
        if end - start > tol:
            result.append((start, end))
        if first[i][1] < second[j][1]:
            i += 1
        else:
            j += 1
    return result


def solve_plane_intersection(
    normal_a: np.ndarray,
    offset_a: float,
    normal_b: np.ndarray,
    offset_b: float,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    direction = np.cross(normal_a, normal_b)
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm < EPSILON:
        return None, None

    direction = direction / direction_norm
    system = np.vstack([normal_a, normal_b, direction])
    rhs = np.array([offset_a, offset_b, 0.0], dtype=float)
    try:
        point = np.linalg.solve(system, rhs)
    except np.linalg.LinAlgError:
        point = np.linalg.lstsq(system, rhs, rcond=None)[0]
    return point, direction


def clip_line_to_convex_polygon(
    polygon: np.ndarray,
    line_point: np.ndarray,
    line_direction: np.ndarray,
    tol: float = 1e-6,
) -> tuple[float, float] | None:
    if len(polygon) < 3:
        return None

    t_min = -np.inf
    t_max = np.inf
    for index in range(len(polygon)):
        current = polygon[index]
        nxt = polygon[(index + 1) % len(polygon)]
        edge = nxt - current
        denom = cross2d(edge, line_direction)
        numer = cross2d(edge, line_point - current)

        if abs(denom) <= tol:
            if numer < -tol:
                return None
            continue

        boundary_t = -numer / denom
        if denom > 0:
            t_min = max(t_min, boundary_t)
        else:
            t_max = min(t_max, boundary_t)

        if t_min - t_max > tol:
            return None

    return (t_min, t_max)


def clip_polygon_against_halfspace(
    polygon: np.ndarray,
    line_normal: np.ndarray,
    line_offset: float,
    keep_positive: bool,
    tol: float = 1e-6,
) -> np.ndarray:
    if len(polygon) == 0:
        return polygon

    def signed_value(point: np.ndarray) -> float:
        return float(np.dot(line_normal, point) + line_offset)

    def inside(value: float) -> bool:
        return value >= -tol if keep_positive else value <= tol

    clipped: list[np.ndarray] = []
    for index in range(len(polygon)):
        current = polygon[index]
        nxt = polygon[(index + 1) % len(polygon)]
        current_value = signed_value(current)
        next_value = signed_value(nxt)
        current_inside = inside(current_value)
        next_inside = inside(next_value)

        if current_inside and next_inside:
            clipped.append(nxt)
        elif current_inside and not next_inside:
            alpha = current_value / (current_value - next_value)
            clipped.append(current + alpha * (nxt - current))
        elif not current_inside and next_inside:
            alpha = current_value / (current_value - next_value)
            clipped.append(current + alpha * (nxt - current))
            clipped.append(nxt)

    if not clipped:
        return np.empty((0, 2), dtype=float)

    return clean_polygon(np.asarray(clipped, dtype=float), tol=tol)


def merge_occupied_cells(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    rows, cols = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    rectangles: list[tuple[int, int, int, int]] = []

    for row in range(rows):
        col = 0
        while col < cols:
            if not mask[row, col] or visited[row, col]:
                col += 1
                continue

            width = 1
            while (
                col + width < cols
                and mask[row, col + width]
                and not visited[row, col + width]
            ):
                width += 1

            height = 1
            while row + height < rows:
                candidate = mask[row + height, col : col + width]
                if candidate.size == 0:
                    break
                if not np.all(candidate & (~visited[row + height, col : col + width])):
                    break
                height += 1

            visited[row : row + height, col : col + width] = True
            rectangles.append((row, col, height, width))
            col += width

    return rectangles


def serialize_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, float)):
        return float(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, list):
        return [serialize_value(item) for item in value]
    if isinstance(value, tuple):
        return [serialize_value(item) for item in value]
    if isinstance(value, dict):
        return {key: serialize_value(item) for key, item in value.items()}
    return value


def category_from_normal(normal: np.ndarray) -> str:
    z_alignment = abs(float(normal[2]))
    if z_alignment >= 0.80:
        return "horizontal"
    if z_alignment <= 0.40:
        return "vertical"
    return "oblique"


def color_from_category(category: str) -> np.ndarray:
    if category == "horizontal":
        return HORIZONTAL_COLOR.copy()
    if category == "vertical":
        return VERTICAL_COLOR.copy()
    return OBLIQUE_COLOR.copy()


@dataclass
class CandidatePlane:
    source_pass: str
    category: str
    color: np.ndarray
    center: np.ndarray
    normal: np.ndarray
    u_axis: np.ndarray
    v_axis: np.ndarray
    extent_uv: np.ndarray
    thin_extent: float


@dataclass
class PlanePatch:
    plane_id: int
    source_pass: str
    category: str
    semantic_type: str
    color: np.ndarray
    origin: np.ndarray
    normal: np.ndarray
    u_axis: np.ndarray
    v_axis: np.ndarray
    support_points: np.ndarray
    support_uv: np.ndarray
    support_indices: np.ndarray
    cell_size: float
    support_distance: float
    raw_extent_uv: np.ndarray
    uv_bounds: np.ndarray
    polygons_uv_initial: list[np.ndarray]
    polygons_uv: list[np.ndarray]
    area_bbox: float
    area_supported: float
    area_trimmed: float
    grid_shape: tuple[int, int]
    trim_operations: list[dict[str, Any]] = field(default_factory=list)
    semantic_relations: list[dict[str, Any]] = field(default_factory=list)

    @property
    def offset(self) -> float:
        return float(np.dot(self.normal, self.origin))

    def world_from_uv(self, uv: np.ndarray, normal_offset: float = 0.0) -> np.ndarray:
        return (
            self.origin
            + np.outer(uv[:, 0], self.u_axis)
            + np.outer(uv[:, 1], self.v_axis)
            + normal_offset * self.normal
        )

    def local_uv_from_world(self, points: np.ndarray) -> np.ndarray:
        relative = points - self.origin
        return np.column_stack([relative @ self.u_axis, relative @ self.v_axis])

    def area_ratio(self) -> float:
        if self.area_bbox <= EPSILON:
            return 0.0
        return self.area_trimmed / self.area_bbox


SEMANTIC_TYPE_COLORS = {
    "wall": np.array([0.90, 0.10, 0.10], dtype=float),
    "floor": np.array([0.05, 0.35, 0.95], dtype=float),
    "ceiling": np.array([0.10, 0.75, 0.85], dtype=float),
    "oblique": np.array([0.55, 0.55, 0.55], dtype=float),
    "unknown": np.array([0.80, 0.80, 0.80], dtype=float),
}


def semantic_type_from_plane(plane: PlanePatch, scene_bounds: tuple[np.ndarray, np.ndarray]) -> str:
    if plane.category == "vertical":
        return "wall"
    if plane.category == "oblique":
        return "oblique"

    min_bound, max_bound = scene_bounds
    distance_to_floor = abs(float(plane.origin[2] - min_bound[2]))
    distance_to_ceiling = abs(float(max_bound[2] - plane.origin[2]))
    return "floor" if distance_to_floor <= distance_to_ceiling else "ceiling"


def assign_semantic_types(planes: list[PlanePatch], scene_bounds: tuple[np.ndarray, np.ndarray]) -> None:
    for plane in planes:
        plane.semantic_type = semantic_type_from_plane(plane, scene_bounds)
        plane.color = SEMANTIC_TYPE_COLORS.get(plane.semantic_type, SEMANTIC_TYPE_COLORS["unknown"]).copy()


def load_point_cloud(file_path: str):
    o3d = require_module("open3d", "pip install open3d laspy")
    ext = os.path.splitext(file_path)[1].lower()

    if ext in [".las", ".laz"]:
        laspy = require_module("laspy", "pip install laspy")
        print(f"Caricamento file {ext} tramite laspy...")
        las = laspy.read(file_path)
        coords = np.vstack((las.x, las.y, las.z)).transpose()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
        try:
            r = las.red.astype(np.float64) / 65535.0
            g = las.green.astype(np.float64) / 65535.0
            b = las.blue.astype(np.float64) / 65535.0
            pcd.colors = o3d.utility.Vector3dVector(np.vstack((r, g, b)).transpose())
        except AttributeError:
            pass
        return o3d, pcd

    print(f"Caricamento file {ext} tramite Open3D...")
    return o3d, o3d.io.read_point_cloud(file_path)


def preprocess_point_cloud(
    file_path: str,
    knn: int,
    voxel_size: float,
) -> tuple[Any, Any, np.ndarray, float]:
    if not os.path.exists(file_path):
        raise SystemExit(f"Errore: il file '{file_path}' non esiste.")

    o3d, pcd = load_point_cloud(file_path)
    if len(pcd.points) == 0:
        raise SystemExit("Errore: la nuvola di punti è vuota.")

    original_center = np.asarray(pcd.get_center(), dtype=float)
    pcd.translate(-original_center)

    if voxel_size > 0:
        print(f"Voxel downsampling attivo (voxel_size={voxel_size:.3f})...")
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    if len(pcd.points) == 0:
        raise SystemExit("Errore: il downsampling ha eliminato tutti i punti.")

    print(f"Punti usati per l'analisi: {len(pcd.points):,}")
    print(f"Stima delle normali (knn={knn})...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    pcd.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))

    points = np.asarray(pcd.points, dtype=float)
    scene_extent = np.ptp(points, axis=0)
    scene_diag = float(np.linalg.norm(scene_extent))
    return o3d, pcd, original_center, scene_diag


def candidate_from_obb(obox: Any, source_pass: str) -> CandidatePlane:
    axes = np.asarray(obox.R, dtype=float)
    extents = np.asarray(obox.extent, dtype=float)
    min_axis_index = int(np.argmin(extents))
    tangent_indices = [axis for axis in range(3) if axis != min_axis_index]
    tangent_indices.sort(key=lambda axis: extents[axis], reverse=True)

    normal = normalize(axes[:, min_axis_index])
    u_axis = normalize(axes[:, tangent_indices[0]])
    v_axis = normalize(axes[:, tangent_indices[1]])
    if np.dot(np.cross(u_axis, v_axis), normal) < 0:
        v_axis = -v_axis

    category = category_from_normal(normal)
    return CandidatePlane(
        source_pass=source_pass,
        category=category,
        color=color_from_category(category),
        center=np.asarray(obox.center, dtype=float),
        normal=normal,
        u_axis=u_axis,
        v_axis=v_axis,
        extent_uv=np.array([extents[tangent_indices[0]], extents[tangent_indices[1]]], dtype=float),
        thin_extent=float(extents[min_axis_index]),
    )


def detect_plane_candidates(o3d: Any, pcd: Any, knn: int, hide_oblique: bool, args) -> list[CandidatePlane]:
    print("\n[Passaggio 1] Ricerca dei piani orizzontali...")
    horizontal_boxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=45.0,
        coplanarity_deg=80.0,
        outlier_ratio=0.90,
        min_plane_edge_length=args.horizontal_min_edge,
        min_num_points=args.horizontal_min_points,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn),
    )

    candidates: list[CandidatePlane] = []
    for obox in horizontal_boxes:
        candidate = candidate_from_obb(obox, "horizontal_pass")
        if candidate.category == "horizontal":
            candidates.append(candidate)

    print(f"-> Candidati orizzontali: {len(candidates)}")

    print("\n[Passaggio 2] Ricerca dei piani verticali/obliqui...")
    vertical_boxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=15.0,
        coplanarity_deg=70.0,
        outlier_ratio=0.85,
        min_plane_edge_length=args.vertical_min_edge,
        min_num_points=args.vertical_min_points,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn),
    )

    vertical_count = 0
    oblique_count = 0
    for obox in vertical_boxes:
        candidate = candidate_from_obb(obox, "vertical_pass")
        if candidate.category == "vertical":
            candidates.append(candidate)
            vertical_count += 1
        elif candidate.category == "oblique" and not hide_oblique:
            candidates.append(candidate)
            oblique_count += 1

    print(f"-> Candidati verticali: {vertical_count}")
    if not hide_oblique:
        print(f"-> Candidati obliqui: {oblique_count}")
    return candidates


def auto_cell_size(extent_uv: np.ndarray, scene_diag: float, voxel_size: float) -> float:
    base = max(scene_diag * 0.002, voxel_size if voxel_size > 0 else 0.0, 0.03)
    max_reasonable = max(min(float(extent_uv[0]), float(extent_uv[1])) / 6.0, 0.03)
    return min(base, max_reasonable)


def auto_support_distance(candidate: CandidatePlane, cell_size: float, scene_diag: float) -> float:
    return max(candidate.thin_extent * 3.0, cell_size * 0.85, scene_diag * 0.0009, 0.02)


def extract_support_points(
    points: np.ndarray,
    candidate: CandidatePlane,
    support_distance: float,
    tangent_padding: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    relative = points - candidate.center
    local_u = relative @ candidate.u_axis
    local_v = relative @ candidate.v_axis
    local_n = relative @ candidate.normal

    half_u = candidate.extent_uv[0] * 0.5 + tangent_padding
    half_v = candidate.extent_uv[1] * 0.5 + tangent_padding
    mask = (
        (np.abs(local_n) <= support_distance)
        & (np.abs(local_u) <= half_u)
        & (np.abs(local_v) <= half_v)
    )

    indices = np.flatnonzero(mask)
    local_uv = np.column_stack([local_u[mask], local_v[mask]])
    return indices, points[mask], local_uv


def refine_plane_frame(points: np.ndarray, initial_normal: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)

    u_axis = normalize(vh[0])
    v_axis = normalize(vh[1])
    normal = normalize(vh[2])

    if np.dot(normal, initial_normal) < 0:
        normal = -normal
    if np.dot(np.cross(u_axis, v_axis), normal) < 0:
        v_axis = -v_axis

    return centroid, normal, u_axis, v_axis


def build_supported_polygons(
    support_uv: np.ndarray,
    cell_size: float,
    min_points_per_cell: int,
    trim_quantile: float,
) -> tuple[list[np.ndarray], np.ndarray, float, tuple[int, int]]:
    if len(support_uv) == 0:
        return [], np.zeros((2, 2), dtype=float), 0.0, (0, 0)

    lower = np.percentile(support_uv, trim_quantile, axis=0)
    upper = np.percentile(support_uv, 100.0 - trim_quantile, axis=0)
    if np.any(upper - lower < EPSILON):
        lower = support_uv.min(axis=0)
        upper = support_uv.max(axis=0)

    lower -= cell_size * 0.5
    upper += cell_size * 0.5
    size = np.maximum(upper - lower, cell_size)
    grid_size = np.maximum(np.ceil(size / cell_size).astype(int), 1)

    counts = np.zeros((int(grid_size[1]), int(grid_size[0])), dtype=int)
    indices = np.floor((support_uv - lower) / cell_size).astype(int)
    indices[:, 0] = np.clip(indices[:, 0], 0, counts.shape[1] - 1)
    indices[:, 1] = np.clip(indices[:, 1], 0, counts.shape[0] - 1)
    np.add.at(counts, (indices[:, 1], indices[:, 0]), 1)

    occupied = counts >= min_points_per_cell
    if not np.any(occupied):
        occupied = counts > 0

    polygons: list[np.ndarray] = []
    supported_area = 0.0
    for row, col, height, width in merge_occupied_cells(occupied):
        u0 = lower[0] + col * cell_size
        u1 = lower[0] + (col + width) * cell_size
        v0 = lower[1] + row * cell_size
        v1 = lower[1] + (row + height) * cell_size
        polygon = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]], dtype=float)
        polygons.append(polygon)
        supported_area += (u1 - u0) * (v1 - v0)

    uv_bounds = np.array([[lower[0], upper[0]], [lower[1], upper[1]]], dtype=float)
    return polygons, uv_bounds, supported_area, occupied.shape


def plane_patch_from_candidate(
    candidate: CandidatePlane,
    points: np.ndarray,
    scene_diag: float,
    args,
) -> PlanePatch | None:
    cell_size = args.semantic_cell_size or auto_cell_size(candidate.extent_uv, scene_diag, args.voxel_size)
    support_distance = args.support_distance or auto_support_distance(candidate, cell_size, scene_diag)
    tangent_padding = max(cell_size * 1.5, args.tangent_padding)

    support_indices, support_points, _ = extract_support_points(
        points,
        candidate,
        support_distance=support_distance,
        tangent_padding=tangent_padding,
    )
    if len(support_points) < args.min_support_points:
        return None

    origin, normal, u_axis, v_axis = refine_plane_frame(support_points, candidate.normal)
    relative = support_points - origin
    refined_uv = np.column_stack([relative @ u_axis, relative @ v_axis])

    polygons_uv, uv_bounds, supported_area, grid_shape = build_supported_polygons(
        refined_uv,
        cell_size=cell_size,
        min_points_per_cell=args.min_points_per_cell,
        trim_quantile=args.trim_quantile,
    )
    if not polygons_uv:
        return None

    raw_extent_uv = np.array(
        [
            max(candidate.extent_uv[0], uv_bounds[0, 1] - uv_bounds[0, 0]),
            max(candidate.extent_uv[1], uv_bounds[1, 1] - uv_bounds[1, 0]),
        ],
        dtype=float,
    )
    area_bbox = float(raw_extent_uv[0] * raw_extent_uv[1])
    if supported_area < args.min_supported_area:
        return None

    return PlanePatch(
        plane_id=-1,
        source_pass=candidate.source_pass,
        category=candidate.category,
        semantic_type="unknown",
        color=candidate.color.copy(),
        origin=origin,
        normal=normal,
        u_axis=u_axis,
        v_axis=v_axis,
        support_points=support_points,
        support_uv=refined_uv,
        support_indices=support_indices,
        cell_size=cell_size,
        support_distance=support_distance,
        raw_extent_uv=raw_extent_uv,
        uv_bounds=uv_bounds,
        polygons_uv_initial=[polygon.copy() for polygon in polygons_uv],
        polygons_uv=[polygon.copy() for polygon in polygons_uv],
        area_bbox=area_bbox,
        area_supported=supported_area,
        area_trimmed=supported_area,
        grid_shape=grid_shape,
    )


def deduplicate_planes(planes: list[PlanePatch], args) -> list[PlanePatch]:
    keep_flags = [True] * len(planes)
    for first in range(len(planes)):
        if not keep_flags[first]:
            continue
        for second in range(first + 1, len(planes)):
            if not keep_flags[second]:
                continue
            angle = angle_between_normals_deg(planes[first].normal, planes[second].normal)
            if angle > args.merge_angle_deg:
                continue

            plane_gap = abs(np.dot(planes[first].normal, planes[second].origin) - planes[first].offset)
            if plane_gap > args.merge_distance:
                continue

            first_indices = set(planes[first].support_indices.tolist())
            second_indices = set(planes[second].support_indices.tolist())
            intersection = len(first_indices & second_indices)
            if intersection == 0:
                continue

            union = len(first_indices | second_indices)
            coverage = intersection / max(min(len(first_indices), len(second_indices)), 1)
            jaccard = intersection / max(union, 1)
            if max(coverage, jaccard) < args.duplicate_overlap_ratio:
                continue

            first_score = (planes[first].area_trimmed, len(planes[first].support_indices))
            second_score = (planes[second].area_trimmed, len(planes[second].support_indices))
            if first_score >= second_score:
                keep_flags[second] = False
            else:
                keep_flags[first] = False
                break

    deduplicated = [plane for plane, keep in zip(planes, keep_flags) if keep]
    for plane_id, plane in enumerate(deduplicated):
        plane.plane_id = plane_id
    return deduplicated


def line_intervals_on_plane(plane: PlanePatch, point_world: np.ndarray, direction_world: np.ndarray) -> list[tuple[float, float]]:
    line_point_uv = plane.local_uv_from_world(point_world.reshape(1, 3))[0]
    line_direction_uv = np.array(
        [
            float(np.dot(direction_world, plane.u_axis)),
            float(np.dot(direction_world, plane.v_axis)),
        ],
        dtype=float,
    )
    if np.linalg.norm(line_direction_uv) < EPSILON:
        return []

    intervals: list[tuple[float, float]] = []
    for polygon in plane.polygons_uv:
        clipped = clip_line_to_convex_polygon(polygon, line_point_uv, line_direction_uv)
        if clipped is not None:
            intervals.append(clipped)
    return merge_intervals(intervals)


def compute_plane_relations(planes: list[PlanePatch], args) -> list[dict[str, Any]]:
    relations: list[dict[str, Any]] = []
    for plane in planes:
        plane.semantic_relations = []

    for first in range(len(planes)):
        for second in range(first + 1, len(planes)):
            plane_a = planes[first]
            plane_b = planes[second]
            angle = angle_between_normals_deg(plane_a.normal, plane_b.normal)

            if angle <= args.parallel_angle_deg:
                gap = abs(np.dot(plane_a.normal, plane_b.origin) - plane_a.offset)
                if gap <= args.merge_distance:
                    relation = {
                        "type": "coplanar",
                        "plane_a": plane_a.plane_id,
                        "plane_b": plane_b.plane_id,
                        "angle_deg": angle,
                        "gap": gap,
                    }
                    relations.append(relation)
                    plane_a.semantic_relations.append(relation)
                    plane_b.semantic_relations.append(relation)
                continue

            point_world, direction_world = solve_plane_intersection(
                plane_a.normal,
                plane_a.offset,
                plane_b.normal,
                plane_b.offset,
            )
            if point_world is None or direction_world is None:
                continue

            intervals_a = line_intervals_on_plane(plane_a, point_world, direction_world)
            intervals_b = line_intervals_on_plane(plane_b, point_world, direction_world)
            overlap = intersect_intervals(intervals_a, intervals_b)
            if not overlap:
                continue

            overlap_length = max(end - start for start, end in overlap)
            if overlap_length < args.min_intersection_length:
                continue

            start_t, end_t = overlap[0][0], overlap[-1][1]
            relation_type = (
                "orthogonal_intersection"
                if abs(angle - 90.0) <= args.orthogonal_angle_tol_deg
                else "oblique_intersection"
            )
            relation = {
                "type": relation_type,
                "plane_a": plane_a.plane_id,
                "plane_b": plane_b.plane_id,
                "angle_deg": angle,
                "line_point": point_world,
                "line_direction": direction_world,
                "segment_start": point_world + direction_world * start_t,
                "segment_end": point_world + direction_world * end_t,
                "segment_length": float(end_t - start_t),
            }
            relations.append(relation)
            plane_a.semantic_relations.append(relation)
            plane_b.semantic_relations.append(relation)

    return relations


def infer_boundary_clip(source: PlanePatch, boundary: PlanePatch, relation: dict[str, Any], args) -> dict[str, Any] | None:
    if relation["type"] not in {"orthogonal_intersection", "oblique_intersection"}:
        return None

    signed = source.support_points @ boundary.normal - boundary.offset
    threshold = max(source.cell_size * 0.75, source.support_distance * 0.5, 0.01)
    positive = int(np.sum(signed > threshold))
    negative = int(np.sum(signed < -threshold))
    stable_total = positive + negative
    if stable_total == 0:
        return None

    minority_fraction = min(positive, negative) / stable_total
    if minority_fraction > args.boundary_minor_fraction:
        return None

    keep_positive = positive >= negative
    line_normal = np.array(
        [
            float(np.dot(boundary.normal, source.u_axis)),
            float(np.dot(boundary.normal, source.v_axis)),
        ],
        dtype=float,
    )
    if np.linalg.norm(line_normal) < EPSILON:
        return None

    line_offset = float(np.dot(boundary.normal, source.origin) - boundary.offset)
    return {
        "boundary_plane_id": boundary.plane_id,
        "relation_type": relation["type"],
        "keep_positive": keep_positive,
        "minority_fraction": minority_fraction,
        "line_normal": line_normal,
        "line_offset": line_offset,
        "positive_points": positive,
        "negative_points": negative,
    }


def trim_plane_polygons(plane: PlanePatch, boundary_rules: list[dict[str, Any]], args) -> None:
    polygons = [polygon.copy() for polygon in plane.polygons_uv_initial]
    trim_operations: list[dict[str, Any]] = []

    for rule in boundary_rules:
        next_polygons: list[np.ndarray] = []
        area_before = sum(polygon_area_2d(polygon) for polygon in polygons)
        for polygon in polygons:
            clipped = clip_polygon_against_halfspace(
                polygon,
                line_normal=rule["line_normal"],
                line_offset=rule["line_offset"],
                keep_positive=rule["keep_positive"],
                tol=max(plane.cell_size * 0.05, 1e-5),
            )
            if len(clipped) >= 3 and polygon_area_2d(clipped) >= args.min_polygon_area:
                next_polygons.append(clipped)

        area_after = sum(polygon_area_2d(polygon) for polygon in next_polygons)
        if area_after + args.min_polygon_area < area_before:
            trim_operations.append(
                {
                    "boundary_plane_id": rule["boundary_plane_id"],
                    "relation_type": rule["relation_type"],
                    "keep_positive": rule["keep_positive"],
                    "minority_fraction": rule["minority_fraction"],
                    "removed_area": max(area_before - area_after, 0.0),
                }
            )

        polygons = next_polygons if next_polygons else polygons

    plane.polygons_uv = polygons
    plane.area_trimmed = sum(polygon_area_2d(polygon) for polygon in polygons)
    plane.trim_operations = trim_operations


def apply_semantic_trimming(planes: list[PlanePatch], relations: list[dict[str, Any]], args) -> None:
    relation_map: dict[int, list[dict[str, Any]]] = {plane.plane_id: [] for plane in planes}
    for relation in relations:
        relation_map[relation["plane_a"]].append(relation)
        relation_map[relation["plane_b"]].append(relation)

    for plane in planes:
        boundary_rules: list[dict[str, Any]] = []
        for relation in relation_map[plane.plane_id]:
            other_id = relation["plane_b"] if relation["plane_a"] == plane.plane_id else relation["plane_a"]
            other_plane = planes[other_id]
            rule = infer_boundary_clip(plane, other_plane, relation, args)
            if rule is not None:
                boundary_rules.append(rule)
        trim_plane_polygons(plane, boundary_rules, args)


def mesh_data_from_plane(plane: PlanePatch, thickness: float) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[list[float]] = []
    triangles: list[list[int]] = []

    for polygon in plane.polygons_uv:
        polygon = clean_polygon(polygon)
        if len(polygon) < 3:
            continue

        world_polygon = plane.world_from_uv(polygon)
        base_index = len(vertices)

        if thickness <= EPSILON:
            vertices.extend(world_polygon.tolist())
            for offset in range(1, len(polygon) - 1):
                triangles.append([base_index, base_index + offset, base_index + offset + 1])
            continue

        half = thickness * 0.5
        front = plane.world_from_uv(polygon, normal_offset=half)
        back = plane.world_from_uv(polygon, normal_offset=-half)
        vertices.extend(front.tolist())
        vertices.extend(back.tolist())

        count = len(polygon)
        for offset in range(1, count - 1):
            triangles.append([base_index, base_index + offset, base_index + offset + 1])
            triangles.append(
                [
                    base_index + count,
                    base_index + count + offset + 1,
                    base_index + count + offset,
                ]
            )

        for index in range(count):
            nxt = (index + 1) % count
            front_a = base_index + index
            front_b = base_index + nxt
            back_a = base_index + count + index
            back_b = base_index + count + nxt
            triangles.append([front_a, front_b, back_b])
            triangles.append([front_a, back_b, back_a])

    return np.asarray(vertices, dtype=float), np.asarray(triangles, dtype=np.int32)


def pyvista_mesh_from_plane(plane: PlanePatch, thickness: float):
    pv = require_module("pyvista", "pip install pyvista")
    vertices, triangles = mesh_data_from_plane(plane, thickness=thickness)
    if len(vertices) == 0 or len(triangles) == 0:
        return None

    face_sizes = np.full((len(triangles), 1), 3, dtype=np.int32)
    faces = np.hstack([face_sizes, triangles]).astype(np.int32).ravel()
    return pv.PolyData(vertices, faces)


def visualize_with_pyvista(
    points: np.ndarray,
    planes: list[PlanePatch],
    relations: list[dict[str, Any]],
    args,
) -> None:
    pv = require_module("pyvista", "pip install pyvista")
    plotter = pv.Plotter()

    try:
        plotter.set_background("#f3f5f7")
        if hasattr(plotter, "add_text"):
            plotter.add_text("Modello Semantico", position="upper_left", font_size=12)

        if not args.planes_only:
            cloud = pv.PolyData(points)
            plotter.add_points(
                cloud,
                color="#6b7280",
                point_size=2.0,
                render_points_as_spheres=True,
                opacity=0.55,
            )

        for plane in planes:
            mesh = pyvista_mesh_from_plane(plane, thickness=args.mesh_thickness)
            if mesh is not None and mesh.n_points > 0:
                plotter.add_mesh(
                    mesh,
                    color=plane.color.tolist(),
                    opacity=0.88,
                    show_edges=False,
                    smooth_shading=True,
                )

        if not args.hide_intersections:
            for relation in relations:
                if "segment_start" not in relation:
                    continue
                start = np.asarray(relation["segment_start"], dtype=float)
                end = np.asarray(relation["segment_end"], dtype=float)
                color = (
                    INTERSECTION_COLOR.tolist()
                    if relation["type"] == "orthogonal_intersection"
                    else OBLIQUE_INTERSECTION_COLOR.tolist()
                )
                plotter.add_mesh(pv.Line(start, end), color=color, line_width=4)

        if hasattr(plotter, "show_grid"):
            plotter.show_grid(color="#9ca3af")
        if hasattr(plotter, "add_axes"):
            plotter.add_axes(line_width=2)
        plotter.show()
    finally:
        if hasattr(plotter, "close"):
            plotter.close()


def ensure_report_dir(report_dir: str) -> str:
    directory = os.path.abspath(report_dir or "report")
    os.makedirs(directory, exist_ok=True)
    return directory


def report_base_name(input_file: str) -> str:
    return os.path.splitext(os.path.basename(input_file))[0]


def resolve_report_path(report_dir: str, input_file: str, suffix: str) -> str:
    return os.path.join(ensure_report_dir(report_dir), f"{report_base_name(input_file)}_{suffix}")


def plane_report_record(plane: PlanePatch) -> dict[str, Any]:
    return {
        "plane_id": plane.plane_id,
        "category": plane.category,
        "semantic_type": plane.semantic_type,
        "source_pass": plane.source_pass,
        "normal": plane.normal,
        "origin": plane.origin,
        "raw_extent_uv": plane.raw_extent_uv,
        "uv_bounds": plane.uv_bounds,
        "cell_size": plane.cell_size,
        "support_distance": plane.support_distance,
        "support_points": len(plane.support_indices),
        "grid_shape": plane.grid_shape,
        "area_bbox": plane.area_bbox,
        "area_supported": plane.area_supported,
        "area_trimmed": plane.area_trimmed,
        "coverage_ratio": plane.area_ratio(),
        "polygon_count": len(plane.polygons_uv),
        "trim_operations": plane.trim_operations,
        "relations": plane.semantic_relations,
        "polygons_world": [plane.world_from_uv(polygon) for polygon in plane.polygons_uv],
    }


def export_grouped_elements_report(
    planes: list[PlanePatch],
    relations: list[dict[str, Any]],
    report_dir: str,
    input_file: str,
    original_center: np.ndarray,
    scene_bounds: tuple[np.ndarray, np.ndarray],
) -> str:
    grouped: dict[str, list[dict[str, Any]]] = {
        "wall": [],
        "floor": [],
        "ceiling": [],
        "oblique": [],
        "unknown": [],
    }

    for plane in planes:
        grouped.setdefault(plane.semantic_type, []).append(plane_report_record(plane))

    summary = {
        semantic_type: {
            "count": len(items),
            "total_area": float(sum(item["area_trimmed"] for item in items)),
        }
        for semantic_type, items in grouped.items()
    }

    payload = {
        "input_file": os.path.abspath(input_file),
        "original_center": original_center,
        "scene_bounds": {
            "min": scene_bounds[0],
            "max": scene_bounds[1],
        },
        "summary": summary,
        "relations": relations,
        "elements": grouped,
    }

    report_path = resolve_report_path(report_dir, input_file, "elements_by_type.json")
    with open(report_path, "w", encoding="utf-8") as handle:
        json.dump(serialize_value(payload), handle, indent=2, ensure_ascii=False)
    return report_path


def export_trimmed_faces_dxf(
    planes: list[PlanePatch],
    report_dir: str,
    input_file: str,
) -> str:
    dxf_path = resolve_report_path(report_dir, input_file, "trimmed_faces.dxf")

    entities: list[str] = []
    for plane in planes:
        layer = plane.semantic_type.upper()
        for polygon in plane.polygons_uv:
            world_polygon = plane.world_from_uv(clean_polygon(polygon))
            if len(world_polygon) < 3:
                continue

            for index in range(1, len(world_polygon) - 1):
                a = world_polygon[0]
                b = world_polygon[index]
                c = world_polygon[index + 1]
                d = c
                entities.extend(
                    [
                        "0", "3DFACE",
                        "8", layer,
                        "10", f"{a[0]:.6f}", "20", f"{a[1]:.6f}", "30", f"{a[2]:.6f}",
                        "11", f"{b[0]:.6f}", "21", f"{b[1]:.6f}", "31", f"{b[2]:.6f}",
                        "12", f"{c[0]:.6f}", "22", f"{c[1]:.6f}", "32", f"{c[2]:.6f}",
                        "13", f"{d[0]:.6f}", "23", f"{d[1]:.6f}", "33", f"{d[2]:.6f}",
                    ]
                )

    dxf_lines = [
        "0", "SECTION",
        "2", "HEADER",
        "0", "ENDSEC",
        "0", "SECTION",
        "2", "ENTITIES",
        *entities,
        "0", "ENDSEC",
        "0", "EOF",
    ]

    with open(dxf_path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write("\n".join(dxf_lines))
        handle.write("\n")
    return dxf_path


def export_semantic_report(
    planes: list[PlanePatch],
    relations: list[dict[str, Any]],
    report_dir: str,
    input_file: str,
    original_center: np.ndarray,
    args,
) -> None:
    report = {
        "input_file": os.path.abspath(input_file),
        "original_center": original_center,
        "parameters": vars(args),
        "plane_count": len(planes),
        "intersection_count": sum(
            1
            for relation in relations
            if relation["type"] in {"orthogonal_intersection", "oblique_intersection"}
        ),
        "planes": [],
        "relations": relations,
    }

    for plane in planes:
        report["planes"].append(plane_report_record(plane))

    report_path = resolve_report_path(report_dir, input_file, "semantic_planes.json")
    with open(report_path, "w", encoding="utf-8") as handle:
        json.dump(serialize_value(report), handle, indent=2, ensure_ascii=False)
    print(f"\nReport semantico esportato in: {os.path.abspath(report_path)}")


def print_plane_summary(planes: list[PlanePatch], relations: list[dict[str, Any]]) -> None:
    orthogonal_count = sum(1 for relation in relations if relation["type"] == "orthogonal_intersection")
    oblique_count = sum(1 for relation in relations if relation["type"] == "oblique_intersection")

    print("\nRiepilogo semantico:")
    print(f"  Piani finali: {len(planes)}")
    print(f"  Intersezioni ortogonali: {orthogonal_count}")
    print(f"  Intersezioni oblique: {oblique_count}")

    for plane in planes:
        trim_area = sum(operation["removed_area"] for operation in plane.trim_operations)
        print(
            "  "
            f"Piano {plane.plane_id:02d} | {plane.semantic_type:10s} | "
            f"supporto={len(plane.support_indices):5d} pts | "
            f"area={plane.area_trimmed:7.2f} m² | "
            f"tagliato={trim_area:6.2f} m² | "
            f"relazioni={len(plane.semantic_relations):2d}"
        )


def build_plane_patches(points: np.ndarray, candidates: list[CandidatePlane], scene_diag: float, args) -> list[PlanePatch]:
    patches: list[PlanePatch] = []
    for candidate in candidates:
        patch = plane_patch_from_candidate(candidate, points, scene_diag, args)
        if patch is not None:
            patches.append(patch)
    return patches


def detect_and_classify_planes(args) -> None:
    o3d, pcd, original_center, scene_diag = preprocess_point_cloud(
        args.input_file,
        knn=args.knn,
        voxel_size=args.voxel_size,
    )
    points = np.asarray(pcd.points, dtype=float)
    scene_bounds = (points.min(axis=0), points.max(axis=0))

    candidates = detect_plane_candidates(o3d, pcd, args.knn, args.hide_oblique, args)
    print(f"\nCandidati complessivi: {len(candidates)}")

    patches = build_plane_patches(points, candidates, scene_diag, args)
    print(f"Patch supportate dai punti: {len(patches)}")

    patches = deduplicate_planes(patches, args)
    print(f"Patch dopo deduplicazione: {len(patches)}")

    initial_relations = compute_plane_relations(patches, args)
    apply_semantic_trimming(patches, initial_relations, args)
    final_relations = compute_plane_relations(patches, args)
    assign_semantic_types(patches, scene_bounds)
    print_plane_summary(patches, final_relations)

    report_dir = ensure_report_dir(args.report_dir)
    export_semantic_report(
        patches,
        final_relations,
        report_dir=report_dir,
        input_file=args.input_file,
        original_center=original_center,
        args=args,
    )
    grouped_report_path = export_grouped_elements_report(
        patches,
        final_relations,
        report_dir=report_dir,
        input_file=args.input_file,
        original_center=original_center,
        scene_bounds=scene_bounds,
    )
    print(f"Report per tipo elemento esportato in: {os.path.abspath(grouped_report_path)}")

    if args.cad_export.lower() not in {"off", "none", "false"}:
        cad_export_path = export_trimmed_faces_dxf(
            patches,
            report_dir=report_dir,
            input_file=args.input_file,
        )
        print(f"Export CAD-like esportato in: {os.path.abspath(cad_export_path)}")

    if args.no_visualization:
        return

    print("\nApertura del visualizzatore 3D con PyVista...")
    visualize_with_pyvista(points, patches, final_relations, args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Estrae piani da nuvole di punti e ricostruisce una semantica topologica "
            "basata su intersezioni e trimming delle porzioni in eccesso."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("input_file", type=str, help="Percorso del file .las/.laz/.ply/.pcd.")
    parser.add_argument("--knn", type=int, default=30, help="Vicini usati per la stima delle normali.")
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="Voxel downsampling prima della segmentazione. Usa 0 per disattivarlo.",
    )
    parser.add_argument("--planes-only", action="store_true", help="Mostra solo il modello trimmato.")
    parser.add_argument("--hide-oblique", action="store_true", help="Esclude i piani obliqui.")
    parser.add_argument("--hide-intersections", action="store_true", help="Nasconde le linee di intersezione.")
    parser.add_argument("--no-visualization", action="store_true", help="Esegue la pipeline senza viewer.")
    parser.add_argument("--mesh-thickness", type=float, default=0.05, help="Spessore visuale delle superfici planari.")
    parser.add_argument("--report-dir", type=str, default="report", help="Cartella in cui salvare tutti i report e gli export.")
    parser.add_argument("--cad-export", type=str, default="dxf", help="Export CAD-like delle facce trimmate. Usa 'off' per disattivarlo.")

    parser.add_argument("--horizontal-min-edge", type=float, default=3.0, help="Lato minimo per i grandi piani orizzontali.")
    parser.add_argument("--horizontal-min-points", type=int, default=1000, help="Punti minimi per piano orizzontale.")
    parser.add_argument("--vertical-min-edge", type=float, default=1.5, help="Lato minimo per le pareti verticali.")
    parser.add_argument("--vertical-min-points", type=int, default=100, help="Punti minimi per piano verticale.")

    parser.add_argument("--semantic-cell-size", type=float, default=0.0, help="Risoluzione 2D delle facce; 0 = auto.")
    parser.add_argument("--support-distance", type=float, default=0.0, help="Distanza massima dal piano; 0 = auto.")
    parser.add_argument("--tangent-padding", type=float, default=0.05, help="Margine laterale per raccogliere supporto.")
    parser.add_argument("--min-support-points", type=int, default=120, help="Minimo numero di punti per confermare un piano.")
    parser.add_argument("--min-supported-area", type=float, default=0.25, help="Area minima supportata dai punti.")
    parser.add_argument("--min-points-per-cell", type=int, default=2, help="Punti minimi per cella semantica.")
    parser.add_argument("--trim-quantile", type=float, default=1.0, help="Percentile usato per scartare outlier laterali.")
    parser.add_argument("--min-polygon-area", type=float, default=0.01, help="Area minima di un poligono dopo il clipping.")

    parser.add_argument("--merge-angle-deg", type=float, default=6.0, help="Angolo massimo per deduplicare patch coincidenti.")
    parser.add_argument("--merge-distance", type=float, default=0.12, help="Distanza massima tra patch coincidenti.")
    parser.add_argument("--duplicate-overlap-ratio", type=float, default=0.35, help="Overlap minimo tra supporti duplicati.")
    parser.add_argument("--parallel-angle-deg", type=float, default=8.0, help="Angolo sotto cui due piani sono paralleli.")
    parser.add_argument("--orthogonal-angle-tol-deg", type=float, default=18.0, help="Tolleranza attorno ai 90 gradi.")
    parser.add_argument("--boundary-minor-fraction", type=float, default=0.18, help="Soglia per decidere il lato da tagliare.")
    parser.add_argument("--min-intersection-length", type=float, default=0.25, help="Lunghezza minima del segmento comune.")
    return parser


if __name__ == "__main__":
    parser = build_parser()
    detect_and_classify_planes(parser.parse_args())
