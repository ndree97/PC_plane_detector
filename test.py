import open3d as o3d
import numpy as np
import argparse
import os
import laspy

# [La funzione load_point_cloud() rimane invariata...]
def load_point_cloud(file_path):
    ext = os.path.splitext(file_path)[1].lower()
    if ext in ['.las', '.laz']:
        print(f"Caricamento file {ext} tramite laspy in corso...")
        las = laspy.read(file_path)
        coords = np.vstack((las.x, las.y, las.z)).transpose()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
        try:
            r, g, b = las.red / 65535.0, las.green / 65535.0, las.blue / 65535.0
            pcd.colors = o3d.utility.Vector3dVector(np.vstack((r, g, b)).transpose())
        except AttributeError:
            pass 
        return pcd
    else:
        print(f"Caricamento file {ext} tramite Open3D...")
        return o3d.io.read_point_cloud(file_path)

def detect_and_classify_planes(file_path, variance, coplanarity, outlier, knn, min_edge, min_points, planes_only):
    if not os.path.exists(file_path):
        return

    pcd = load_point_cloud(file_path)
    center = pcd.get_center()
    pcd.translate(-center)
    
    print(f"Stima normali (knn={knn})...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    pcd.orient_normals_to_align_with_direction(np.array([0., 0., 1.]))

    # QUI APPLICHIAMO I NUOVI FILTRI DIMENSIONALI
    print("Estrazione delle patch planari...")
    oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=variance,
        coplanarity_deg=coplanarity,
        outlier_ratio=outlier,
        min_plane_edge_length=min_edge,  # <-- Inserito dinamicamente
        min_num_points=min_points,       # <-- Inserito dinamicamente
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn)
    )
    
    print(f"Rilevati {len(oboxes)} piani geometrici.")

    geometries = [] if planes_only else [pcd]
    
    for obox in oboxes:
        min_axis_idx = np.argmin(obox.extent)
        normal = obox.R[:, min_axis_idx]
        verticality = 1.0 - abs(normal[2])
        
        mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.05])
        mesh.compute_vertex_normals()
        
        if verticality < 0.2:
            mesh.paint_uniform_color([0.0, 0.0, 1.0])
        elif verticality > 0.8:
            mesh.paint_uniform_color([1.0, 0.0, 0.0])
        else:
            mesh.paint_uniform_color([0.5, 0.5, 0.5])
            
        geometries.append(mesh)

    o3d.visualization.draw_geometries(geometries)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("input_file", type=str)
    parser.add_argument("--variance", type=float, default=45.0)
    parser.add_argument("--coplanarity", type=float, default=75.0)
    parser.add_argument("--outlier", type=float, default=0.75)
    parser.add_argument("--knn", type=int, default=30)
    
    # NUOVI PARAMETRI DIMENSIONALI
    parser.add_argument("--min-edge", type=float, default=3.0, help="Lunghezza minima del bordo del piano. Usa valori bassi (es. 0.5) per trovare pareti piccole.")
    parser.add_argument("--min-points", type=int, default=1000, help="Numero minimo di punti affinché una superficie sia considerata un piano.")
    
    parser.add_argument("--planes-only", action="store_true")
    
    args = parser.parse_args()
    
    detect_and_classify_planes(
        args.input_file, args.variance, args.coplanarity, 
        args.outlier, args.knn, args.min_edge, args.min_points, args.planes_only
    )
