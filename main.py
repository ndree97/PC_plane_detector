import open3d as o3d
import numpy as np
import argparse
import os
import laspy

def load_point_cloud(file_path):
    """
    Funzione robusta per caricare nuvole di punti in vari formati.
    Gestisce i file .las/.laz tramite laspy come suggerito nel Capitolo 4.
    """
    ext = os.path.splitext(file_path)[1].lower()
    
    if ext in ['.las', '.laz']:
        print(f"Caricamento file {ext} tramite laspy in corso...")
        las = laspy.read(file_path)
        
        # Estrazione delle coordinate e trasformazione per l'oggetto Open3D
        coords = np.vstack((las.x, las.y, las.z)).transpose()
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
        
        # Estrazione opzionale del colore (normalizzato per Open3D)
        try:
            r = las.red / 65535.0
            g = las.green / 65535.0
            b = las.blue / 65535.0
            colors = np.vstack((r, g, b)).transpose()
            pcd.colors = o3d.utility.Vector3dVector(colors)
        except AttributeError:
            pass # Nuvola senza dati RGB
            
        return pcd
    else:
        # Per .ply, .pcd, .xyz
        print(f"Caricamento file {ext} tramite Open3D...")
        return o3d.io.read_point_cloud(file_path)

def detect_and_classify_planes(file_path, variance, coplanarity, outlier, knn):
    if not os.path.exists(file_path):
        print(f"Errore: Il file {file_path} non esiste.")
        return

    # Step 1: Caricamento e Pre-elaborazione
    pcd = load_point_cloud(file_path)
    
    # Pulizia spaziale: Centratura della nuvola
    print("Centratura della nuvola di punti...")
    center = pcd.get_center()
    pcd.translate(-center)
    
    # Stima delle normali ibrida
    print(f"Stima delle normali (ricerca ibrida con knn={knn})...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    pcd.orient_normals_to_align_with_direction(np.array([0., 0., 1.]))

    # Step 2: Estrazione Automatica dei Piani (RANSAC + Region Growing)
    print("Estrazione delle patch planari...")
    oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=variance,
        coplanarity_deg=coplanarity,
        outlier_ratio=outlier,
        min_plane_edge_length=0.0,
        min_num_points=0,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn)
    )
    
    print(f"Rilevati {len(oboxes)} piani geometrici.")

    # Step 3 & 4: Classificazione Geometrica e Generazione Mesh
    geometries = [pcd]
    
    for obox in oboxes:
        min_axis_idx = np.argmin(obox.extent)
        normal = obox.R[:, min_axis_idx]
        
        # Calcolo verticalità
        verticality = 1.0 - abs(normal[2])
        
        # Solidificazione
        mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.05])
        mesh.compute_vertex_normals()
        
        if verticality < 0.2:
            # Piani Orizzontali -> Blu
            mesh.paint_uniform_color([0.0, 0.0, 1.0])
        elif verticality > 0.8:
            # Piani Verticali -> Rosso
            mesh.paint_uniform_color([1.0, 0.0, 0.0])
        else:
            # Superfici oblique -> Grigio
            mesh.paint_uniform_color([0.5, 0.5, 0.5])
            
        geometries.append(mesh)

    print("Inizializzazione visualizzazione...")
    o3d.visualization.draw_geometries(geometries, window_name="Identificazione Superfici Architettoniche 3D")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Analisi multimodale 3D per trovare e classificare piani orizzontali/verticali.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # Parametro obbligatorio
    parser.add_argument("input_file", type=str, help="Percorso del file (supporta .las, .laz, .ply, .pcd, .xyz)")
    
    # Parametri adattivi CLI
    parser.add_argument("--variance", type=float, default=45.0, help="Varianza normale (gradi)")
    parser.add_argument("--coplanarity", type=float, default=75.0, help="Tolleranza di complanarità (gradi)")
    parser.add_argument("--outlier", type=float, default=0.75, help="Rapporto massimo di outlier (0-1)")
    parser.add_argument("--knn", type=int, default=30, help="Numero di K-vicini per stima normali e ricerca k-d tree")
    
    args = parser.parse_args()
    
    detect_and_classify_planes(
        args.input_file, 
        args.variance, 
        args.coplanarity, 
        args.outlier, 
        args.knn
    )
