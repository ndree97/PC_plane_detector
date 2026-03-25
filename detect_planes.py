import open3d as o3d
import numpy as np
import argparse
import os
import laspy

def load_point_cloud(file_path):
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

def detect_and_classify_planes(file_path, knn, planes_only, hide_oblique):
    """Esegue l'estrazione iterativa dei piani: prima pavimenti, poi pareti."""
    if not os.path.exists(file_path):
        print(f"Errore: Il file {file_path} non esiste.")
        return

    # --- Pre-elaborazione ---
    pcd = load_point_cloud(file_path)
    
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Estrazione iterativa di pavimenti e pareti da nuvole di punti.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument("input_file", type=str, help="Percorso del file della nuvola di punti (.las, .laz, .ply, .pcd).")
    parser.add_argument("--knn", type=int, default=30, help="Punti vicini considerati per il calcolo delle normali.")
    parser.add_argument("--planes-only", action="store_true", help="Nasconde la nuvola di punti originale e mostra solo i piani 3D.")
    parser.add_argument("--hide-oblique", action="store_true", help="Nasconde completamente i piani grigi (piani non perfettamente orizzontali né verticali).")
    
    args = parser.parse_args()
    
    detect_and_classify_planes(
        args.input_file, 
        args.knn, 
        args.planes_only, 
        args.hide_oblique
    )
