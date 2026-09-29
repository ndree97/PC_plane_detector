# 🏗️ Plane Detector CLI - Analisi Architetturale e Strutturale 3D

Questo strumento da riga di comando permette di caricare nuvole di punti 3D (supporto nativo per formati `.las`, `.laz`, `.ply`, `.pcd`, `.xyz`) ed estrarre automaticamente i piani geometrici orizzontali e verticali presenti nella scena, ignorando le geometrie curve e il rumore. Sfrutta l'approccio ibrido (RANSAC + Region Growing) di Open3D.

---

## 📦 Installazione

Si consiglia l'uso di un ambiente virtuale (es. `conda` o `venv` con Python >= 3.10):

```bash
git clone https://github.com/ndree97/PC_plane_detector.git
cd PC_plane_detector
pip install -r requirements.txt
```

---

## 🛠️ Guida ai Parametri CLI

L'algoritmo *detect_planar_patches* adatta l'estrazione alla conformazione geometrica e al livello di rumore della scansione:

| Parametro | Descrizione Geometrica | Impatto Visivo |
| :--- | :--- | :--- |
| `--knn` | Numero di punti vicini considerati per calcolare le normali e raggruppare i piani. | **Basso**: cattura dettagli fini ma è vulnerabile al rumore.<br>**Alto**: leviga le superfici, ideale per materiali riflettenti o rumore diffuso. |
| `--variance` | Varianza normale massima (in gradi). Tolleranza di divergenza delle normali all'interno dello stesso piano. | **Basso**: accetta solo superfici perfettamente lisce e complanari.<br>**Alto**: accetta superfici più grezze, rugose o con leggera curvatura. |
| `--coplanarity` | Tolleranza di complanarità (in gradi). Definisce lo "spessore" accettato per un piano. | **Basso**: piani sottili e rigorosi.<br>**Alto**: assorbe il rumore di profondità (es. canaline, tubature a parete). |
| `--outlier` | Percentuale massima di outlier accettata nel volume di delimitazione del piano. | **0.5**: rigoroso.<br>**0.85-0.90**: tollera molti ostacoli/oggetti a contatto con il piano. |
| `--min-edge` | Lunghezza minima del lato più corto del piano (in metri o unità scena). | **0.0**: disattivato (estrae qualsiasi frammento).<br>**1.0**: ignora muri o frammenti larghi meno di 1 metro. |
| `--min-points` | Numero minimo di punti richiesti per validare un piano. | **0**: disattivato.<br>**1000**: ignora superfici poco campionate. |
| `--planes-only`| **(Flag)** Nasconde la nuvola di punti originale e mostra solo le mesh dei piani estratti. | Visione pulita delle geometrie estratte. |

---

## 📐 Caso d'Uso: Scansioni di Ambienti Complessi (Architettonici, Industriali, Navali)

**L'Obiettivo:** Separare i piani orizzontali (pavimenti, solette, ponti) dalle pareti verticali (muri, tramezzi, paratie), ignorando le superfici curve (involucri esterni, volte, tubazioni, scafi).

### 🔴 Il Problema delle Pareti Verticali "Sfuggenti"
Nelle scansioni di interni o strutture dense, i pavimenti (ampi, densi e continui) vengono individuati facilmente, mentre le pareti verticali rischiano di essere frammentate o parzialmente ignorate a causa di:
- **Aperture e porte** che spezzano la continuità geometrica.
- **Canaline, quadri elettrici, arredi o tubazioni** a parete.
- **Minore densità di campionamento** rispetto alle superfici vicine alla stazione di scansione.

### 🎯 Configurazioni Consigliate per Superfici Verticali

A seconda delle caratteristiche della nuvola, è possibile testare tre strategie:

#### 1. Modalità ad Alta Tolleranza (Bassa soglia dimensionale)
Cattura anche pareti corte e tramezzi, assorbendo elementi superficiali a parete (quadri, canaline, tubi):
```bash
python main.py sample_pointcloud.las --knn 50 --variance 45.0 --coplanarity 80.0 --outlier 0.90 --min-edge 0.0 --min-points 50 --planes-only
```
*   **Perché funziona:** Rimuovendo i vincoli minimi di dimensione (`--min-edge 0.0`) e accettando fino al 90% di rumore nel bounding box, nessun frammento verticale viene scartato.

#### 2. Modalità "Pareti Rigide" (Blocco scivolamento su superfici curve)
Da utilizzare quando la scansione include involucri curvi (es. volte, cupole o scafi) per evitare che i piani verticali continuino nelle superfici curve:
```bash
python main.py sample_pointcloud.las --knn 30 --variance 15.0 --coplanarity 70.0 --outlier 0.85 --min-edge 0.5 --min-points 100 --planes-only
```
*   **Perché funziona:** Una varianza ristretta (`--variance 15.0`) interrompe il piano non appena la superficie inizia a curvare. Il parametro `--min-edge 0.5` filtra i micro-frammenti spuri.

#### 3. Modalità "Scansione Diradata" (Punti radi o pareti distanti)
Da utilizzare quando la densità di punti sulle pareti lontane è bassa:
```bash
python main.py sample_pointcloud.las --knn 80 --variance 50.0 --coplanarity 85.0 --outlier 0.80 --min-edge 0.0 --min-points 10 --planes-only
```
*   **Perché funziona:** Un `--knn 80` estende il vicinato per il calcolo della complanarità, connettendo punti più distanziati. `--min-points 10` valida anche superfici poco campionate.

---

## 🔧 Risoluzione dei Problemi Frequenti

*   **Pavimento rilevato, ma mancano i muri?** ➔ Imposta `--min-edge 0.0` e alza `--coplanarity` a `80.0`.
*   **Troppi piccoli piani spuri a mezz'aria?** ➔ Aumenta `--min-edge` a `1.0` o `--min-points` a `500`.
*   **Superfici curve classificate erroneamente come piani verticali?** ➔ Abbassa `--variance` (es. `10.0` - `15.0`).
*   **Troppi frammenti grigi (obliqui)?** ➔ La nuvola potrebbe non essere allineata all'asse Z globale (scanner non perfettamente in bolla o assetto inclinato). Verifica l'orientamento globale prima dell'estrazione.

---

## 🚀 Pipeline Avanzata a Due Passaggi

Per un'estrazione automatica completa con isolamento sequenziale (prima orizzontali, poi verticali), trimming topologico e generazione di report DXF/JSON, consulta la documentazione dettagliata in [detect_planes.md](detect_planes.md):

```bash
python detect_planes.py sample_pointcloud.las --planes-only --hide-oblique
```