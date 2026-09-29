# 🏢 Plane Detector CLI - Pipeline Iterativa a 2 Passi

Questo strumento da riga di comando è progettato per estrarre automaticamente i piani architettonici e strutturali (pavimenti, solette, soffitti, paratie, muri) da nuvole di punti 3D complesse (scansioni `.las`, `.laz`, `.ply`, `.pcd`). 

La versione adotta una **soluzione iterativa a due passaggi**, risolvendo la tipica sfida della segmentazione 3D: i parametri ideali per catturare grandi superfici orizzontali tendono a compromettere la rilevazione delle pareti verticali, e viceversa.

---

## 🧠 Il Problema e la Soluzione

Nelle scansioni di ambienti complessi e densi di elementi (edifici civili, impianti industriali, infrastrutture o strutture navali):
1. **Piani Orizzontali (Pavimenti, Solette, Ponti):** Sono estesi e continui, ma spesso ricchi di ostacoli (arredi, macchinari, elementi appoggiati a terra). Richiedono una stima altamente tollerante e permissiva.
2. **Piani Verticali (Muri, Tramezzi, Paratie):** Sono spesso prossimi a superfici curve (volte, condotti, involucri curvi o scafi) e interrotti da porte, finestre e tubazioni. Un algoritmo troppo tollerante rischia di "scivolare" sulle curve contigue; uno troppo rigido frammenta eccessivamente la parete.

**La Soluzione:** La pipeline esegue l'estrazione in **due passaggi sequenziali**, isolando geometricamente le due tipologie con parametri dedicati.

### 🔹 Passaggio 1: Rilevazione Piani Orizzontali (Pavimenti e Tetti)
Ricerca iniziale delle superfici orizzontali estese (lato minimo **3 metri**).
* **Varianza normale (45°)** e **Outlier tollerante (90%)**: Permette all'algoritmo di assorbire dislivelli e ingombri a terra senza spezzare la superficie principale.
* Le geometrie estratte vengono filtrate per verticalità (`< 0.2`) e colorate in **Blu**.

### 🔸 Passaggio 2: Rilevazione Pareti Verticali
Ricerca successiva delle superfici verticali (lato minimo ridotto a **1.5 metri** per includere tramezzi e partizioni interne).
* **Varianza normale contenuta (15°)**: Interrompe l'estrazione non appena la superficie devia o curva.
* **Tolleranza di verticalità estesa (`> 0.6`)**: Riconosce pareti leggermente inclinate o scansioni con lieve disallineamento rispetto al piombo globale.
* Le geometrie estratte vengono colorate in **Rosso**.

---

## 🛠️ Utilizzo della CLI

I parametri geometrici chiave sono pre-ottimizzati all'interno dei due passaggi. Da terminale è sufficiente specificare il file di input e le opzioni di visualizzazione ed esportazione:

### Comando Base Consigliato
Permette di ottenere una visualizzazione pulita delle sole strutture estratte, escludendo elementi obliqui o artefatti:

```bash
python detect_planes.py sample_pointcloud.las --planes-only --hide-oblique
```

### Parametri Principali

| Parametro CLI | Descrizione |
| :--- | :--- |
| `input_file` | **Obbligatorio.** Percorso al file `.las`, `.laz`, `.ply` o `.pcd`. |
| `--knn` | *(Default: 30)* Vicini usati per la stima delle normali. Aumentare (es. `50`) per nuvole rumorose per levigare le superfici. |
| `--voxel-size` | *(Default: 0.05)* Dimensione del voxel per il downsampling preliminare. Impostare a `0` per disattivarlo. |
| `--planes-only` | **Flag.** Nasconde la nuvola di punti originale e mostra esclusivamente le mesh dei piani geometrici estratti. |
| `--hide-oblique` | **Flag.** Nasconde i piani classificati come **Grigi** (inclinazione compresa tra 0.2 e 0.6, tipici di elementi trasversali o rumore). |
| `--report-dir` | *(Default: "report")* Directory in cui salvare i report JSON e i file CAD/DXF generati. |
| `--cad-export` | *(Default: "dxf")* Esporta le facce trimmate in formato CAD (DXF). Impostare a `off` per disattivare. |
| `--no-visualization` | **Flag.** Esegue l'estrazione e l'esportazione dei report in modalità headless senza aprire il visualizzatore. |

---

## 🎨 Legenda Colori nel Visualizzatore

* 🟦 **Blu:** Superfici orizzontali (Pavimenti, Solette, Soffitti).
* 🟥 **Rosso:** Superfici verticali o sub-verticali (Pareti, Tramezzi, Paratie).
* ⬜ **Grigio:** *(Visibili solo senza `--hide-oblique`)* Superfici oblique (rampe, scale, canalizzazioni diagonali).

---

## 🔧 Personalizzazione dei Parametri nel Codice

Se necessario, i parametri di soglia per ciascun passaggio possono essere personalizzati direttamente tramite argomenti CLI o all'interno di `detect_planes.py`:

* **Pareti corte o frammentate non rilevate?** Riduci `--vertical-min-edge` da `1.5` a `0.8`.
* **Frammenti spuri a terra?** Aumenta `--horizontal-min-edge` a `4.0` o `5.0`.
* **Bordi curvi inglobati nelle pareti?** Riduci la varianza angolare delle pareti verticali nel codice (es. da `15.0` a `10.0`).