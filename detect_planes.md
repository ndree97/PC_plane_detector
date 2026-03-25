# 🚢 Plane Detector CLI - Pipeline Iterativa a 2 Passi

Questo strumento da riga di comando è progettato per estrarre automaticamente i piani architettonici (pavimenti, ponti, paratie, muri) da nuvole di punti 3D complesse (come scansioni navali `.las` o `.laz`). 

La versione attuale adotta una **soluzione iterativa a due passaggi**, questo approccio risolve il classico paradosso della segmentazione 3D: i parametri perfetti per trovare i pavimenti distruggono le pareti, e viceversa.

***

## 🧠 Il Problema e la Soluzione

Nelle scansioni di ambienti complessi come le navi:
1. **I Pavimenti (Ponti):** Sono grandi e continui, ma pieni di "rumore" (oggetti appoggiati, macchinari). Richiedono un algoritmo molto tollerante e permissivo.
2. **Le Pareti (Paratie):** Sono spesso vicine a superfici curve (lo scafo esterno) e tagliate da tubature. Se si usa un algoritmo tollerante, il muro "scivola" e ingloba la curva dello scafo. Se si usa un algoritmo rigido, il muro si spezza troppo.

**La Soluzione:** Lo script ora esegue la funzione `detect_planar_patches` in **due passaggi sequenziali**, isolando geometricamente le due entità con parametri diametralmente opposti.

### 🔹 Passaggio 1: "Modalità Aspiratutto" (Pavimenti e Tetti)
Il programma cerca prima di tutto le superfici orizzontali enormi (lato minimo **3 metri**).
* **Varianza alta (45°)** e **Outlier tollerante (90%)**: Permette all'algoritmo di assorbire ostacoli a terra (macchinari, gradini) senza spezzare il ponte principale.
* Le geometrie estratte vengono filtrate per verticalità (`< 0.2`) e colorate di **Blu**.

### 🔸 Passaggio 2: "Modalità Muri Rigidi" (Pareti Verticali)
Successivamente, il programma cerca le superfici verticali (lato minimo abbassato a **1.5 metri** per includere tramezzi e cabine).
* **Varianza bassa (15°)**: Fondamentale per far sì che la ricerca del piano si fermi immediatamente appena tocca la parete curva dello scafo navale.
* **Tolleranza di verticalità allargata (`> 0.6`)**: Molte pareti scannerizzate potrebbero non essere perfettamente a piombo (o lo scanner non era in bolla). Invece di chiedere un muro perfetto al 100%, accettiamo pareti leggermente storte.
* Le geometrie vengono colorate di **Rosso**.

***

## 🛠️ Utilizzo della CLI

Abbiamo semplificato l'interfaccia. Essendo i parametri geometrici ottimizzati e codificati nei due passaggi (Passaggio 1 per l'orizzontale, Passaggio 2 per il verticale), da terminale devi solo gestire la visualizzazione e la risoluzione.

### Comando Base (Consigliato)
Questo è il comando perfetto per ottenere una vista pulita delle sole strutture architettoniche, ignorando lo scafo esterno e gli artefatti geometrici:

```bash
python detect_planes.py scan_6302_U2_Ponte4.las --planes-only --hide-oblique
```

### Parametri Disponibili

| Parametro CLI | Descrizione |
| :--- | :--- |
| `input_file` | **Obbligatorio.** Il percorso al tuo file `.las`, `.laz`, `.ply` o `.pcd`. |
| `--knn` | *(Default: 30)* Numero di punti vicini considerati per il calcolo delle normali. Aumentalo (es. `50`) se la nuvola è molto densa o rumorosa, per "lisciare" le superfici. |
| `--planes-only`| **Flag.** Se inserito, non carica la nuvola di punti originale nel visualizzatore, ma mostra esclusivamente le maglie (mesh) dei piani geometrici trovati. Ideale per la pulizia visiva. |
| `--hide-oblique`| **Flag.** Se inserito, nasconde tutti i piani classificati come **Grigi** (quelli la cui inclinazione è compresa tra 0.2 e 0.6, spesso artefatti, tubi diagonali o scale). |

***

## 🎨 Legenda Colori nel Visualizzatore

*   🟦 **Blu:** Superfici orizzontali (Pavimenti, Ponti, Soffitti).
*   🟥 **Rosso:** Superfici verticali o sub-verticali (Paratie, Muri).
*   ⬜ **Grigio:** *(Visibili solo se non si usa `--hide-oblique`)* Superfici oblique, come rampe di scale, tubazioni trasversali o artefatti del rumore.

## 🔧 Personalizzazione Avanzata (Nel Codice)

Se hai bisogno di adattare l'algoritmo a una nave con misure diverse, puoi aprire `plane_detector_cli.py` e modificare i parametri cablati nelle due chiamate a `detect_planar_patches()`:

* **Pareti troppo corte ignorate?** Nel "Passaggio 2", abbassa `min_plane_edge_length` da `1.5` a `0.8`.
* **Troppi frammentini sul pavimento?** Nel "Passaggio 1", alza `min_plane_edge_length` a `5.0`.
* **Scafo curvo ancora colorato di rosso?** Nel "Passaggio 2", abbassa ulteriormente `normal_variance_threshold_deg` da `15.0` a `10.0`.