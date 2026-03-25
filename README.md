# 🚢 Plane Detector CLI - Analisi Architetturale 3D

Questo strumento da riga di comando permette di caricare nuvole di punti (inclusi `.las` e `.laz`) ed estrarre automaticamente i piani geometrici orizzontali e verticali presenti nella scena, ignorando le geometrie curve. Sfrutta l'approccio ibrido (RANSAC + Region Growing) di Open3D.

## 🛠️ Guida ai Parametri CLI

L'algoritmo *detect_planar_patches* è potente, ma la sua vera forza sta nell'adattare i parametri alla fisica della tua scansione.

| Parametro | Descrizione Geometrica | Impatto Visivo |
| :--- | :--- | :--- |
| `--knn` | Numero di punti vicini considerati per calcolare le normali e raggruppare i piani. | **Basso**: cattura dettagli finissimi ma è vulnerabile al rumore.<br>**Alto**: leviga le superfici, ideale per materiali riflettenti. |
| `--variance` | Varianza normale massima (in gradi). Quanto i vettori normali dei punti possono divergere l'uno dall'altro all'interno dello stesso piano. | **Basso**: accetta solo superfici perfettamente lisce.<br>**Alto**: accetta superfici più grezze o curve. |
| `--coplanarity` | Tolleranza di complanarità (in gradi). Definisce lo "spessore" accettato per un piano. | **Basso**: piani sottilissimi e rigorosi.<br>**Alto**: assorbe il rumore di profondità (es. tubature sui muri). |
| `--outlier` | Percentuale massima di rumore accettato all'interno del riquadro di delimitazione del piano. | **0.5**: rigoroso.<br>**0.8**: tollera la presenza di molti oggetti estranei attaccati al piano. |
| `--min-edge` | Lunghezza minima del lato più corto del piano (es. in metri). | **0.0**: disattivato (trova tutto).<br>**1.0**: ignora muri o frammenti larghi meno di 1 metro. |
| `--min-points` | Numero minimo di punti richiesti per validare un piano. | **0**: disattivato.<br>**1000**: ignora le pareti poco campionate. |
| `--planes-only`| **(Flag)** Se richiamato, nasconde la nuvola di punti originale e mostra solo le mesh dei piani estratti per una visione più chiara. | - |

***

## 🌊 Caso d'Uso Reale: Scansione LiDAR di uno Scafo Navale

**L'Obiettivo:** Vogliamo separare i pavimenti/tetti (ponti) dalle pareti verticali (paratie interne), ignorando completamente l'involucro esterno dello scafo (il fasciame) che è curvo.

### 🔴 Il Problema delle Pareti Verticali "Sfuggenti"
Se utilizzi i valori di default, è normale che il pavimento (grande, denso e continuo) venga individuato perfettamente, mentre le pareti verticali vengano parzialmente o totalmente ignorate. Questo accade perché nelle navi le pareti sono **frammentate da porte**, **coperte di quadri elettrici** o **tagliate da tubazioni**. L'algoritmo di Region Growing vede queste interruzioni e "abbandona" il piano, creando frammenti così piccoli che i filtri interni li cestinano come rumore.

### 🎯 Strategie per la Cattura Totale delle Pareti Verticali

Per forzare l'algoritmo a "saltare" gli ostacoli sui muri e a non cestinare i frammenti più piccoli, **prova queste 3 configurazioni sequenziali**, dalla più tollerante alla più rigorosa:

#### 1. Modalità "Aspiratutto" (Nessun vincolo dimensionale, alta tolleranza)
Questa modalità forza l'algoritmo a considerare anche le pareti piccolissime (es. tramezzi tra le cabine) e ad assorbire tubi/cavi dentro il volume della parete stessa.
```bash
python main.py scan_6302_U2_Ponte4.las --knn 50 --variance 45.0 --coplanarity 80.0 --outlier 0.90 --min-edge 0.0 --min-points 50 --planes-only
```
*   **Perché funziona:** Disattivando i limiti di dimensione (`--min-edge 0.0`) e accettando fino al 90% di rumore nel bounding box (`--outlier 0.90`), nessun frammento verticale verrà scartato. Il pavimento rimarrà comunque compatto.

#### 2. Modalità "Muri Rigidi" (Per evitare che i muri "scivolino" nello scafo curvo)
Se la modalità precedente ha colorato di rosso anche le curve dello scafo, devi rendere la ricerca del piano molto più severa sugli angoli, abbassando la varianza, ma mantenendo la complanarità alta (per scavalcare i tubi).
```bash
python main.py scan_6302_U2_Ponte4.las --knn 30 --variance 15.0 --coplanarity 70.0 --outlier 0.85 --min-edge 0.5 --min-points 100 --planes-only
```
*   **Perché funziona:** Una `--variance` a `15.0` spezza immediatamente il muro non appena la superficie inizia a curvarsi (tipico dello scafo). Un `--min-edge` di `0.5` filtra i micro-artefatti creati da questa frammentazione, restituendo solo le vere pareti diritte lunghe almeno mezzo metro.

#### 3. Modalità "Scansione Diradata" (Se le pareti sono lontane dal laser)
Spesso le pareti verticali non vengono rilevate perché la densità di punti sulle superfici verticali lontane è molto più bassa rispetto al pavimento sotto il treppiede dello scanner.
```bash
python main.py scan_6302_U2_Ponte4.las --knn 80 --variance 50.0 --coplanarity 85.0 --outlier 0.80 --min-edge 0.0 --min-points 10 --planes-only
```
*   **Perché funziona:** Un `--knn 80` obbliga l'algoritmo a cercare punti molto più lontani per calcolare la complanarità, riuscendo a "cucire" insieme punti radi su una parete lontana. Il limite `--min-points 10` accetta pareti composte da pochissimi punti laser.

### 🔧 Riepilogo Troubleshooting

*   **Il pavimento è perfetto, ma mancano i muri?** ➔ Imposta `--min-edge 0.0` e alza `--coplanarity` a `80.0`.
*   **Trova troppi piccoli quadratini falsi a mezz'aria?** ➔ Aumenta `--min-edge` a `1.0` o `--min-points` a `500`.
*   **Colora di rosso lo scafo curvo?** ➔ Abbassa drasticamente la `--variance` (es. `10.0` o `15.0`).
*   **Troppi frammenti grigi (obliqui)?** ➔ La nave potrebbe non essere perfettamente allineata all'asse Z (il mare la inclina o lo scanner non era in bolla). Assicurati che la nuvola sia stata allineata all'orizzonte globale prima dell'export in `.laz`.