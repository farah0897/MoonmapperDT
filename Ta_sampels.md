# MoonMapper – Ta Triad-samples (oppdatert metode)

Denne filen beskriver **dagens** innsamlingsrutine: **kun Triad (AS7265X)** over seriell til Arduino, rådata i `ml/datasets/raw/`, metadata i `ml/datasets/metadata.csv`. **Mikroskop, USB-kamera og YOLO er fjernet** fra hovedløpet.

---

## 1) Forutsetninger


| Sjekk   | Detalj                                                                          |
| ------- | ------------------------------------------------------------------------------- |
| Katalog | Du står i workspace-root: `~/rover/simulasjon/moonmapper_ws`                    |
| Python  | Virtuelt miljø: `source .venv-ml/bin/activate`                                  |
| Pakker  | `pip install -r requirements-ml.txt` (hvis nødvendig) — trengs bl.a. `pyserial` |
| Arduino | `moonmapper_triad_logger` er flashed og svarer på `SB` / `BURST`                |
| Port    | Ofte `/dev/ttyACM0` — sjekk med `ls /dev/ttyACM`*                               |


---

## 2) Hva hver innsamling gjør (anbefalt med sandbakgrunn)

Ved `**--sample-background`** kjører du først **SB** (bakgrunn på ren sand). **Etterpå** kan du velge *hvordan* du vil pause før **BURST** — det er **to forskjellige flagg**:


| Flag                                                | Hva det gjør                                                                                                                                                         |
| --------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `**--vent-etter-sb SEC`** (alias `--wait-after-sb`) | **Automatisk** pause i SEC sekunder. Ingen Enter. Bruk når du vil ha fast tid til å legge objekt uten å måtte trykke tastatur.                                       |
| `**--interactive-after-background`**                | **Manuell** pause: skriptet venter til du trykker **Enter**. Ingen timer (med mindre du også setter `--vent-etter-sb`; da kjøres først timeren, **deretter** Enter). |


Velg **enten** timer **eller** Enter om du vil ha én enkel flyt; begge samtidig gir lengst pause (først sekunder, så bekreftelse).

Deretter: `BURST <sample_id>` → 20 rader til `ml/datasets/raw/<sample_id>_triad_raw.csv`. Med `--append-metadata` legges rad i `ml/datasets/metadata.csv`.

**Rekkefølge i skriptet:** SB → 0,2 s → evt. **SEC sekunder** → evt. **Enter** → BURST.

Dette reduserer problemer hvis port-åpning får Arduino til å miste siste SB.

---

## 3) Metadata (felt du fyller)

Kolonner i `ml/datasets/metadata.csv` (se header i fila):

- **Påkrevd logisk:** `sample_id`, `label_object`, `label_material`, `triad_file`  
- **Sand / underlag:** `sand_type` (f.eks. `torr_sand`, `fuktig_sand`) — settes med `--sand-type` ved innsamling  
- **Lysforhold:** `lysforhold` — bruk `**lampelys`** eller `**mørk`** (som i oppsettet under) — `--lysforhold`  
- **Avstand:** `avstand_cm` — `**4.0`** eller `**4.8`** (cm mellom sensor og objekt) — `--avstand-cm`  
- **Vinkel:** `angle_id` — `**A30`** eller `**A90`** (30° / 90°) — `--angle-id`  
- **Rutefelt (valgfritt):** `position_id` brukes ikke i det faste 104-oppsettet; la stå default eller tom om du vil  
- **Overflate:** `surface_condition` — `--surface-condition` eller `--surface_condition`  
- **Grader av nedgravning:** `buried_level` — `--buried-level` eller `--buried_level`  
- **Øvrig kontekst:** `diameter_mm`, `run_id`, `size_group` etter behov

For **104 samples – stål kule 4,5 mm** (se §5):

- `label_object` = `stål_kule` (samme streng i hele batch-en)  
- `label_material` = f.eks. `steel` — vær konsekvent  
- `diameter_mm` = `4.5`

---

## 4) Kommando-mal (én sample)

Bytt ut port og sample-id.

**Anbefalt pause etter SB — kun tidsstyring** (`--vent-etter-sb`, ikke Enter):

```bash
cd ~/rover/simulasjon/moonmapper_ws
source .venv-ml/bin/activate

python3 scripts/collect_triad_data.py \
  --port /dev/ttyACM0 \
  --sample-id S0001 \
  --sample-background \
  --vent-etter-sb 30 \
  --append-metadata \
  --label-object stål_kule \
  --label-material steel \
  --sand-type torr_sand \
  --lysforhold lampelys \
  --avstand-cm 4.0 \
  --angle-id A30 \
  --diameter-mm 4.5 \
  --surface-condition synlig \
  --buried-level 0 \
  --run-id R01
```

**Pause etter SB — kun Enter** (ingen timer; du starter BURST når du trykker Enter):

```bash
python3 scripts/collect_triad_data.py \
  --port /dev/ttyACM0 \
  --sample-id S0006 \
  --sample-background \
  --interactive-after-background \
  --append-metadata \
  --label-object stål_kule \
  --label-material steel \
  --sand-type torr_sand \
  --lysforhold lampelys \
  --avstand-cm 4.8 \
  --angle-id A30 \
  --diameter-mm 4.5 \
  --surface-condition synlig \
  --buried-level 0 \
  --run-id R06
```

**Alternativ** uten `--sample-background` (kun BURST, ingen SB — ingen pause etter SB):

```bash
python3 scripts/collect_triad_data.py \
  --port /dev/ttyACM0 \
  --sample-id S0001 \
  --append-metadata \
  --label-object stål_kule \
  --label-material steel \
  --sand-type torr_sand \
  --lysforhold mørk \
  --avstand-cm 4.8 \
  --angle-id A90 \
  --diameter-mm 4.5
```

- Nye råfiler: `ml/datasets/raw/S0001_triad_raw.csv`  
- Nye rader: appendes i `ml/datasets/metadata.csv` med `triad_file` satt til f.eks. `raw/S0001_triad_raw.csv`  
- Hvis fila finnes fra før: legg til `--overwrite` (sletter/overskriver råfilen for det `sample_id`)

**Direkte (uten wrapper):**  
`python3 ml/data_collection/collect_triad_burst.py` med de samme flaggene (samme defaults under `ml/datasets/`).

---

## 5) Plan: 104 samples – `stål_kule`, 4,5 mm (konkret oppsett)

**Faktorer:** tre binære valg gir **2 × 2 × 2 = 8** unike kombinasjoner.


| Faktor                 | Verdier                 |
| ---------------------- | ----------------------- |
| Lys (`lysforhold`)     | `lampelys`, `mørk`      |
| Avstand (`avstand_cm`) | `4.0`, `4.8` (cm)       |
| Vinkel (`angle_id`)    | `A30`, `A90` (30°, 90°) |


**Totalt:** **8 × 13 = 104** samples (13 repetisjoner per kombinasjon).

### Konkret oppsett (alle kombinasjoner)


| `lysforhold` | `avstand_cm` | `angle_id` | Antall | diameter | sjekk  |
| ------------ | ------------ | ---------- | ------ | -------- |--------
| lampelys     | 12           | A30        | 13     | 4.5mm    |
| lampelys     | 8            | A30        | 13     | 4.5mm    |
| lampelys     | 12           | A30        | 13     | 11mm     |
| lampelys     | 8            | A30        | 13     | 11mm     |
| lampelys     | 4.0          | A90        | 13     |          |
| mørk         | 4.5          | A30        | 13     |          |
| mørk         | 4.0          | A90        | 13     |          |
| mørk         | 4.8          | A30        | 13     |          |
| mørk         | 4.8          | A90        | 13     |          |


### Forslag til `sample_id`-rekkefølge (S0001–S0104)

En enkel blokkering: første kombinasjon = `S0001`–`S0013`, neste = `S0014`–`S0026`, osv. til `S0104`:


| Blokk | `sample_id` | `lysforhold` | `avstand_cm` | `angle_id` |
| ----- | ----------- | ------------ | ------------ | ---------- |
| 1     | S0001–S0013 | lampelys     | 12          | A30         |
| 2     | S0014–S0026 | lampelys     | 8          | A30          |
| 3     | S0027–S0039 | lampelys     | 12          | A30         |
| 4     | S0040–S0052 | lampelys     | 8          | A30          |
| 5     | S0053–S0065 | mørk         | 4.0          | A30        |
| 6     | S0066–S0078 | mørk         | 4.0          | A90        |
| 7     | S0079–S0091 | mørk         | 4.8          | A30        |
| 8     | S0092–S0104 | mørk         | 4.8          | A90        |


Du kan også ta kombinasjonene i annen rekkefølge — det viktige er at **metadata matcher** hvert opptak.

**Rutine per sample**

1. Still inn **lys** (lampelys vs mørk), **avstand** (4,0 cm vs 4,8 cm) og **vinkel** (30° vs 90°) som i tabellen.
2. Kjør innsamling med matchende `--lysforhold`, `--avstand-cm`, `--angle-id`.

Eksempel (**mørk**, 4,8 cm, 90°) for **S0100**:

```bash
python3 scripts/collect_triad_data.py \
  --port /dev/ttyACM0 \
  --sample-id S0002 \
  --sample-background \
  --append-metadata \
  --label-object stål_kule \
  --label-material steel \
  --sand-type regolith \
  --lysforhold lampelys \
  --avstand-cm 4.8 \
  --angle-id A90 \
  --diameter-mm 4.5 \
  --run-id R2
```

Bruk **unike** `sample_id` for alle 104 (tilpass startnummer hvis du allerede har tatt samples).

---

## 6) Sammenligne to råfiler og forstå **S** vs **N**

Råfilen har per rad **S0/S1** (korrigert spektrum per sensor) og **N0/N1** (normalisert «form»). Se `moonmapper_triad_logger.ino` for presis definisjon. Kort fortalt:


| Kolonner       | Hva det er                                                                                         | Typisk bruk                                              |
| -------------- | -------------------------------------------------------------------------------------------------- | -------------------------------------------------------- |
| **S0_*, S1_*** | Korrigert signal (LED av/på, minus sandbakgrunn fra **SB**). Kan være negative.                    | Absolutt nivå + form; følsomt for lysstyrke og avstand.  |
| **N0_*, N1_*** | Hver kanal delt på **summen av positive** S-verdier for *samme sensor*; kanaler ≤ 0 i S gir 0 i N. | Sammenligne **kurveform** mer uavhengig av total styrke. |


**Hvorfor kan N1 (eller N0) være helt flatt på null?** Koden bruker bare **positive** S-verdier i nevneren. Hvis *alle* korrigerte verdier for den sensoren er ≤ 0 (ingenting «over» sandbakgrunnen), blir **hele** N-kurven 0. Det betyr ikke nødvendigvis at sensoren er «død», men at **geometri, skygge eller at kulen/sanden dominerer annerledes** i den målingen — ofte ser du da også svak eller negativ S på den sensoren.

**Tolke sammenligning (to CSV-filer):**

- **Samme materiale, ulik posisjon** kan gi **veldig ulike** kurver (særlig S1) — det er **målevariasjon** (vinkel mot kule, sand, to sensorer), ikke nødvendigvis feil.
- **Ulike materialer** under like betingelser: forskjellig **S- og N-form** tyder på **ulik spektral signatur**; vær like forsiktig hvis **kulens størrelse** også endrer seg (skygge/areal), da kan noe av forskjellen være **geometri**, ikke bare materiale.

**Skript:** `scripts/compare_triad_spectra.py` — plotter to filer (standard: **gjennomsnitt** over hele burst).

Kun korrigert S:

```bash
cd ~/rover/simulasjon/moonmapper_ws
source .venv-ml/bin/activate

python3 scripts/compare_triad_spectra.py \
  ml/datasets/raw/S0001_triad_raw.csv \
  ml/datasets/raw/S0002_triad_raw.csv \
  --label-a "Stål1 4.5 mm" \
  --label-b "stål2 4.5 mm" \
  --out ml/datasets/processed/compare_staal_vs_staal.png
```

Kun normalisert N:

```bash
python3 scripts/compare_triad_spectra.py \
  ml/datasets/raw/S0001_triad_raw.csv \
  ml/datasets/raw/S0002_triad_raw.csv \
  --columns N \
  --label-a "Stål" \
  --label-b "Ukjent" \
  --out ml/datasets/processed/compare_N.png
```

**S og N i samme figur** (2×2: S0/S1 øverst, N0/N1 nederst):

```bash
python3 scripts/compare_triad_spectra.py \
  ml/datasets/raw/S0001_triad_raw.csv \
  ml/datasets/raw/S0002_triad_raw.csv \
  --label-a "Stål1 4.5 mm" \
  --label-b "stål2 4.5 mm" \
  --both \
  --out ml/datasets/processed/compare_staal_vs_staal_S_og_N.png
```

**Eksempelplot** (generert med `--both`, gjennomsnitt over burst — `mean` i tittel):

*Samme stålkule 4,5 mm, to målinger (ulik posisjon, samme vinkel/høyde):*

Sammenligning stål vs stål: S0/S1 og N0/N1

*Stålkule 4,5 mm vs ukjent kule (~1,1 cm), samme posisjon/høyde:*

Sammenligning stål vs ukjent kule: S0/S1 og N0/N1

---

## 7) Etter innsamling (kort)

Når alle 104 rader ligger i metadata og råfilene i `ml/datasets/raw/`:

```bash
python3 scripts/extract_features.py
python3 scripts/split_dataset.py
python3 scripts/train_random_forest.py
python3 scripts/evaluate_model.py
```

(Forutsetter at `triad_file` i metadata peker på eksisterende filer under `ml/datasets/`.)

---

## 8) Feilsøking (kort)


| Problem                          | Tiltak                                                                                                 |
| -------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `No module named 'ml'`           | Kjør fra **repo-root** (`moonmapper_ws`), ikke fra `ml/`.                                              |
| Ingen svar fra Arduino           | Sjekk port, baud, at riktig sketch er flashed.                                                         |
| `raw file already exists`        | Bruk annet `sample-id` eller `--overwrite`.                                                            |
| Tom `metadata.csv` utenom header | Første rad: bruk `collect_triad_data.py` med `--append-metadata` (ikke rediger manuelt feil kolonner). |


---

*Sist oppdatert: Triad-only pipeline; kap. 6 om S/N og sammenligning av råfiler.*