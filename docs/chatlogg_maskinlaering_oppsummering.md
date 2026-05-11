# Chatlogg og oppsummering: Maskinlæring i Moonmapper

Denne filen oppsummerer arbeidet og forklaringene fra chatten om maskinlæringsdelen i Moonmapper-prosjektet. Den er skrevet som en ryddig dokumentasjonsfil, slik at den kan brukes som støtte til rapportskriving, muntlig forklaring og videre utvikling.

## 1. Overordnet mål

Målet med maskinlæringsdelen er å bruke spektraldata fra SparkFun AS7265X Triad-spektrometre til å identifisere materialer eller metallobjekter. Sensorene måler refleksjon ved flere bølgelengder, og disse målingene brukes til å lage features som en maskinlæringsmodell kan klassifisere.

Prosjektet bruker foreløpig en Random Forest-modell. Modellen trenes på prosesserte features fra Triad-data og kan senere brukes til live-prediksjon av nye målinger.

## 2. Arduino og Python har ulike roller

Arduino-koden og Python-koden gjør ikke samme jobb, selv om begge inngår i datainnsamlingen.

`arduino/moonmapper_triad_logger/moonmapper_triad_logger.ino` kjører på Arduino/mikrokontrolleren. Den håndterer den fysiske sensormålingen:

- Velger riktig Triad-sensor via Qwiic I2C-multiplekser.
- Setter gain og integrasjonstid.
- Måler med intern LED av og på.
- Beregner `LED_ON - LED_OFF`.
- Trekker fra bakgrunnsmåling hvis den finnes.
- Skriver CSV-formatert tekst ut via Serial/USB.

`ml/data_collection/collect_triad_burst.py` kjører på PC/WSL. Den håndterer datalogging:

- Åpner serial-porten, for eksempel `/dev/ttyACM0`.
- Sender kommandoer til Arduino, for eksempel `SB` og `BURST <sample_id>`.
- Leser CSV-linjene Arduino sender tilbake.
- Lagrer målingene som `.csv`-fil på PC.
- Kan oppdatere `metadata.csv`.

Arduino lagrer altså ikke CSV-filen permanent. Den sender CSV-tekst via Serial/USB. Python-scriptet mottar teksten og lagrer filen, vanligvis som:

```text
ml/datasets/raw/<sample_id>_triad_raw.csv
```

Ved live-test kan filen lagres som:

```text
ml/datasets/live/<sample_id>_triad_raw.csv
```

## 3. Målekonfigurasjon for Triad

Triad-sensorene er konfigurert med:

- Gain: `64x`
- Integrasjonstid: 18 sykluser, omtrent 50 ms per måling
- To sensorer
- 18 spektralkanaler per sensor
- Totalt 36 spektralkanaler per måling
- Bølgelengder fra 410 nm til 940 nm

Gain brukes for å øke sensorfølsomheten. Integrasjonstid bestemmer hvor lenge sensoren samler lys før en måling leses ut. Verdiene er valgt for å få et hensiktsmessig signal-støy-forhold uten at måletiden blir unødvendig lang.

## 4. Kontrollert belysning

Spektralmålinger påvirkes av omgivelseslys, skygger og sensoroffset. Derfor brukes en differansemetode med sensorens interne LED.

Først måles signalet med LED av:

```text
LED_OFF
```

Deretter måles signalet med LED på:

```text
LED_ON
```

Differansen beregnes slik:

```latex
\Delta_i = LED_{ON,i} - LED_{OFF,i}
```

Målingen med LED av inneholder hovedsakelig omgivelseslys og sensoroffset. Målingen med LED på inneholder dette pluss refleksjon fra objektet. Ved å trekke dem fra hverandre reduseres påvirkning fra omgivelseslys.

## 5. Bakgrunnskompensasjon

Før objektmåling tas en bakgrunnsmåling uten objekt, for eksempel bare sand. Denne bakgrunnsmålingen trekkes fra objektmålingen:

```latex
x_{korrigert,i} = \Delta_i - b_i
```

Her er:

- `i` en spektralkanal
- `\Delta_i` LED-korrigert måling med objekt
- `b_i` bakgrunnsverdi for samme kanal
- `x_{korrigert,i}` bakgrunnskorrigert kanalverdi

Dette gjøres fordi sensoren ikke bare ser objektet. Den kan også se sand, underlag og refleksjon rundt objektet. Bakgrunnskompensasjon gjør at modellen i større grad lærer objektets signal, ikke underlagets signal.

## 6. Normalisering

Normalisering brukes for å redusere påvirkning fra total lysintensitet. To målinger kan ha samme spektrale form, men ulik styrke fordi sensoren er litt nærmere objektet, vinkelen er annerledes eller refleksjonen er sterkere.

Normalisering kan uttrykkes slik:

```latex
n_i = \frac{x_i}{\sum_{k=1}^{N} x_k}
```

Dette gjør at modellen i større grad lærer forholdet mellom bølgelengdene, altså spektral form, i stedet for bare absolutt signalstyrke.

Kort forklart:

```text
Uten normalisering: modellen kan lære lysstyrke.
Med normalisering: modellen lærer spektral form.
```

## 7. Datastruktur for maskinlæring

Etter at Arduino har sendt dataene via Serial/USB, mottar `collect_triad_burst.py` CSV-linjene og lagrer dem på PC. Datastrukturen består av to hoveddeler:

1. Rådatafiler
2. Metadatafil

Rådatafilene inneholder spektralmålingene. Metadatafilen beskriver hva målingen representerer.

Eksempel på rådatafil:

```text
ml/datasets/raw/S0001_triad_raw.csv
```

Eksempel på metadata:

```text
sample_id,label_object,label_material,triad_file,size_group,...
S0001,stål_kule,stål,raw/S0001_triad_raw.csv,liten,...
```

Denne separasjonen gjør at råmålingene kan beholdes uendret, mens metadata og features kan forbedres senere.

Dataflyten er:

```text
Triad-sensorer
  -> Arduino
  -> Serial/USB CSV-strøm
  -> collect_triad_burst.py
  -> rå CSV-fil
  -> metadata.csv
  -> feature-ekstraksjon
  -> prosessert feature-tabell
  -> modelltrening
```

## 8. Feature-ekstraksjon

Feature-ekstraksjonen skjer i `ml/training/extract_triad_features.py`. Målet er å gjøre råmålinger om til numeriske verdier som modellen kan lære av.

En burst kan beskrives som:

```latex
x_{t,i}
```

der:

- `t` er måling nummer i burst-sekvensen
- `i` er spektralkanal
- `n` er antall målinger i burst

### 8.1 Statistiske features

For hver kanal beregnes `mean`, `std`, `min` og `max`.

Gjennomsnitt:

```latex
\mu_i = \frac{1}{n}\sum_{t=1}^{n} x_{t,i}
```

Standardavvik:

```latex
\sigma_i =
\sqrt{
\frac{1}{n-1}
\sum_{t=1}^{n}
(x_{t,i} - \mu_i)^2
}
```

Minimum:

```latex
min_i = \min(x_{1,i}, x_{2,i}, ..., x_{n,i})
```

Maksimum:

```latex
max_i = \max(x_{1,i}, x_{2,i}, ..., x_{n,i})
```

Statistiske features forteller modellen om typisk refleksjon, variasjon, laveste respons og høyeste respons i hver kanal. De gjør målingen mer robust enn én enkelt sensoravlesning.

### 8.2 RMS

RMS står for Root Mean Square og beskriver samlet signalstyrke.

```latex
RMS_{total} =
\sqrt{
\frac{1}{36}
\sum_{i=1}^{36}
\mu_i^2
}
```

I koden beregnes RMS for hele spekteret og separat for sensor 0 og sensor 1:

```text
rms_total
rms_sensor_0
rms_sensor_1
```

`mean` og `RMS` er ikke det samme. `mean_*` gir én verdi per kanal, mens `RMS` komprimerer hele spekteret eller én sensor til ett mål på signalstyrke.

### 8.3 Kanalforhold

Kanalforhold, eller ratios, sammenligner to bølgelengder:

```latex
ratio = \frac{\mu_a}{\mu_b + \epsilon}
```

I koden brukes `eps = 1e-6` for å unngå deling på null.

Eksempler:

```text
ratio_S0_410_940
ratio_S1_410_940
ratio_S0_560_730
ratio_S1_560_730
ratio_S0_485_610
```

Ratios beskriver relativ spektral form og er mindre følsomme for total lysstyrke.

### 8.4 Spektrale gradienter

Spektrale gradienter, kalt derivatives i koden, beregnes som forskjellen mellom nabokanaler:

```latex
d_i = \mu_{i+1} - \mu_i
```

Eksempel:

```latex
derivative\_S0\_410\_435 = S0_{435} - S0_{410}
```

Dette forteller om spekteret stiger eller synker mellom to bølgelengder. Det gir informasjon om spektral form.

### 8.5 SAM

SAM står for Spectral Angle Mapper. Den måler vinkelen mellom et sample-spekter og et referansespekter:

```latex
\theta =
\arccos
\left(
\frac{x \cdot r}
{\|x\| \|r\| + \epsilon}
\right)
```

En lav SAM-verdi betyr at sample-spekteret ligner referansespekteret. En høyere verdi betyr at spektrene er mindre like.

SAM-features i prosjektet:

```text
sam_to_aluminium
sam_to_steel
sam_to_sand
```

Referansespektrene lagres i:

```text
ml/datasets/processed/sam_reference_spectra.json
```

Denne filen brukes under live-prediksjon fordi SAM ikke kan beregnes fra live-filen alene. SAM trenger både live-spekteret og et lagret referansespekter.

## 9. Hvorfor brukes `sam_reference_spectra.json` ved live-prediksjon?

De fleste features kan beregnes direkte fra live-filen:

```text
mean
std
min
max
RMS
ratios
derivatives
```

SAM er annerledes. SAM spør:

```text
Hvor lik er live-spekteret et kjent referansespekter?
```

Derfor trenger predict-scriptet:

1. Live-spekteret
2. Referansespekteret fra `sam_reference_spectra.json`

Random Forest bruker alle feature-kolonnene modellen ble trent med. Selv om én feature, for eksempel `std_31`, kan være viktigst, betyr ikke det at de andre feature-ene ikke brukes. Feature importance viser bare hvilke features modellen brukte mest totalt.

## 10. Hvorfor brukes `--max-rows 11`?

Live-filer kan inneholde 20 rader, mens treningsfilene i prosjektet har brukt 11 datarader per sample. Features som `mean`, `std`, `min` og `max` påvirkes av hvor mange rader som brukes.

Derfor brukes:

```bash
--max-rows 11
```

Dette gjør live-prediksjonen mest mulig lik treningssituasjonen.

## 11. Random Forest

Random Forest er en ensemble-modell som består av mange beslutningstrær. Hvert tre lærer regler basert på feature-verdier. Når modellen predikerer, stemmer trærne på en klasse, og flertallet bestemmer resultatet.

Fordeler med Random Forest i prosjektet:

- Fungerer godt på små og mellomstore datasett.
- Håndterer ikke-lineære sammenhenger.
- Er robust mot støy.
- Krever mindre datakraft enn dype nevrale nettverk.
- Gir feature importance, som gjør modellen lettere å tolke.

## 12. Gini, samples, value og class i beslutningstre

I visualisering av et beslutningstre betyr:

```text
gini    = hvor blandet klassene er i noden
samples = hvor mange samples som nådde noden
value   = fordeling mellom klassene
class   = klassen noden predikerer
```

Gini beregnes slik:

```latex
Gini = 1 - \sum_{k=1}^{K} p_k^2
```

`gini = 0.0` betyr at noden er ren, altså bare én klasse. For to klasser er `gini = 0.5` omtrent maksimal blanding.

Eksempel:

```text
value = [13, 13]
```

Dette betyr 13 samples i første klasse og 13 i andre klasse.

## 13. Evaluering av modellen

Modellen evalueres med:

- Accuracy
- Precision
- Recall
- F1-score
- Konfusjonsmatrise
- Classification report

`support` i classification report betyr hvor mange ekte samples som finnes i hver klasse i testsettet.

Eksempel:

```text
aluminium_kule  support = 4
stål_kule       support = 4
```

Dette betyr at testsettet har 4 aluminium-samples og 4 stål-samples.

`classification_report` viser metrikker per klasse. En egen metrics-tabell med `average="weighted"` viser én samlet score for hele modellen, der klassene vektes etter hvor mange samples de har.

## 14. OOB og testsett

Random Forest kan bruke out-of-bag-observasjoner, ofte forkortet OOB, for intern estimering av generaliseringsfeil. Likevel brukes separat testsett fordi det gir en mer uavhengig sluttvurdering.

Kort forklart:

```text
OOB = intern estimert feil under trening
testsett = uavhengig sluttkontroll etter trening
```

I rapporten kan dette formuleres slik:

```text
OOB kan redusere behovet for separat valideringsdata under modellutvikling,
men et separat testsett benyttes fortsatt for en uavhengig sluttevaluering.
```

## 15. Live-prediksjon

Eksempel på live-prediksjon:

```bash
python3 scripts/predict.py \
  --raw ml/datasets/live/T0007_triad_raw.csv \
  --model ml/models/random_forest_example_binary.joblib \
  --encoder ml/models/label_encoder_example_binary.joblib \
  --sam-refs ml/datasets/processed/sam_reference_spectra.json \
  --max-rows 11
```

Denne kommandoen:

- Leser live-råfilen.
- Bruker samme feature-ekstraksjon som trening.
- Bruker SAM-referanser fra JSON-filen.
- Bruker bare 11 rader for å matche treningsdata.
- Laster Random Forest-modellen.
- Predikerer klassen.

## 16. Feedback-loop

Det ble implementert en enkel feedback-loop i:

```text
scripts/add_feedback_sample.py
```

Den brukes når modellen gjetter feil eller når en live-prøve skal legges inn som ny treningsdata.

Flyt:

```text
live-måling
  -> prediksjon
  -> bruker oppgir riktig fasit
  -> live-fil kopieres til raw/
  -> metadata.csv oppdateres
  -> features regenereres
  -> modellen trenes på nytt
```

Modellen husker ikke feilen direkte slik et menneske gjør. Den får bare mer riktig treningsdata. Ved neste retrening bygges en ny modell basert på hele datasettet, inkludert feedback-prøven.

Eksempel:

```bash
python scripts/add_feedback_sample.py \
  --raw ml/datasets/live/S9999_triad_raw.csv \
  --correct-size liten
```

## 17. Ny trening etter feedback

Etter feedback kjøres typisk:

```bash
python scripts/extract_features.py
python scripts/split_dataset.py
python scripts/train_random_forest.py \
  --input ml/datasets/processed/train.csv \
  --label-column size_group \
  --model-output ml/models/random_forest_size.joblib \
  --encoder-output ml/models/label_encoder_size.joblib \
  --train-all
```

For materialklassifisering kan `--label-column` endres til:

```text
label_object
```

eller en annen relevant label-kolonne.

## 18. Viktige begrensninger

Maskinlæringssystemet har noen viktige begrensninger:

- Lite datasett kan gi dårlig generalisering.
- Metalloverflater gir spekulær refleksjon som endres med vinkel.
- Lysforhold og avstand påvirker signalet.
- Live-data må ligne treningsdata i måleoppsett og antall rader.
- Feature importance betyr ikke at bare én feature brukes.
- SAM krever referansespektrene fra JSON-filen for live-prediksjon.

## 19. Kort oppsummering

Maskinlæringsdelen i Moonmapper består av en komplett pipeline:

```text
Triad-måling
  -> Arduino
  -> Python datainnsamling
  -> rå CSV
  -> metadata
  -> feature-ekstraksjon
  -> Random Forest-trening
  -> evaluering
  -> live-prediksjon
  -> feedback og retrening
```

Random Forest-modellen bruker mange features samtidig. Statistiske features beskriver nivå og variasjon, RMS beskriver samlet signalstyrke, ratios og derivatives beskriver spektral form, og SAM beskriver likhet mot kjente referansespektra. Samlet gir dette modellen et bedre grunnlag for å skille mellom materialer eller objekttyper enn rådata alene.
