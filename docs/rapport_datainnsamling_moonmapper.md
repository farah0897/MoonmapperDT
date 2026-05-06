# Datainnsamling med Triad-spektrometer i Moonmapper

> Bachelorrapport – kapittelutkast. Tall og kommandoer er hentet fra `arduino/moonmapper_triad_logger/moonmapper_triad_logger.ino` (bl.a. burst = 20, baud 115200, gain 64×, integrasjon 18 sykluser, mux port 0 og 1, 36 spektralkanaler + normaliserte kolonner).

## 1 Innledning og formål

Denne delen av Moonmapper-projektet omhandler **hvordan spektral måledata fra metallprøver samles inn** og lagres i et format som kan brukes til senere **maskinlæring** og objektidentifisering. Målet er å etablere et **sporbart, konsistent og gjentakbart datagrunnlag**: samme sensoroppsett, samme signalbehandling på mikrokontroller, og et standardisert utdataformat som kan kobles til prosjektets Python-pipeline.

Datainnsamlingen er med vilje **skilt fra klassifisering**. Mikrokontrolleren (Arduino) utfører **ingen** beslutning om metalltype; den produserer kun **rå/korrigert spektral tidsserie** (burst) som lagres som CSV. Klassifisering og analyse skjer senere på PC i `ml/training/` og eventuelt i ROS-runtime (`src/moonmapper_ml/`).

## 2 Kontekst: arbeidskjeden i prosjektet

Objektidentifikasjonskjeden kan beskrives som en **tredelt prosess**:

1. **Objektlokalisering / scene-oppsett** – hvordan prøven presenteres for sensorene (avstand, vinkel, belysning i rommet, plassering i sand).
2. **Datainnsamling (dette kapittelet)** – spektralmåling og logging til fil.
3. **ML-analyse og objektidentifisering** – feature-ekstraksjon, trening, evaluering og inferens.

Kapittelet dekker primært punkt 2, men peker på hvordan punkt 3 forventer dataene strukturert.

## 3 Apparatur og oppkobling

### 3.1 Komponenter

- **To** SparkFun **AS7265X Triad**-spektrometre. Hver sensor gir **18 spektralkanaler** (smalbåndede målinger fordelt over synlig og nær-infrarødt område; i prosjektet brukes kolonnenavn med nominelle bølgelengder **410–940 nm**).
- **Én** SparkFun **Qwiic Mux (TCA9548A)** for å koble to sensorer på samme I²C-buss uten adressekonflikt.

### 3.2 Mux-porter

Programvaren bruker:

- **Sensor 0** på mux-port **0** (`SENSOR0_PORT = 0`)
- **Sensor 1** på mux-port **1** (`SENSOR1_PORT = 1`)

### 3.3 Kommunikasjon

- **I²C** via `Wire` med klokke **400 kHz** (`Wire.setClock(400000)`).
- **USB-seriell** til PC med **115200 baud** (`Serial.begin(115200)`), typisk via Arduino Serial Monitor eller et script som logger til fil.

### 3.4 Programvare på mikrokontroller

Implementasjonen ligger i:

`arduino/moonmapper_triad_logger/moonmapper_triad_logger.ino`

Biblioteker (Arduino Library Manager):

- `SparkFun AS7265X`
- `SparkFun Qwiic Mux Arduino Library`

## 4 Måleprinsipp og signalbehandling

### 4.1 Hva sensoren måler

Triaden måler **spektralt filtrert lys** som returneres til sensoren etter interaksjon med prøve og nærområde. Tallene er **kalibrerte kanalverdier** fra biblioteket (`getCalibrated*()`), ikke laboratoriemålt absolutt reflektans i SI-enheter. Verdiene er derfor **relative** og mest meningsfulle **innen samme gain, integrasjonstid og oppsett**.

### 4.2 Integrasjonstid og gain

I `initSensorOnPort()` settes:

- **Gain:** `AS7265X_GAIN_64X`
- **Integrasjon:** `setIntegrationCycles(18)`  
  Kommentaren i koden angir at bibliotekets forhold er omtrent **2,8 ms × antall sykluser**, dvs. **ca. 50 ms** integrasjon per enkelt `takeMeasurements()`-måling (avhengig av bibliotekets nøyaktige definisjon).

### 4.3 LED på – LED av (kontrollert belysning)

For hver sensor gjøres to målinger i sekvens:

1. **LED av:** alle interne “bulbs” for UV/VIS/NIR slås av (`disableBulb(...)`), deretter `takeMeasurements()` → `offVals[0..17]`.
2. **LED på:** alle tre bulbs slås på (`enableBulb(...)`), deretter `takeMeasurements()` → `onVals[0..17]`.

Deretter beregnes per kanal:

\[
\Delta_i = \mathrm{onVals}_i - \mathrm{offVals}_i
\]

Dette reduserer bidrag som ikke primært kommer fra den kontrollerte interne belysningen (mørkesignal, deler av omgivelseslys, osv.), og gir et mer stabilt spektrum for sammenligning mellom prøver.

Etter LED på-måling skrus bulbs av igjen for å etterlate sensoren i en **kjent hviletilstand** før neste kommando.

### 4.4 Sandbakgrunn (valgfri, men nødvendig for `BURST` i nåværende kode)

Kommandoen **`SB`** måler bakgrunn på **sand/underlag uten objekt** og lagrer et bakgrunnsspekter **per sensor** i `backgroundDiff[0][...]` og `backgroundDiff[1][...]`. Bakgrunnen representerer også \((\mathrm{LED\_ON}-\mathrm{LED\_OFF})\) målt på bakgrunnsscenen.

Ved senere målinger trekkes bakgrunnen fra hvis `backgroundReady[]` er satt:

\[
\mathrm{korrigert}_i = \Delta_i - \mathrm{backgroundDiff}_{sensor,i}
\]

**To sensorer** behandles separat fordi bakgrunn og optikk kan avvike mellom dem.

## 5 Kanalrekkefølge og kolonnenavn

CSV-header bruker bølgelengder i **stigende rekkefølge** (`WL`-tabellen: 410, 435, …, 940). Innlesingen fra biblioteket følger SparkFun-bibliotekets standardrekkefølge for `getCalibratedA()` osv., slik at verdien som havner under kolonnen `S0_610` svarer til **610 nm-båndet** i den mappingen SparkFun bruker i eksempelet `Example1_BasicReadings` (rekkefølgen **A,B,C,D,E,F,G,H,R,I,S,J,T,U,V,W,K,L** med tilhørende nm-kommentarer).

**Referanse:** SparkFun `Example1_BasicReadings.ino` i biblioteket:

- [Example1_BasicReadings.ino (raw)](https://raw.githubusercontent.com/sparkfun/SparkFun_AS7265x_Arduino_Library/main/examples/Example1_BasicReadings/Example1_BasicReadings.ino)

**Viktig metodepunkt:** Hvis eldre råfiler er produsert med feil mapping mellom kanalnavn og faktisk `getCalibrated*()`-rekkefølge, må dataene enten **samles på nytt** eller **korrigeres** før trening. I dette prosjektet finnes et hjelpescript:

`scripts/fix_triad_raw_channel_order.py`

som kan brukes til å permutere verdier i eksisterende raw CSV-er slik at kolonnens **navn** og **fysiske bånd** samsvarer.

## 6 Utdataformat (CSV)

### 6.1 Burst og metadatafelter

Ved kommandoen `BURST <sample_id>` tas **`BURST_SAMPLES = 20`** målinger etter hverandre. For hver rad skrives minst:

- `sample_id` (tekststreng, f.eks. `S0042`)
- `burst_index` (0…19)
- `timestamp_ms` (Arduino `millis()`)

### 6.2 Spektralkolonner

For hver sensor skrives 18 kolonner:

- `S0_410 … S0_940`
- `S1_410 … S1_940`

Totalt **36** spektralkanaler per rad.

### 6.3 Normaliserte kolonner (form-signatur)

I tillegg genereres `N0_*` og `N1_*` som en **sum-normalisering** av positive kanalbidrag per sensor (kommentar i koden: “shape-only”). Dette gir et supplement til råverdiene og kan brukes til analyse som er mer robust mot total intensitet, avhengig av senere feature-design.

### 6.4 Lagring på PC

Serial Monitor kan kopieres manuelt, eller output kan logges direkte til fil med et lite script/terminalverktøy. Viktig er at filen beholder **header + alle rader** uendret (CSV-format).

## 7 Bruksanvisning (kommandoer)

Programmet starter med vellykket init og melding om gyldige kommandoer:

`OK,Triad logger ready. Commands: SB | BURST <sample_id> | PS`

### 7.1 `SB` – bakgrunn

Måler bakgrunn for begge sensorer og aktiverer `backgroundReady` for begge. Ved feil skrives f.eks. `ERROR,SB failed...`.

### 7.2 `BURST <sample_id>` – datasettinnsamling

Krever at bakgrunn er målt (`backgroundReady` for begge sensorer). Ved suksess printes header og **20** rader.

Eksempel:

```text
BURST S0042
```

### 7.3 `PS` – enkel debug

Skriver én korrigert måling per sensor i lesbar tekstform (nyttig ved feilsøking).

Feilmeldinger følger mønster `ERROR,...` for å skille fra data.

## 8 Innsamlingsprosedyre (anbefalt for bachelor-datasett)

For å gjøre datasettet dokumenterbart i rapporten bør dere bruke en fast sjekkliste:

1. **Oppsett:** samme sandtype, samme avstand og samme “standard vinkel” når dere sammenligner klasser; eventuelle avvik **loggføres i metadata**.
2. **Varmstart / stabilisering:** vent til oppsett er stabilt (lys, temperatur, mekanikk).
3. **`SB`:** ta bakgrunn på tom sandflate uten objekt.
4. **`BURST <sample_id>`:** plasser objekt, kjør burst, lagre seriell output som fil.
5. **Metadata:** knytt `sample_id` til klasse (f.eks. aluminium, titan, jern, stål), dato, lysforhold, avstand, run-id, og filsti til rå CSV (for reproduserbarhet).

### 8.1 Variasjon med vilje vs. støy

Små variasjoner mellom burst-rader er normale (mekanisk mikrobevegelse, spekulær refleksjon fra metall, små endringer i omgivelseslys). For generalisering kan dere **systematisk** variere vinkel/avstand i egne “conditions”, slik at ML-modellen lærer variasjon som er representativ for rover-scenarioet deres.

## 9 Feilkilder og begrensninger

- **Geometri og metallflate:** kurvede kuler og spekulær glans gir stor variasjon i enkelte bånd.
- **Omgivelseslys:** LED-differanse reduserer, men eliminerer ikke nødvendigvis all variabel strølys.
- **Tidsbruk:** `LED_ON` og `LED_OFF` gir to integrasjoner per sensor per rad; to sensorer gir flere målinger → burst tar mer tid enn en enkelt “rå” avlesning.
- **Datasett uten metadata:** uten etiketter og oppsettbeskrivelse blir ML-trening vanskelig å forklare i rapport.

## 10 Kobling til resten av Moonmapper-pipelinen

Etter datainnsamling fortsetter flyten typisk slik:

1. Rå CSV lagres under `ml/datasets/...` og refereres fra `metadata.csv` (`triad_file`-felt).
2. `ml/training/extract_triad_features.py` leser råfilene og genererer numeriske features (bl.a. statistikk over burst og båndforhold).
3. `ml/training/train_random_forest.py` trener klassifikator og lagrer `.joblib`-modeller under `ml/models/`.
4. ROS-pakken `moonmapper_ml` kan laste modell og kjøre inferens.

Datainnsamlingskapittel bør eksplisitt angi **“datakontrakten”**: `S0_*`/`S1_*` som forventet av feature-ekstraksjonen.

## 11 Konklusjon (datainnsamling)

Datainnsamlingen er implementert som en **dedikert Triad-logger** som:

- bruker **to sensorer** via mux,
- gjør **LED_ON − LED_OFF**-kompensasjon,
- støtter **per-sensor sandbakgrunn**,
- logger **burst** med **20** repetisjoner per kommando,
- skriver **standardisert CSV** med **36** spektralkanaler pluss normaliserte felt,
- og er **klar for ML-pipelinen** uten å implementere klassifisering på mikrokontroller.

Dette gir et solid grunnlag for videre metodekapittel om feature-ekstraksjon, modellvalg og evaluering.

## 12 Forslag til figurer/tabeller i rapporten

- **Figur:** Blokkdiagram: Triad → Mux → Arduino → USB → CSV → Python features → modell.
- **Tabell:** Liste over kommandoer (`SB`, `BURST`, `PS`) og hva de produserer.
- **Tabell:** Kolonneeksempel (én header-linje + én data-rad, anonymisert).
- **Tabell:** Sensorparametre (gain 64×, integrasjon 18 sykluser, I²C 400 kHz, burst 20).
