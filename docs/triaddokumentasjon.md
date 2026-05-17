## `moonmapper_triad_logger.ino` – kodedokumentasjon

## 1) Hva gjør koden?

**snakker med to AS7265X Triad-sensorer via en Qwiic Mux**, tar målinger i burst, gjør en enkel korreksjon, og printer rader som CSV på Serial:

- **Sensor 0**: 18 kanaler (`S0_410 ... S0_940`)
- **Sensor 1**: 18 kanaler (`S1_410 ... S1_940`)
- I tillegg: normaliserte “shape-only” kolonner (`N0_*`, `N1_*`)

Korreksjonen per sensor er:
corrected = LED_ON - LED_OFF - sand_background 

Der `sand_background` måles og lagres med kommandoen `SB`.

---

## 2) Hardware og avhengigheter

### Hardware

- **2× SparkFun AS7265X Triad** (hver gir 18 spektralkanaler)
- **1× SparkFun Qwiic Mux (TCA9548A)** for å kunne ha to like sensorer på samme I²C-buss

Mux-portene som brukes i koden:

- Port 0 → “sensor0”
- Port 1 → “sensor1”

### Bibliotek (Arduino Library Manager)

- `SparkFun AS7265X Arduino Library`
- `SparkFun Qwiic Mux Arduino Library`

---

## 3) Hvordan dette kobles til Moonmapper-prosjektet

Denne sketchen er **datainnsamling**-delen for triad.

Typisk flyt i prosjektet:

1. **Arduino** logger rå/korrigert burst-data til CSV over Serial.
2. **PC / Python** tar inn CSV-filene og gjør feature-ekstraksjon og trening:
   - `ml/training/extract_triad_features.py` (bygger numeriske features fra rå burst)
   - `ml/training/train_random_forest.py` (trener modell)
3. **Inferens:** `scripts/predict.py` (offline). ROS-pakker `moonmapper_ml` / `moonmapper_interfaces` / `moonmapper_perception` er arkivert under `arkiverte_koder/gamle_ros_pakker/`.

I praksis: Arduino = “sensor → tall”, Python = “tall → features → modell → klasse”.

---

## 4) Kodegjennomgang:

### 5.1 Inkludering av biblioteker og globale objekter

```cpp
#include <Wire.h>

// SparkFun libraries (names may vary slightly depending on install)
#include <SparkFun_AS7265X.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>  // Qwiic Mux (TCA9548A)

AS7265X triad;
QWIICMUX mux;
```

- `#include <Wire.h>`: gir tilgang til I²C-funksjoner på Arduino.
- Kommentarlinjen (`// ...`): bare menneske-tekst; påvirker ikke programmet.
- `#include <SparkFun_AS7265X.h>`: API for AS7265X Triad-sensoren.
- `#include <SparkFun_I2C_Mux_Arduino_Library.h>`: API for mux-en (TCA9548A).
- `AS7265X triad;`: lager et globalt objekt `triad` som representerer **sensoren** (men merk: vi har to fysiske sensorer; vi bruker samme objekt og bytter mux-port).
- `QWIICMUX mux;`: globalt objekt `mux` for å velge hvilken port på mux-en vi prater med.

> Hvorfor ett `triad`-objekt når vi har to sensorer?  
> Fordi på I²C ser det ut som “én sensor om gangen”, avhengig av mux-port. Koden bytter port → kaller `triad.*()` → bytter port → kaller igjen.

---

### 5.2 Konstanter for porter, størrelser og bølgelengder

```cpp
static const uint8_t SENSOR0_PORT = 0;
static const uint8_t SENSOR1_PORT = 1;

static const uint8_t NUM_CHANNELS = 18;
static const uint8_t NUM_SENSORS = 2;
static const uint8_t BURST_SAMPLES = 20;
static const uint8_t TRIAD_DEVICES[3] = {AS72651_NIR, AS72652_VISIBLE, AS72653_UV};

// Wavelength labels for header (must match ML extractor expectations)
static const uint16_t WL[NUM_CHANNELS] = {
  410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940
};
```

- `SENSOR0_PORT = 0` og `SENSOR1_PORT = 1`: dette er mux-portene dere fysisk har koblet sensoren til.
- `NUM_CHANNELS = 18`: AS7265X gir 18 spektralkanaler.
- `NUM_SENSORS = 2`: dere har to sensorer totalt.
- `BURST_SAMPLES = 20`: ved kommando `BURST` tar dere 20 målinger.
- `TRIAD_DEVICES[3]`: AS7265X består av tre interne brikker (UV/VIS/NIR). Denne konstanten er definert, men brukes ikke direkte senere i fila (kan være der for framtidig utvidelse).
- `WL[18]`: en tabell med bølgelengder i samme rekkefølge som kanalene dere skriver ut. Dette brukes til å lage CSV-header som f.eks. `S0_410`.

> Hvorfor er `WL` viktig?  
> Fordi resten av prosjektet (Python feature-ekstraksjon) forventer at “kolonne `S0_610` faktisk er 610nm-kanalen”. Hvis rekkefølgen er feil, blir alle features feil.

---

### 5.3 Bakgrunnslagring per sensor

```cpp
// Background vectors (sand) per sensor: stores (LED_ON - LED_OFF) under sand/background
float backgroundDiff[NUM_SENSORS][NUM_CHANNELS];
bool backgroundReady[NUM_SENSORS] = {false, false};
```

- `backgroundDiff[2][18]`: to rader (en per sensor), hver med 18 tall. Dette lagrer bakgrunnsspekteret dere måler med `SB`.
- `backgroundReady`: husker om vi har målt bakgrunn ennå for hver sensor. Starter som `false, false`.

---

### 5.4 Helper: velg mux-port (`selectSensorPort`)

```cpp
bool selectSensorPort(uint8_t port) {
  if (port > 7) return false;
  mux.setPort(port);
  delay(2);
  return true;
}
```

Linje for linje:

- `bool selectSensorPort(uint8_t port) {`: definerer en funksjon som returnerer `true/false`.
- `if (port > 7) return false;`: TCA9548A har porter 0–7. Hvis noen sender inn 8+ → feil → returner `false` umiddelbart.
- `mux.setPort(port);`: fysisk velg port på mux-en.
- `delay(2);`: vent 2 ms slik at I²C/mux rekker å stabilisere seg.
- `return true;`: fortell at portvalg gikk bra.
- `}`: slutt på funksjonen.

---

### 5.5 Init per port (`initSensorOnPort`)

```cpp
bool initSensorOnPort(uint8_t port) {
  if (!selectSensorPort(port)) return false;
  // Some AS7265X boards need a tiny delay after switching mux
  delay(5);
  if (!triad.begin()) {
    return false;
  }

  // Conservative defaults (adjust later if needed)
  triad.setGain(AS7265X_GAIN_64X);         // gain
  // Integration time is 2.8ms * cycles (see SparkFun library). 18 cycles ≈ 50ms.
  triad.setIntegrationCycles(18);
  triad.disableIndicator();                // disable onboard indicator LED if present
  // Start with bulb off (turn off for all 3 internal devices)
  triad.disableBulb(AS72651_NIR);
  triad.disableBulb(AS72652_VISIBLE);
  triad.disableBulb(AS72653_UV);
  return true;
}
```

Forklaring:

- `if (!selectSensorPort(port)) return false;`:
  - `!` betyr “ikke”.
  - Hvis portvalg feiler → stopp init og returner `false`.
- `delay(5);`: litt ekstra venting etter mux-bytt.
- `if (!triad.begin()) { return false; }`:
  - `triad.begin()` prøver å finne sensoren og konfigurere den.
  - Hvis den ikke svarer → `false` (typisk wiring/port-feil).
- `triad.setGain(AS7265X_GAIN_64X);`: gjør sensoren mer følsom.
- `triad.setIntegrationCycles(18);`: hvor lenge sensoren integrerer lyset per måling.
- `triad.disableIndicator();`: slår av en liten status-LED på noen brett (for å ikke påvirke målingene visuelt).
- `disableBulb(...)` tre ganger: slår av den interne “bulb”/LED på alle tre interne enheter.
- `return true;`: init for denne porten er ok.

---

### 5.6 Lesing av alle 18 kanaler i riktig rekkefølge (`readTriad18`)

```cpp
void readTriad18(float out[NUM_CHANNELS]) {
  triad.takeMeasurementsWithBulb(); // bulb on measurement

  out[0]  = triad.getCalibratedA();  // 410
  out[1]  = triad.getCalibratedB();  // 435
  out[2]  = triad.getCalibratedC();  // 460
  out[3]  = triad.getCalibratedD();  // 485
  out[4]  = triad.getCalibratedE();  // 510
  out[5]  = triad.getCalibratedF();  // 535

  out[6]  = triad.getCalibratedG();  // 560
  out[7]  = triad.getCalibratedH();  // 585
  out[8]  = triad.getCalibratedI();  // 610
  out[9]  = triad.getCalibratedJ();  // 645
  out[10] = triad.getCalibratedK();  // 680
  out[11] = triad.getCalibratedL();  // 705

  out[12] = triad.getCalibratedR();  // 730
  out[13] = triad.getCalibratedS();  // 760
  out[14] = triad.getCalibratedT();  // 810
  out[15] = triad.getCalibratedU();  // 860
  out[16] = triad.getCalibratedV();  // 900
  out[17] = triad.getCalibratedW();  // 940
}
```

Linje for linje:

- `void readTriad18(...)`: `void` betyr “returnerer ingenting”. Funksjonen fyller arrayen `out`.
- `float out[NUM_CHANNELS]`: parameteren er en array som mottar 18 tall.
- `triad.takeMeasurementsWithBulb();`: be sensoren ta en måling med “bulb”/LED på.
- `out[i] = triad.getCalibratedX();`: henter kalibrert verdi for kanal X og lagrer i riktig indeks.

> Dette er “rette” kanalmappingen vi diskuterte. Indeks 0…17 følger stigende bølgelengde (410…940). Dette er bevisst gjort for å unngå FARAH-problemet (feil mapping ved indeks 8+).

---

### 5.7 Hovedmåling: LED\_ON − LED\_OFF og bakgrunnstrekk (`readDiffCorrected`)

Denne funksjonen er kjernen i sketchen: den gir deg “korrigerte” verdier per kanal.

```cpp
bool readDiffCorrected(uint8_t sensorIdx, uint8_t muxPort, float corrected[NUM_CHANNELS]) {
  if (!selectSensorPort(muxPort)) return false;

  float onVals[NUM_CHANNELS];
  float offVals[NUM_CHANNELS];
```

- `sensorIdx`: 0 eller 1, brukt for å velge riktig `backgroundDiff[...]`.
- `muxPort`: porten på mux-en (0 eller 1).
- `corrected[...]`: output-arrayen funksjonen fyller.
- Først: velg mux-port. Hvis det feiler → `false`.
- Lag to lokale arrayer:
  - `offVals`: måling når LED er av
  - `onVals`: måling når LED er på

#### LED OFF-målingen

```cpp
  triad.disableBulb(AS72651_NIR);
  triad.disableBulb(AS72652_VISIBLE);
  triad.disableBulb(AS72653_UV);
  triad.takeMeasurements();
  offVals[0]  = triad.getCalibratedA();
  ...
  offVals[17] = triad.getCalibratedW();
```

- `disableBulb(...)` tre ganger: slå av alle tre interne LED-enheter.
- `triad.takeMeasurements();`: ta en måling (nå uten LED).
- `offVals[...] = ...`: les ut alle 18 kanaler og lagre dem i `offVals`.

#### LED ON-målingen

```cpp
  triad.enableBulb(AS72651_NIR);
  triad.enableBulb(AS72652_VISIBLE);
  triad.enableBulb(AS72653_UV);
  triad.takeMeasurements();
  onVals[0]  = triad.getCalibratedA();
  ...
  onVals[17] = triad.getCalibratedW();
```

- `enableBulb(...)`: slå på LED-ene.
- `takeMeasurements()`: ta ny måling.
- Les ut alle 18 kanaler til `onVals`.

#### Slå av LED igjen og beregn korrigert signal

```cpp
  triad.disableBulb(AS72651_NIR);
  triad.disableBulb(AS72652_VISIBLE);
  triad.disableBulb(AS72653_UV);

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    float diff = onVals[i] - offVals[i];
    if (backgroundReady[sensorIdx]) {
      diff -= backgroundDiff[sensorIdx][i];
    }
    corrected[i] = diff;
  }

  return true;
}
```

Forklaring linje for linje:

- Først slås LED av igjen (god praksis: sketchen lar ikke LED stå på unødvendig).
- `for (...)`: en løkke som går i fra 0 til 17.
- `float diff = onVals[i] - offVals[i];`: dette fjerner mye “mørkesignal” og noe omgivelseslys.
- `if (backgroundReady[sensorIdx]) { diff -= backgroundDiff[sensorIdx][i]; }`:
  - hvis bakgrunn er målt for denne sensoren, trekk den fra.
- `corrected[i] = diff;`: skriv resultatet til output-arrayen.
- `return true;`: fortell at målingen gikk bra.

---

### 5.8 CSV-header (`printHeader`)

```cpp
void printHeader() {
  Serial.print("sample_id,burst_index,timestamp_ms");
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S0_");
    Serial.print(WL[i]);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S1_");
    Serial.print(WL[i]);
  }
  // Normalisert spektrum (shape-only): N0_* = S0_*/sum(S0_*), N1_* = S1_*/sum(S1_*)
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",N0_");
    Serial.print(WL[i]);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",N1_");
    Serial.print(WL[i]);
  }
  Serial.println();
}
```

Forklaring:

- Start med tre kolonner: `sample_id`, `burst_index`, `timestamp_ms`.
- Første `for`: legg til 18 kolonner for sensor0: `S0_410`, `S0_435`, ...
- Andre `for`: legg til 18 kolonner for sensor1: `S1_410`, ...
- Tredje og fjerde `for`: legg til normaliserte kolonner `N0_*` og `N1_*`.
- `Serial.println();` avslutter linja (CSV-headeren blir én hel linje).

---

### 5.9 CSV-rad (`printCsvRow`)

Denne skriver **én måling** som CSV.

```cpp
void printCsvRow(const String &sampleId, uint8_t burstIndex, unsigned long ts,
                 const float s0[NUM_CHANNELS], const float s1[NUM_CHANNELS]) {
  Serial.print(sampleId);
  Serial.print(",");
  Serial.print(burstIndex);
  Serial.print(",");
  Serial.print(ts);
```

- `const String &sampleId`: `const` betyr “ikke endre”, `&` betyr “referanse” (unngå kopi).
- `burstIndex`: hvilken måling i burst (0..19).
- `ts`: tid i millisekunder (fra `millis()`).
- `s0` og `s1`: arrays med 18 korrigerte tall.
- De første `Serial.print`: skriver de tre første CSV-feltene med komma.

#### Normalisering: beregn sum av positive verdier

```cpp
  float sum0 = 0.0f;
  float sum1 = 0.0f;
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    if (s0[i] > 0.0f) sum0 += s0[i];
    if (s1[i] > 0.0f) sum1 += s1[i];
  }
  const float inv0 = (sum0 > 1e-9f) ? (1.0f / sum0) : 0.0f;
  const float inv1 = (sum1 > 1e-9f) ? (1.0f / sum1) : 0.0f;
```

- `sum0/sum1`: summen av (kun) positive kanaler.
- Hvorfor bare positive? Fordi `LED_ON - LED_OFF - background` kan gi negative tall. Hvis vi summerer negative og positive kan summen bli nær 0 → normalisering blir ustabil.
- `?:` er “ternary operator”:
  - `(sum0 > 1e-9f) ? (1.0f / sum0) : 0.0f` betyr:
    - hvis `sum0` er stor nok: `inv0 = 1/sum0`
    - ellers: `inv0 = 0`

#### Skriv ut rå (korrigerte) verdier for S0 og S1

```cpp
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print(s0[i], 6);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print(s1[i], 6);
  }
```

- Hver kanal skrives med 6 desimaler (`Serial.print(x, 6)`).
- Legg merke til komma før hvert tall → dette lager CSV-format.

#### Skriv ut normalisert “shape-only” N0 og N1

```cpp
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print((s0[i] > 0.0f ? s0[i] * inv0 : 0.0f), 6);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print((s1[i] > 0.0f ? s1[i] * inv1 : 0.0f), 6);
  }
  Serial.println();
}
```

- For hver kanal:
  - hvis `s0[i] > 0`: skriv `s0[i]/sum0` (via `s0[i] * inv0`)
  - ellers: skriv 0
- Til slutt `println()` for å avslutte raden.

> Hvorfor har vi både S0/S1 og N0/N1?  
> S0/S1 inneholder både “styrke” og “form”. N0/N1 prøver å isolere “formen” på spekteret uavhengig av total mengde lys.

---

### 5.10 Kommando `SB`: mål bakgrunn (`handleSampleBackground`)

```cpp
void handleSampleBackground() {
  float s0[NUM_CHANNELS];
  float s1[NUM_CHANNELS];

  bool prev0 = backgroundReady[0];
  bool prev1 = backgroundReady[1];
  backgroundReady[0] = false;
  backgroundReady[1] = false;

  bool ok0 = readDiffCorrected(0, SENSOR0_PORT, s0);
  bool ok1 = readDiffCorrected(1, SENSOR1_PORT, s1);
```

- Lager lokale buffere `s0` og `s1`.
- Midlertidig settes `backgroundReady` til `false` for begge, for å sikre at `readDiffCorrected` **ikke** trekker fra gammel bakgrunn når vi måler ny bakgrunn.
- Kaller `readDiffCorrected` for begge sensorer.

```cpp
  if (!ok0 || !ok1) {
    Serial.println("ERROR,SB failed to read sensors (check wiring/mux ports)");
    backgroundReady[0] = prev0;
    backgroundReady[1] = prev1;
    return;
  }
```

- Hvis en av målingene feiler: print feilmelding, restore gamle `backgroundReady`, og `return`.

```cpp
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    backgroundDiff[0][i] = s0[i];
    backgroundDiff[1][i] = s1[i];
  }
  backgroundReady[0] = true;
  backgroundReady[1] = true;

  Serial.println("OK,SB background sampled for sensor0 and sensor1");
}
```

- Kopierer målingene inn i `backgroundDiff`.
- Setter `backgroundReady` til `true` for begge.
- Skriver “OK” til bruker.

---

### 5.11 Kommando `PS`: print én måling per sensor (`handlePrintSeparate`)

```cpp
void handlePrintSeparate() {
  float s0[NUM_CHANNELS];
  float s1[NUM_CHANNELS];
  bool ok0 = readDiffCorrected(0, SENSOR0_PORT, s0);
  bool ok1 = readDiffCorrected(1, SENSOR1_PORT, s1);
  if (!ok0 || !ok1) {
    Serial.println("ERROR,PS failed to read sensors");
    return;
  }
```

- Les én korrigert måling fra begge sensorer.
- Hvis fail → feilmelding.

```cpp
  Serial.print("SENSOR0");
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S0_");
    Serial.print(WL[i]);
    Serial.print("=");
    Serial.print(s0[i], 6);
  }
  Serial.println();
```

- Printer en debug-linje: `SENSOR0,S0_410=...,S0_435=...`

```cpp
  Serial.print("SENSOR1");
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S1_");
    Serial.print(WL[i]);
    Serial.print("=");
    Serial.print(s1[i], 6);
  }
  Serial.println();
}
```

- Samme for sensor1.

---

### 5.12 Kommando `BURST <sample_id>`: logg 20 rader (`handleBurst`)

```cpp
void handleBurst(const String &sampleId) {
  if (!backgroundReady[0] || !backgroundReady[1]) {
    Serial.println("ERROR,BURST requires background. Run SB first.");
    return;
  }

  printHeader();
```

- Først sjekkes at bakgrunn er målt for begge sensorer.
- Printer CSV-header før data-radene.

```cpp
  for (uint8_t i = 0; i < BURST_SAMPLES; i++) {
    float s0[NUM_CHANNELS];
    float s1[NUM_CHANNELS];
    bool ok0 = readDiffCorrected(0, SENSOR0_PORT, s0);
    bool ok1 = readDiffCorrected(1, SENSOR1_PORT, s1);
    if (!ok0 || !ok1) {
      Serial.println("ERROR,BURST failed to read sensors");
      return;
    }
    unsigned long ts = millis();
    printCsvRow(sampleId, i, ts, s0, s1);
  }
}
```

- Løkke som går 20 ganger.
- Hver gang:
  - les måling fra sensor0 og sensor1
  - ta timestamp med `millis()`
  - skriv CSV-rad med `printCsvRow(...)`

---

### 5.13 Serial input: les en hel kommando-linje (`readLine`)

```cpp
String readLine() {
  static String line = "";
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = line;
      line = "";
      out.trim();
      return out;
    }
    line += c;
    if (line.length() > 200) {
      line = "";
      return "";
    }
  }
  return "";
}
```

Forklaring:

- `String` er Arduino sin tekst-type.
- `static String line = ""`: “line” huskes mellom kall til funksjonen (buffer).
- `Serial.available()`: hvor mange tegn som ligger i bufferen.
- `Serial.read()`: les ett tegn.
- Ignorer `\r` (carriage return) slik at både Windows og Linux linjeslutt funker.
- Når `\n` kommer:
  - returner linja og tøm buffer.
- Hvis linja blir > 200 tegn: reset og returner tom streng (enkelt “sikkerhetsnett”).

> Dette er mye tryggere enn FARAH sin `flagg[3]`-lesing, fordi vi ikke skriver utenfor en 3-tegn buffer.

---

### 5.14 `setup()`: init Serial, I²C, mux og sensorer

```cpp
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();
  Wire.setClock(400000);

  if (!mux.begin()) {
    Serial.println("ERROR,Mux not detected. Check Qwiic Mux wiring.");
    return;
  }

  bool ok0 = initSensorOnPort(SENSOR0_PORT);
  bool ok1 = initSensorOnPort(SENSOR1_PORT);
```

- Start Serial på 115200.
- `while (!Serial)`: på noen boards venter dette til Serial er klar.
- Start I²C og sett hastighet 400 kHz.
- Init mux. Hvis mux ikke finnes på I²C → print error og stopp.
- Kjør init for port 0 og port 1.

```cpp
  if (!ok0 || !ok1) {
    Serial.println("ERROR,AS7265X not detected on one or both mux ports (0 and 1).");
    Serial.print("INFO,ok0=");
    Serial.print(ok0 ? "1" : "0");
    Serial.print(",ok1=");
    Serial.println(ok1 ? "1" : "0");
  } else {
    Serial.println("OK,Triad logger ready. Commands: SB | BURST <sample_id> | PS");
  }
}
```

- Hvis en sensor ikke svarer: print feilmelding og hvilken port som feilet.
- Hvis alt ok: print “klar”-melding med kommandoene.

---

### 5.15 `loop()`: kommando-dispatcher

```cpp
void loop() {
  String cmd = readLine();
  if (cmd.length() == 0) {
    delay(5);
    return;
  }
```

- Prøv å lese en kommando-linje.
- Hvis tom (ingen full linje ennå): vent litt og gå ut av `loop` denne runden.

```cpp
  if (cmd == "SB") {
    handleSampleBackground();
    return;
  }
```

- Hvis brukeren skrev `SB`: mål bakgrunn.

```cpp
  if (cmd == "PS") {
    handlePrintSeparate();
    return;
  }
```

- Hvis `PS`: print en måling per sensor.

```cpp
  if (cmd.startsWith("BURST")) {
    int sp = cmd.indexOf(' ');
    if (sp < 0 || sp == (int)cmd.length() - 1) {
      Serial.println("ERROR,BURST requires sample_id. Usage: BURST S0001");
      return;
    }
    String sampleId = cmd.substring(sp + 1);
    sampleId.trim();
    handleBurst(sampleId);
    return;
  }
```

- Hvis kommandoen begynner med `BURST`:
  - finn første mellomrom (`indexOf(' ')`).
  - hvis det ikke finnes mellomrom, eller sample_id mangler: error.
  - ellers: ta alt etter mellomrom som `sampleId`.
  - kjør `handleBurst(sampleId)`.

```cpp
  Serial.print("ERROR,Unknown command: ");
  Serial.println(cmd);
}
```

- Hvis ingen kjente kommandoer passet: print “Unknown command”.

---

## 6) Praktisk bruk (hva du skriver i Serial Monitor)

1. Åpne Serial Monitor på **115200 baud**.
2. Send:

- `SB`  → måler bakgrunn for begge sensorer (må gjøres først)
- `PS`  → debug: se en korrigert måling per sensor
- `BURST S0001` → skriver header + 20 rader for `sample_id = S0001`

> Tips: når du gjør datasettinnsamling, kjør `BURST` flere ganger per objekt og varier vinkel/avstand. Det gir modellen bedre generalisering.

---

## 7) Hvorfor denne sketchen er “riktig” design for prosjektet

- Den er **enkel** og gjør bare én ting: stabile målinger.
- Den har **forutsigbar CSV-kontrakt** (kolonnenavn matcher bølgelengder).
- Den behandler **sensor0 og sensor1 separat** (36 kanaler), som gir mer informasjon enn å gjennomsnittliggjøre dem.
- Den legger ikke “smarte” regler på microcontroller. Det gjør det mye enklere å forbedre klassifisering senere i Python/ROS uten å måtte flashe ny Arduino-kode hver gang.


