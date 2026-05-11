    skripte gjør fire hovedting:

    1. Sender kommandoer til Arduino.
    2. Leser CSV-data tilbake fra Arduino.
    3. Lagrer målingene som en `.csv`-fil.
    4. Kan legge til metadata om prøven i `metadata.csv`.

    Arduinoen gjør selve sensormålingen. Python-scriptet samler, sjekker og lagrer dataene.


    ```python
    """
    Collect a Triad BURST over serial and write to a raw CSV file.
    ...
    """
    ```

    Dette er en **docstring**. Det er tekst som forklarer hva filen gjør.

    Den sier:

    - Scriptet samler en `BURST` fra Triad-sensoren.
    - Arduino forventer kommandoen `BURST <sample_id>`.
    - Arduino svarer med én header-linje og 20 datarader.
    - Data lagres som `ml/datasets/raw/<sample_id>_triad_raw.csv`.
    - Scriptet trenger Python-pakken `pyserial`.

    ## Linje 22: Fremtidig Type-Syntaks

    ```python
    from __future__ import annotations
    ```

    Dette gjør at Python kan håndtere typehint litt mer fleksibelt.

    Typehint betyr små hint om hva slags data en variabel eller funksjon bruker.

    Eksempel:

    ```python
    timeout_s: float
    ```

    Det betyr at `timeout_s` bør være et desimaltall, for eksempel `5.0`.

    ## Linje 24-29: Importer

    ```python
    import argparse
    import csv
    import sys
    import time
    from pathlib import Path
    from typing import Dict, List, Optional, Tuple
    ```

    Her henter scriptet ferdige Python-verktøy.

    `argparse` brukes for kommandoer i terminalen, for eksempel `--port /dev/ttyACM0`.

    `csv` brukes for å lese og skrive CSV-filer riktig.

    `sys` brukes her mest for å skrive feilmeldinger til `stderr`.

    `time` brukes for pauser og timeout.

    `Path` brukes for filstier på en tryggere måte enn vanlige tekststrenger.

    `Dict`, `List`, `Optional`, `Tuple` brukes til typehint.

    ## Linje 32-40: Sjekker At `pyserial` Finnes

    ```python
    def _require_pyserial():
        try:
            import serial
            return serial
        except Exception as exc:
            raise RuntimeError(...) from exc
    ```

    `def` betyr at vi lager en funksjon.

    Funksjonen heter `_require_pyserial`.

    Den prøver å importere `serial`, som kommer fra pakken `pyserial`.

    Hvis `pyserial` finnes, returnerer den `serial`.

    Hvis pakken mangler, lager den en tydelig feilmelding:

    ```text
    pyserial is not installed. Install in your ML venv with: pip install pyserial
    ```

    Understreken først i `_require_pyserial` betyr: denne funksjonen er ment som intern hjelpefunksjon.

    ## Linje 43-61: Leser Én Linje Med Timeout

    ```python
    def _readline_with_timeout(ser, *, timeout_s: float) -> Optional[str]:
    ```

    Denne funksjonen prøver å lese én tekstlinje fra Arduino.

    `ser` er serial-forbindelsen.

    `timeout_s` er hvor lenge vi maksimalt venter.

    `Optional[str]` betyr: funksjonen returnerer enten tekst (`str`) eller ingenting (`None`).

    ```python
    deadline = time.time() + timeout_s
    ```

    Her regner scriptet ut når det skal slutte å vente.

    ```python
    buf = bytearray()
    ```

    `buf` er en midlertidig buffer. Den samler byte for byte fra Arduino.

    ```python
    while time.time() < deadline:
    ```

    Dette er en løkke. Den kjører så lenge vi ikke har passert tidsgrensen.

    ```python
    chunk = ser.read(1)
    ```

    Her leses ett tegn/én byte fra serial-porten.

    ```python
    if chunk == b"\n":
    ```

    `b"\n"` betyr newline som bytes. Newline betyr slutten på en linje.

    Når en hel linje er mottatt, konverteres bytes til tekst:

    ```python
    return buf.decode("utf-8", errors="replace").strip()
    ```

    `decode("utf-8")` gjør bytes om til vanlig tekst.

    `strip()` fjerner ekstra mellomrom og linjeskift på starten/slutten.

    ```python
    if chunk != b"\r":
        buf.extend(chunk)
    ```

    Dette ignorerer `\r`, som ofte kommer fra Windows-/Arduino-linjeslutt.

    Hvis det ikke kommer data:

    ```python
    time.sleep(0.01)
    ```

    Da venter scriptet 0.01 sekund, så det ikke bruker CPU unødvendig.

    Hvis tiden går ut:

    ```python
    return None
    ```

    Da betyr det: ingen komplett linje ble mottatt.

    ## Linje 64-73: Sender Kommando Og Forventer `OK`

    ```python
    def _send_command_and_expect_ok(ser, *, command: str, timeout_s: float) -> None:
    ```

    Denne funksjonen sender en kommando til Arduino og forventer at Arduino svarer med `OK`.

    ```python
    ser.write((command.strip() + "\n").encode("utf-8"))
    ```

    Dette gjør tre ting:

    - `command.strip()` fjerner ekstra mellomrom.
    - `+ "\n"` legger til linjeskift, slik Arduino skjønner at kommandoen er ferdig.
    - `encode("utf-8")` gjør tekst om til bytes før sending.

    ```python
    ser.flush()
    ```

    Dette tvinger Python til å sende alt med én gang.

    ```python
    line = _readline_with_timeout(...)
    ```

    Så venter scriptet på svar fra Arduino.

    Hvis svaret mangler, kommer timeout-feil.

    Hvis svaret starter med `ERROR`, stoppes scriptet med Arduino-feil.

    Hvis svaret ikke starter med `OK`, stoppes scriptet fordi svaret var uventet.

    Denne brukes blant annet når Python sender `SB` for bakgrunnsmåling.

    ## Linje 76-86: Hovedfunksjonen For Innsamling

    ```python
    def collect_burst(
        *,
        port: str,
        baud: int,
        sample_id: str,
        ...
    ) -> Tuple[str, List[str]]:
    ```

    Dette er funksjonen som faktisk samler målinger fra Arduino.

    Parameterne betyr:

    `port`: hvilken USB/serial-port Arduino ligger på, for eksempel `/dev/ttyACM0`.

    `baud`: hastighet for serial-kommunikasjon, vanligvis `115200`.

    `sample_id`: ID for prøven, for eksempel `S0001`.

    `sample_background`: om scriptet først skal sende `SB` for sandbakgrunn.

    `wait_after_sb_s`: hvor mange sekunder scriptet skal vente etter bakgrunnsmåling.

    `interactive_after_background`: om brukeren må trykke Enter etter bakgrunnsmåling.

    `debug`: om scriptet skal skrive ekstra mottatte linjer til terminalen.

    `timeout_s`: hvor lenge scriptet venter på svar før det gir opp.

    Returnverdien:

    ```python
    Tuple[str, List[str]]
    ```

    betyr at funksjonen returnerer to ting:

    1. Header-linjen som tekst.
    2. En liste med CSV-rader.

    ## Linje 87-98: Åpner Serial-Porten

    ```python
    serial = _require_pyserial()
    ```

    Først sjekker scriptet at `pyserial` er installert.

    ```python
    ser = serial.Serial(port=port, baudrate=baud, timeout=0)
    ```

    Dette åpner forbindelsen til Arduino.

    `timeout=0` betyr non-blocking read. Altså: lesing stopper ikke programmet mens det venter.

    ```python
    with ser:
    ```

    `with` sørger for at serial-porten blir lukket riktig etterpå.

    ```python
    time.sleep(2.0)
    ```

    Arduino resetter ofte når serial åpnes. Derfor venter scriptet 2 sekunder.

    ```python
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ```

    Dette tømmer gamle data som kan ligge i buffer.

    ## Linje 100-121: Valgfri Bakgrunnsmåling

    ```python
    if sample_background:
    ```

    Hvis brukeren har valgt `--sample-background`, kjører denne delen.

    ```python
    _send_command_and_expect_ok(ser, command="SB", timeout_s=timeout_s)
    ```

    Python sender `SB` til Arduino.

    `SB` betyr sample background, altså mål sand/underlag uten objekt.

    ```python
    time.sleep(0.2)
    ```

    Kort pause etter bakgrunnsmålingen.

    ```python
    if wait_after_sb_s > 0:
    ```

    Hvis brukeren har bedt om ventetid, venter scriptet.

    Dette er nyttig hvis du skal rekke å legge objektet i sanden før `BURST`.

    ```python
    if interactive_after_background:
    ```

    Hvis dette er aktivert, stopper scriptet og venter på at brukeren trykker Enter.

    ```python
    input()
    ```

    Dette venter på brukerinput.

    ```python
    except EOFError:
        pass
    ```

    Hvis scriptet kjører i et miljø uten tastaturinput, ignoreres feilen og scriptet fortsetter.

    ## Linje 123-125: Sender `BURST`

    ```python
    cmd = f"BURST {sample_id}\n".encode("utf-8")
    ser.write(cmd)
    ser.flush()
    ```

    Her lages kommandoen som sendes til Arduino.

    Hvis `sample_id` er `S0001`, blir kommandoen:

    ```text
    BURST S0001
    ```

    Arduino skal da ta 20 målinger og sende dem tilbake som CSV.

    `f"BURST {sample_id}\n"` er en f-string. Det betyr at Python setter verdien av `sample_id` inn i teksten.

    ## Linje 127-148: Venter På CSV-Header

    ```python
    deadline = time.time() + timeout_s
    header: Optional[str] = None
    ```

    Scriptet setter en tidsgrense og sier at `header` foreløpig er tom.

    ```python
    while time.time() < deadline:
    ```

    Så leser det linjer frem til timeout.

    ```python
    if debug:
        print(...)
    ```

    Hvis debug er aktivert, skrives mottatte linjer ut.

    ```python
    if line.startswith("ERROR"):
    ```

    Hvis Arduino sender feil, stopper scriptet.

    ```python
    if line.startswith("sample_id,burst_index,timestamp_ms,"):
    ```

    Dette er viktig. Scriptet leter etter CSV-headeren.

    Headeren er første linje i CSV-tabellen. Den forklarer kolonnenavnene:

    ```text
    sample_id,burst_index,timestamp_ms,S0_410,...
    ```

    Hvis header ikke kommer før timeout:

    ```python
    raise RuntimeError(...)
    ```

    Da får brukeren beskjed om at Arduino ikke svarte med riktig CSV-header.

    ## Linje 150-173: Leser 20 Datarader

    ```python
    rows: List[str] = []
    ```

    Her lager scriptet en tom liste som skal fylles med CSV-rader.

    ```python
    for i in range(20):
    ```

    Dette betyr: gjør dette 20 ganger.

    Det matcher Arduino-koden, som lager 20 burst samples.

    ```python
    row_deadline = time.time() + timeout_s
    line: Optional[str] = None
    ```

    For hver rad får scriptet en ny tidsgrense.

    ```python
    candidate = _readline_with_timeout(...)
    ```

    Scriptet leser én mulig linje fra Arduino.

    ```python
    if not candidate:
        continue
    ```

    Hvis linjen er tom, hopp over og les neste.

    ```python
    if candidate.startswith("ERROR"):
    ```

    Hvis Arduino sender feil midt i burst, stoppes scriptet.

    ```python
    if "," in candidate:
    ```

    CSV-rader inneholder komma, så dette brukes som enkel sjekk.

    ```python
    rows.append(line)
    ```

    Når en gyldig rad er funnet, legges den til i listen.

    Til slutt:

    ```python
    return header, rows
    ```

    Funksjonen returnerer headeren og alle dataradene.

    ## Linje 176-193: Skriver Rå CSV-Fil

    ```python
    def write_raw_csv(*, output_path: Path, header_line: str, rows: List[str]) -> None:
    ```

    Denne funksjonen lagrer målingen til fil.

    `output_path` er hvor filen skal lagres.

    `header_line` er CSV-headeren.

    `rows` er de 20 dataradene.

    ```python
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ```

    Dette lager mappen hvis den ikke finnes.

    Eksempel:

    ```text
    ml/datasets/raw/
    ```

    ```python
    header = next(csv.reader([header_line]))
    parsed_rows = [next(csv.reader([r])) for r in rows]
    ```

    Her bruker scriptet Python sitt `csv`-bibliotek til å tolke CSV riktig.

    Det er bedre enn å bare bruke `split(",")`, fordi CSV kan ha spesialtilfeller.

    ```python
    expected_min_cols = 3 + 36
    ```

    Scriptet forventer minst:

    - 3 metadatafelt: `sample_id`, `burst_index`, `timestamp_ms`
    - 36 spektralkanaler: 2 sensorer x 18 kanaler

    Totalt minst 39 kolonner.

    ```python
    if len(header) < expected_min_cols:
    ```

    Hvis headeren har for få kolonner, stoppes scriptet.

    ```python
    if len(r) != len(header):
    ```

    Hver rad må ha like mange kolonner som headeren.

    Til slutt skrives filen:

    ```python
    with output_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(parsed_rows)
    ```

    Dette lager en ordentlig CSV-fil på PC-en.

    ## Linje 196-211: Metadata-Kolonner

    ```python
    METADATA_HEADER: List[str] = [
        "sample_id",
        "label_object",
        ...
    ]
    ```

    Dette er listen over kolonner i `metadata.csv`.

    Metadata beskriver forsøket.

    Eksempler:

    `sample_id`: ID-en til prøven.

    `label_object`: objektklassen, for eksempel `stål_kule`.

    `label_material`: materiale, for eksempel `steel`.

    `triad_file`: filen som inneholder råmålingen.

    `sand_type`: hvilken sand/underlag.

    `lysforhold`: lysforhold.

    `avstand_cm`: avstand fra sensor til objekt.

    `angle_id`: vinkelkode.

    `buried_level`: hvor mye objektet er begravd.

    Metadata er viktig fordi ML-modellen trenger fasit og kontekst.

    ## Linje 214-220: Lager Metadata-Fil Hvis Den Mangler

    ```python
    def ensure_metadata_file(path: Path) -> None:
    ```

    Denne funksjonen sjekker at metadatafilen finnes.

    ```python
    path.parent.mkdir(parents=True, exist_ok=True)
    ```

    Lager mappen hvis den mangler.

    ```python
    if path.exists():
        return
    ```

    Hvis filen allerede finnes, gjør funksjonen ingenting.

    ```python
    writer.writerow(METADATA_HEADER)
    ```

    Hvis filen ikke finnes, opprettes den med header-kolonnene.

    ## Linje 223-227: Legger Til Én Metadata-Rad

    ```python
    def append_metadata_row(path: Path, row: Dict[str, str]) -> None:
    ```

    Denne funksjonen legger én ny rad til `metadata.csv`.

    `Dict[str, str]` betyr en ordbok der nøkkel og verdi er tekst.

    Eksempel:

    ```python
    {
        "sample_id": "S0001",
        "label_object": "stål_kule"
    }
    ```

    ```python
    ensure_metadata_file(path)
    ```

    Først sikrer den at metadatafilen finnes.

    ```python
    with path.open("a", newline="") as f:
    ```

    `"a"` betyr append, altså legg til nederst i filen.

    ```python
    writer.writerow({k: row.get(k, "") for k in METADATA_HEADER})
    ```

    Dette skriver radverdiene i riktig kolonnerekkefølge.

    Hvis en verdi mangler, brukes tom tekst `""`.

    ## Linje 230-328: Terminal-Argumenter

    ```python
    def main(argv: Optional[List[str]] = None) -> int:
    ```

    `main` er hovedfunksjonen som kjøres når du starter scriptet fra terminal.

    ```python
    parser = argparse.ArgumentParser(...)
    ```

    Dette lager en parser. Parseren leser argumenter fra terminalen.

    Eksempel:

    ```bash
    python ml/data_collection/collect_triad_burst.py --port /dev/ttyACM0 --sample-id S0001
    ```

    ### Viktige Argumenter

    ```python
    parser.add_argument("--port", required=True, ...)
    ```

    `--port` er påkrevd. Scriptet må vite hvor Arduino er koblet til.

    ```python
    parser.add_argument("--baud", type=int, default=115200, ...)
    ```

    `--baud` er hastigheten. Standard er `115200`.

    ```python
    parser.add_argument("--sample-id", required=True, ...)
    ```

    `--sample-id` er også påkrevd. Hver måling må ha en ID.

    ```python
    parser.add_argument("--sample-background", action="store_true", ...)
    ```

    `action="store_true"` betyr at dette er en av/på-bryter.

    Hvis du skriver `--sample-background`, blir verdien `True`.

    Hvis du ikke skriver den, blir verdien `False`.

    ```python
    parser.add_argument("--wait-after-sb", "--vent-etter-sb", ...)
    ```

    Dette argumentet har to navn. Du kan bruke engelsk eller norsk variant.

    Det bestemmer ventetid etter bakgrunnsmåling.

    ```python
    parser.add_argument("--interactive-after-background", action="store_true", ...)
    ```

    Dette gjør at scriptet venter på Enter etter bakgrunnsmålingen.

    ```python
    parser.add_argument("--output-dir", default="ml/datasets/raw", ...)
    ```

    Dette bestemmer hvor rå CSV-filen skal lagres.

    ```python
    parser.add_argument("--overwrite", action="store_true", ...)
    ```

    Hvis filen finnes fra før, stopper scriptet normalt.

    Med `--overwrite` får scriptet lov til å overskrive.

    ```python
    parser.add_argument("--debug", action="store_true", ...)
    ```

    Med debug ser du flere linjer som mottas fra Arduino.

    ```python
    parser.add_argument("--append-metadata", action="store_true", ...)
    ```

    Hvis denne brukes, legges metadata til i `metadata.csv`.

    Resten av argumentene er metadatafelt, for eksempel:

    - `--label-object`
    - `--label-material`
    - `--sand-type`
    - `--lysforhold`
    - `--avstand-cm`
    - `--angle-id`
    - `--run-id`

    ```python
    args = parser.parse_args(argv)
    ```

    Her leses alle argumentene inn i objektet `args`.

    Etter dette kan koden bruke for eksempel:

    ```python
    args.port
    args.sample_id
    args.output_dir
    ```

    ## Linje 330-338: Lager Filnavn Og Hindrer Overskriving

    ```python
    out_dir = Path(args.output_dir)
    out_path = out_dir / f"{args.sample_id}_triad_raw.csv"
    ```

    Her lager scriptet filstien til rådatafilen.

    Hvis `sample_id` er `S0001`, blir standard fil:

    ```text
    ml/datasets/raw/S0001_triad_raw.csv
    ```

    ```python
    if out_path.exists() and not args.overwrite:
    ```

    Hvis filen allerede finnes, og brukeren ikke har skrevet `--overwrite`, stopper scriptet.

    Dette beskytter deg mot å ødelegge gamle målinger ved et uhell.

    ```python
    return 2
    ```

    `2` betyr at scriptet avsluttet med feil.

    ## Linje 340-354: Samler Måling Og Skriver CSV

    ```python
    try:
        header, rows = collect_burst(...)
        write_raw_csv(...)
    except Exception as exc:
        print(...)
        return 2
    ```

    Dette er hovedjobben.

    Først kalles `collect_burst`, som snakker med Arduino.

    Så kalles `write_raw_csv`, som lagrer dataene.

    `try/except` betyr:

    - Prøv å gjøre dette.
    - Hvis noe går galt, fang feilen og skriv en ryddig feilmelding.

    Argumentene fra terminalen sendes videre:

    ```python
    port=args.port
    baud=int(args.baud)
    sample_id=str(args.sample_id)
    ```

    Dette kobler terminalkommandoen til funksjonen som faktisk samler data.

    ## Linje 356-388: Valgfri Metadata-Lagring

    ```python
    if args.append_metadata:
    ```

    Hvis brukeren skrev `--append-metadata`, kjøres denne delen.

    ```python
    meta_path = Path(args.metadata_path)
    ```

    Dette er filen metadata skal skrives til.

    Standard er:

    ```text
    ml/datasets/metadata.csv
    ```

    ```python
    triad_rel = out_path.relative_to(Path("ml/datasets"))
    ```

    Scriptet prøver å lagre `triad_file` relativt til `ml/datasets`.

    Eksempel:

    ```text
    raw/S0001_triad_raw.csv
    ```

    Det er bedre enn full absolutt sti, fordi prosjektet lettere kan flyttes.

    ```python
    row: Dict[str, str] = {
        "sample_id": str(args.sample_id),
        ...
    }
    ```

    Her bygges metadata-raden.

    Alle verdier kommer fra terminalargumentene.

    ```python
    append_metadata_row(meta_path, row)
    ```

    Dette skriver raden til `metadata.csv`.

    Hvis noe går galt under metadata-lagring, skrives en feilmelding og scriptet returnerer `2`.

    ## Linje 390-391: Vellykket Avslutning

    ```python
    print(f"[collect_triad_burst] Saved: {out_path}")
    return 0
    ```

    Dette betyr at scriptet lyktes.

    `return 0` betyr vanligvis: ingen feil.

    ## Linje 394-395: Kjør `main`

    ```python
    if __name__ == "__main__":
        raise SystemExit(main())
    ```

    Dette er standard Python-mønster.

    Det betyr:

    Hvis denne filen kjøres direkte fra terminalen, kjør `main()`.

    Eksempel:

    ```bash
    python ml/data_collection/collect_triad_burst.py --port /dev/ttyACM0 --sample-id S0001
    ```

    `SystemExit(main())` gjør at tallet fra `main` blir exit-koden til programmet.

    `0` betyr suksess.

    `2` betyr feil.

    ## Eksempel På Bruk

    En enkel innsamling:

    ```bash
    python ml/data_collection/collect_triad_burst.py \
    --port /dev/ttyACM0 \
    --sample-id S0001
    ```

    Med bakgrunnsmåling og metadata:

    ```bash
    python ml/data_collection/collect_triad_burst.py \
    --port /dev/ttyACM0 \
    --sample-id S0001 \
    --sample-background \
    --interactive-after-background \
    --append-metadata \
    --label-object staal_kule \
    --label-material steel \
    --sand-type torr_sand \
    --lysforhold lampelys \
    --avstand-cm 8 \
    --angle-id A30 \
    --run-id R01
    ```

    ## Hele Flyten Enkelt Forklart

    ```text
    1. Du starter Python-scriptet.
    2. Python åpner USB/Serial-porten til Arduino.
    3. Python kan sende SB for bakgrunnsmåling.
    4. Python sender BURST S0001.
    5. Arduino måler med Triad-sensorene.
    6. Arduino sender CSV-header og 20 rader tilbake.
    7. Python sjekker at CSV-en ser riktig ut.
    8. Python lagrer rådatafilen.
    9. Python kan legge inn metadata i metadata.csv.
    ```

    ## Viktigste Ting Å Huske

    Arduino-filen gjør selve sensormålingen.

    Python-filen gjør datainnsamling på PC.

    De er ikke samme kode. De samarbeider via en enkel protokoll:

    ```text
    Python sender: BURST S0001
    Arduino svarer: CSV-data
    ```

    Dette scriptet er viktig fordi ML-modellen trenger ryddige filer og metadata for å kunne trenes senere.
