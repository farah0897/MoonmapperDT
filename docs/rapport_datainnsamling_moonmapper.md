Datainnsamling for spektral metallidentifikasjon
4.1 Relevans til problemstilling
En sentral del av oppgaven er å identifisere edle metaller under realistiske forhold.
For å oppnå dette er det nødvendig med en pålitelig metode for materialidentifikasjon. I dette prosjektet løses dette ved bruk av spektrale målinger kombinert med maskinlæring. Datainnsamlingen som beskrives i dette kapittelet danner grunnlaget for klassifiseringssystemet, og er direkte knyttet til systemkrav SRQ6, som spesifiserer at systemet skal kunne identifisere metaller med høy nøyaktighet.
Systemarkitektur for datainnsamling
Datainnsamlingen er implementert som en separat modul i systemarkitekturen. Mikrokontrolleren (Arduino) er ansvarlig for å samle inn og lagre spektrale data, men utfører ingen klassifisering. Dette er et bevisst designvalg for å sikre lav kompleksitet og høy stabilitet i det innebygde systemet.
Den totale prosesseringskjeden kan deles inn i tre steg:
	Lokalisering av objekter i robotens omgivelser 
	Spektralmåling og datalogging 
	Maskinlæringsbasert analyse og klassifisering 
Denne modulære oppdelingen gjør det mulig å videreutvikle analysemetoder uten å endre datainnsamlingssystemet.
Sensorsystem og måleprinsipp
Datainnsamlingen utføres ved hjelp av to AS7265X Triad spektrometre, som hver måler 18 spektralbånd i det synlige og nær infrarøde området. Sensorene er koblet til samme I²C-buss via en multiplexer (TCA9548A) for å unngå adressekonflikter.
Målingene representerer refleksjon av lys fra objektets overflate. Sensorverdiene er kalibrerte, men ikke absolutte målinger i SI-enheter. Dette innebærer at analysen baseres på relative forskjeller mellom spektrale signaturer.
Kommunikasjonen foregår via:
	I²C (400 kHz) mellom Arduino og sensorer 
	USB-seriell kommunikasjon til ekstern datamaskin
Målekonfigurasjon
Sensorene opererer med følgende parametere:
	Gain: 64× 
	Integrasjonstid: 18 sykluser (~50 ms per måling) 
Disse parameterne er valgt for å oppnå et godt signal til støy forhold uten å øke måletiden unødvendig.
Kontrollert belysning
For å redusere påvirkning fra omgivelseslys benyttes en differansemetode basert på interne lyskilder i sensorene.
For hver måling utføres to steg:
	Måling uten belysning (LED av) 
	Måling med belysning (LED på) 
Differansen mellom disse beregnes som:
Δ_i=〖"onVals" 〗_i-〖"offVals" 〗_i

Denne metoden reduserer effekten av omgivelseslys og forbedrer sammenlignbarheten mellom målinger.
 Bakgrunnskompensasjon
For å isolere objektets spektrale signatur benyttes bakgrunnskompensasjon. Før datainnsamling måles spekteret fra underlaget) uten objekt til stede.
Dette brukes til å korrigere målingene:
〖"korrigert" 〗_i=Δ_i-〖"bakgrunn" 〗_i

Denne metoden er spesielt viktig i et månelignende miljø hvor underlaget kan påvirke refleksjonen betydelig.
 Datainnsamling og datastruktur
Data lagres i CSV-format for enkel videre behandling. Hver måling består av Metadata som sample_id, burst_index , timestamp og Spektraldata fra 36 kanaler (18 per sensor). I tillegg henter vi Normaliserte verdier som er Intensitetsuavhengige representasjoner av spekteret. Datainnsamlingen utføres i bursts på 20 målinger per objekt for å gi et sterkere datagrunnlag og muliggjør statistisk analyse. 



Innsamlingsprosedyre
For å sikre høy datakvalitet er følgende prosedyre definert. 
	Standardisering av målegeometri
Vi har forhold bestemt avstand, vinkel, underlag  og sensor, og metalet. 
	Stabilisering av systemet 
	Måling av bakgrunn på forhold 
	Gjennomføring av burst-måling 
	Registrering av metadata 
Kontrollert variasjon i måleforhold kan også introduseres for å forbedre modellens generaliseringsevne.
Feilkilder og begrensninger
Følgende faktorer kan påvirke målekvaliteten:
	Refleksjonsegenskaper til metalloverflater 
	Variasjoner i omgivelseslys 
	Geometriske variasjoner (vinkel og avstand) 
	Begrenset representasjon uten tilstrekkelig metadata 
Disse faktorene må tas hensyn til både i datainnsamling og videre analyse.
4.10 Integrasjon i maskinlæring
Etter datainnsamling inngår dataene i en videre prosessering:
	Lagring av rådata (CSV) 
	Feature-ekstraksjon 
	Random Forest Modelltrening
	Evaluering og inferens 
Denne strukturen muliggjør effektiv utvikling av klassifikasjonsmodeller uten endringer i datainnsamlingssystemet.

