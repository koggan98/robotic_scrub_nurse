# „Robot“-Wake-Word auf dem Hochschul-GPU-Hub trainieren

Dieses Paket erzeugt einen **ONNX-Kandidaten** für den akustischen Wake-Word-
Filter des Robotic Scrub Nurse. Es verwendet die offizielle
openWakeWord-0.6.0-Pipeline mit festen Quellständen. TensorFlow und TFLite
werden nicht benötigt: Der Roboter lädt ausschließlich ONNX.

Der Trainingskandidat ist nach dem GPU-Lauf noch nicht freigegeben. Er muss
anschließend mit Samson Q2U und Jieli am Jetson kalibriert und abgenommen
werden.

> **Mac und Hochschul-VPN:** Wenn dieser Linux-Rechner den GPU-Hub nicht
> direkt erreicht, folge zuerst der separaten
> [Mac-Transferanleitung](MAC_TRANSFER_DE.md). Sie beschreibt Linux → Mac →
> VPN → Browser/Jupyter sowie den Rückweg des trainierten Modells mit
> Prüfsummen.

## 1. Was du hochlädst

Lade nur das bereitgestellte Archiv
`robot_wakeword_training_kit.tar.gz` auf den GPU-Hub. Es enthält:

- Produktions- und Smoke-Konfiguration,
- Setup-, Download-, Trainings- und Prüfscripte,
- Aufnahme- und Offline-Auswertungstools,
- feste Quellstände, URLs, Größen und SHA-256-Werte.

Es enthält bewusst weder die 17,28-GB-Featuredatei noch Drittanbieter-
Repositories oder Sprachaufnahmen. Diese werden auf dem Hub heruntergeladen
beziehungsweise separat hinzugefügt.

## 2. GPU-Hub-Umgebung auswählen

Wähle:

- **PyTorch**, nicht TensorFlow,
- Linux x86_64,
- Python **3.10**,
- exakt PyTorch **2.10.0** mit Build-CUDA **12.6 oder 12.8**,
- cuDNN **9**,
- kein CUDA-13-Image,
- eine NVIDIA-GPU mit mindestens 12 GB VRAM, 16 GB oder mehr empfohlen,
- mindestens 8 CPU-Kerne,
- mindestens 32 GB RAM, 64 GB sind komfortabler,
- mindestens 80 GB freien, persistenten und möglichst schnellen Speicher,
- mindestens 300.000 freie Inodes.

Der Ordner muss nach Ende oder Neustart eines GPU-Jobs erhalten bleiben.
Trainiere nicht in einem kleinen Home-Quota oder einem automatisch gelöschten
`/tmp`. Für die Produktionsstufen einen ausreichend langen Batch-/GPU-Job
verwenden; ein geschlossener Browser darf den Prozess nicht beenden.

Warum PyTorch: Piper-TTS, Featureerzeugung und Klassifikatortraining verwenden
PyTorch. TensorFlow 2.8.1 wird upstream erst nach dem fertigen ONNX-Export für
eine zusätzliche TFLite-Datei aufgerufen. Das Kit überspringt exakt diesen
unbenötigten Schritt.

`torchaudio` darf im Hub-Image fehlen oder eine unpassende Installation sein.
Das Setup legt im privaten `.venv` automatisch exakt `torchaudio==2.10.0` aus
dem offiziellen `cu126`- beziehungsweise `cu128`-Index ab, ohne das vom Hub
bereitgestellte PyTorch zu ersetzen.

## 3. Archiv entpacken

Im persistenten Projektverzeichnis:

```bash
tar -xzf robot_wakeword_training_kit.tar.gz
cd robot_wakeword_training_kit
```

Alle folgenden Befehle werden aus diesem Verzeichnis ausgeführt.

## 4. Umgebung und Quellen einrichten

Der Setup-Lauf benötigt einen zugewiesenen GPU-Knoten und Internetzugriff:

```bash
bash scripts/01_setup.sh
```

Das Script:

1. prüft Python, Linux, GPU, PyTorch, Speicher und Inodes,
2. erzeugt `.venv` mit Zugriff auf das CUDA-PyTorch des Hub-Images,
3. installiert das zu PyTorch 2.10 passende TorchAudio sowie die gepinnten
   ONNX-Trainingsabhängigkeiten ohne TensorFlow,
4. klont openWakeWord `v0.6.0` und Piper `v2.0.0`,
5. prüft die exakten Git-Commits,
6. entfernt nur die TFLite-Konvertierung nach dem ONNX-Export,
7. erzwingt für PyTorch 2.10 den bisherigen ONNX-Exporter mit
   `dynamo=False` und Opset 13,
8. stellt die in TorchAudio 2.10 entfernten WAV-Ladefunktionen für den alten
   Trainingscode über den gepinnten SoundFile-Adapter bereit,
9. begrenzt Upstream-Worker auf die CPU-Affinität des Scheduler-Jobs und
   verhindert eine Workerzahl von null,
10. prüft mit Mono-/Stereo-WAV, Resampling und einem echten Mini-ONNX-Export
    die Kompatibilität,
11. verlangt einen funktionierenden ONNX Runtime CUDA Provider.

Wenn die Prüfung Python 3.10, PyTorch 2.10 oder Build-CUDA 12.6/12.8 ablehnt,
nicht mit einem anderen Image improvisieren: im Hub das passende
PyTorch-2.10-Image auswählen.

## 5. Unveränderliche Trainingsdaten herunterladen

```bash
bash scripts/02_download_assets.sh
```

Der größte Download ist:

```text
data/openwakeword_features_ACAV100M_2000_hrs_16bit.npy
17,280,000,128 Bytes
```

Das Script unterstützt HTTP-Resume. Jede Datei wird danach anhand von
Byteanzahl und SHA-256 geprüft. Dazu gehören:

- Piper-Sprachmodell,
- Mel-Spektrogramm-ONNX,
- Embedding-ONNX,
- rund 2.000 Stunden vorverarbeitete allgemeine Negativfeatures,
- False-Positive-Validierungsfeatures.

Nach verifizierter Piper-Modellprüfung erzeugt das Script zusätzlich einen
temporären echten „robot“-Testclip und verlangt mono, 16 kHz, PCM16 sowie ein
nicht-stilles Signal. Der Testclip wird anschließend automatisch entfernt.

Bei einem abgebrochenen Download denselben Befehl erneut ausführen. Bei einer
vollständigen Datei mit falscher Prüfsumme die fehlerhafte Datei erst
umbenennen und dann neu laden; das Script löscht sie nicht automatisch.

Falls GPU-Compute-Knoten kein Internet haben, Setup und Downloads in einer
internetfähigen Hub-Sitzung auf demselben persistenten Projektlaufwerk
ausführen. Alternativ exakt die Dateien aus `manifests/assets.json` auf einem
verbundenen Rechner laden und an die dort angegebenen Zielpfade legen; der
Downloadlauf erkennt bereits korrekte Dateien und lädt sie nicht erneut.

## 6. Raumimpulsantworten vorbereiten

```bash
.venv/bin/python scripts/03_prepare_rirs.py
```

Dies reproduziert den MIT-RIR-Schritt aus dem offiziellen Notebook und schreibt
den vollständigen, gepinnten Satz aus **271** 16-kHz-PCM-WAVs nach:

```text
data/mit_rirs/
```

Keine README, JSON-Datei oder Unterordner in dieses Verzeichnis legen.
openWakeWord 0.6.0 versucht jeden direkten Verzeichniseintrag als Audio zu
lesen.

## 7. Brauche ich eigene „Robot“-Aufnahmen?

**Nicht fürs eigentliche Training.** Piper erzeugt die 100.000 positiven
„robot“-Beispiele und die konfigurierten Verwechslungen synthetisch.

Eigene Aufnahmen sind sinnvoll für:

1. reale Raum-, Roboter-, Instrumenten- und Gesprächsgeräusche als
   Trainingsaugmentation,
2. eine strikt getrennte Offline-Kalibrierung,
3. die abschließende Live-Abnahme auf dem Jetson.

### Datenschutz

- Keine Patienten oder echten klinischen Gespräche aufnehmen.
- Nebengespräche mit informierter Zustimmung inszenieren.
- Keine Namen in Dateinamen oder Metadaten verwenden.
- Hochschulvorgaben zu Forschungsdaten, Ethik, Speicherort und Löschfrist
  beachten.
- Nur freigegebene Trainingshintergründe auf den GPU-Hub übertragen.
- Private Rohdaten und finale Abnahmedaten können lokal bleiben.

## 8. Trainingshintergründe aufnehmen

Vor der Aufnahme ASR, Command-Router und autonome Bewegungssteuerung stoppen.
Das Aufnahmetool führt keine ROS-Kommandos aus. Bewegungsgeräusche nur in einem
kontrollierten manuellen/Jog-Test unter den üblichen Sicherheitsregeln
aufnehmen.

Das Archiv enthält `tools/record_audio.py`. Kopiere diese einzelne Datei bei
Bedarf auf den Rechner beziehungsweise Jetson mit angeschlossenem Mikrofon.
Das Tool setzt private Rechte (`umask 077`), speichert WAVs mit Modus `0600`
und Aufnahmeordner mit `0700`.

Auf dem Rechner mit angeschlossenem Mikrofon:

```bash
python3 tools/record_audio.py list
```

Nimm bevorzugt den angezeigten **numerischen Geräteindex** als `--device`,
falls ein Name gar nicht oder mehrfach vorkommt. Die Liste zeigt auch die vom
Gerät gemeldete Standardrate. Das explizite Mikrofonprofil erzwingt trotzdem
Samson mit 16 kHz und Jieli mit 48 kHz.

Falls nötig:

```bash
python3 -m pip install sounddevice scipy
```

Vor jeder Session einen 10-Sekunden-Pegeltest machen. Bei normaler
Sprechlautstärke und vorgesehener Distanz soll der Peak ungefähr zwischen
`-12` und `-6 dBFS` liegen:

```bash
python3 tools/record_audio.py level-check \
  --device "Samson Q2U" \
  --microphone-profile samson \
  --input-rate 16000
```

Für Jieli entsprechend `--microphone-profile jieli --input-rate 48000`.
Gain oder Abstand korrigieren, wenn das Tool Stille, zu niedrigen Pegel,
Clipping oder starken DC-Offset meldet.

### Samson Q2U

```bash
python3 tools/record_audio.py background \
  --device "Samson Q2U" \
  --microphone-profile samson \
  --input-rate 16000 \
  --minutes 60 \
  --chunk-seconds 20 \
  --prefix bgtrain_samson \
  --output-dir recordings/train_samson \
  --consent-confirmed
```

### Jieli

Jieli wird mit 48 kHz aufgenommen und vom Tool hochwertig bandbegrenzt auf die
benötigten 16 kHz resampelt:

```bash
python3 tools/record_audio.py background \
  --device "USB Composite Device" \
  --microphone-profile jieli \
  --input-rate 48000 \
  --minutes 60 \
  --chunk-seconds 20 \
  --prefix bgtrain_jieli \
  --output-dir recordings/train_jieli \
  --consent-confirmed
```

Empfohlen sind insgesamt 60–90 Minuten je Mikrofon mit:

- ruhigem Raum, Lüfter und HVAC,
- Roboter idle und in Bewegung,
- Greifer, Instrumentenklirren, Tablett, Schritte und Türen,
- eingewilligtem, gestelltem Nebengespräch,
- optional Musik, Durchsagen und Gerätealarmen.

Während dieser Trainingsaufnahmen darf niemand `Robot` in **irgendeinem**
Kontext oder einer Form sagen. Das schließt „the robot“, „robots“, „robotic“
und Ein-Atemzug-Kommandos ein. Sonst würde Sprache mit dem akustischen
Zielmuster fälschlich als Negativbeispiel gelabelt.

Nach jeder Session:

1. Metadaten auf `state=complete`, Pegelwarnungen und die richtige Rate prüfen.
2. Jeden Clip vollständig oder in einer dokumentierten vollständigen
   Hörkontrolle auf Zielwort, private Inhalte, Aussetzer und Clipping prüfen.
3. Kontaminierte oder fehlerhafte Clips nicht importieren, sondern neu
   aufnehmen.

Falls die Aufnahme nicht direkt im persistenten Hub-Verzeichnis erfolgt,
danach ausschließlich die freigegebenen Ordner verpacken:

```bash
tar -czf approved_training_backgrounds.tar.gz \
  -C recordings train_samson train_jieli
```

Dieses zweite Archiv über die Hub-Oberfläche hochladen und im entpackten
Trainingskit wieder unter `recordings/` entpacken:

```bash
mkdir -p recordings
tar -xzf approved_training_backgrounds.tar.gz -C recordings
```

Erst nach dieser Pflichtkontrolle die WAVs aus beiden Aufnahmeordnern auf dem
Hub flach nach `data/background_clips/` kopieren. **Nur WAV-Dateien**, keine
Metadaten, Unterordner oder README-Dateien:

```bash
cp recordings/train_samson/*.wav data/background_clips/
cp recordings/train_jieli/*.wav data/background_clips/
```

## 9. Kalibrierungsdaten separat aufnehmen

Diese Daten niemals nach `data/background_clips/` kopieren.

### Isolierte Wake-Versuche

Mindestens 20, besser 100 pro Mikrofon:

```bash
python3 tools/record_audio.py wake-trials \
  --device "Samson Q2U" \
  --microphone-profile samson \
  --input-rate 16000 \
  --count 20 \
  --seconds 3 \
  --output-dir recordings/calibration_samson_wake \
  --consent-confirmed
```

Für Jieli:

```bash
python3 tools/record_audio.py wake-trials \
  --device "USB Composite Device" \
  --microphone-profile jieli \
  --input-rate 48000 \
  --count 20 \
  --seconds 3 \
  --output-dir recordings/calibration_jieli_wake \
  --consent-confirmed
```

Das Tool öffnet den Audiostream vor dem Hinweis `JETZT`, enthält 0,75 s Vorlauf
und verhindert damit ein abgeschnittenes `R`.

Zusätzlich je Mikrofon mindestens 60 Minuten **separates**, Wake-freies
Nebengespräch aufnehmen. Vollständiges Samson-Beispiel:

```bash
python3 tools/record_audio.py background \
  --device "Samson Q2U" \
  --microphone-profile samson \
  --input-rate 16000 \
  --minutes 60 \
  --chunk-seconds 20 \
  --prefix calbg_samson \
  --output-dir recordings/calibration_samson_background \
  --consent-confirmed
```

Für Jieli `--microphone-profile jieli --input-rate 48000`,
`--prefix calbg_jieli` und
`--output-dir recordings/calibration_jieli_background` verwenden. Diese
Sessions dürfen nicht fürs Training verwendet werden.

Nach der Schwellenwertwahl mit denselben Befehlen, aber Präfix `acceptbg_*` und
Ordnern `recordings/acceptance_*_background`, nochmals neue 60 Minuten
aufnehmen. Auch die 20 Wake-Trials in neue
`recordings/acceptance_*_wake`-Ordner aufnehmen. Ganze Sessions bleiben genau
einem Split zugeordnet; nie benachbarte Chunks zwischen Training,
Kalibrierung und Abnahme verteilen. In der Abnahme mindestens eine zuvor nicht
verwendete Stimme vorsehen.

## 10. Alle Eingaben vollständig prüfen

Nach Downloads, RIRs und Hintergrund-WAVs:

```bash
.venv/bin/python scripts/04_validate_inputs.py \
  --config robot.training.yaml
```

Die Prüfung liest die großen Dateien per Memory Mapping, kontrolliert deren
Shape/Dtype und berechnet standardmäßig nochmals alle SHA-256-Werte. Sie prüft
außerdem:

- feste Git-Commits,
- CUDA-Ausführung des Featuremodells,
- flache WAV-only-Verzeichnisse,
- 16 kHz, mono, PCM16,
- mindestens 60 Minuten Trainingshintergrund **je** Mikrofon anhand der
  Präfixe `bgtrain_samson_` und `bgtrain_jieli_`,
- den vollständigen Satz aus 271 Raumimpulsantworten,
- freien Speicher und Inodes.

Für die kurzen Prüfungen vor jeder Trainingsstufe verwendet der Runner den
schnelleren Bytecount-Modus selbstständig.

## 11. Erst einen nicht deploybaren Smoke-Test ausführen

```bash
bash scripts/05_train.sh smoke all
```

Danach:

```bash
.venv/bin/python scripts/06_validate_onnx.py \
  --model output/robot_smoke/robot_smoke_DO_NOT_DEPLOY.onnx \
  --allow-smoke \
  --report logs/smoke_onnx_validation.json
```

Der Smoke-Test bestätigt nur Setup, TTS, Augmentation, Training und Export.
Sein Modell ist absichtlich `DO_NOT_DEPLOY` benannt.
Er verwendet weiterhin die vollständigen gepinnten Negativ- und
Validierungsfeatures. Dadurch bleiben Download- und RAM-Anforderungen
produktionsnah; nur synthetische Clipzahl und Trainingsdauer sind klein.

## 12. Produktion in drei getrennten GPU-Jobs ausführen

### Stufe A: synthetische Clips erzeugen

```bash
bash scripts/05_train.sh production generate
```

### Stufe B: augmentieren und Features berechnen

```bash
bash scripts/05_train.sh production augment
```

### Stufe C: Klassifikator trainieren und ONNX exportieren

```bash
bash scripts/05_train.sh production train
```

Jede Stufe schreibt ein separates Log unter `logs/` und einen Done-Marker unter
`logs/state/`. Bereits abgeschlossene Stufen werden nicht versehentlich erneut
gestartet. Ein exklusiver Lock verhindert parallele Jobs desselben Modus.
Done-Marker enthalten einen Fingerprint der effektiven Konfiguration,
gepatchten Quellen und Eingaben. Jede Stufe prüft ihre Ausgabedateien, bevor
sie als abgeschlossen gilt.

Nach einem Jobabbruch zuerst im Scheduler prüfen, dass der alte Job wirklich
beendet ist. Dann die betroffene Stufe explizit wiederaufnehmen:

```bash
bash scripts/05_train.sh production augment --recover-stale
```

`generate` behält bei identischem Fingerprint vorhandene Clips und setzt fort.
Partielle Augmentationsfeatures und alte ONNX-Dateien werden recoverbar nach
`output/quarantine/` verschoben. `train` hat upstream keinen zuverlässigen
Checkpoint und startet nach einem Abbruch neu. `--recover-stale` niemals
verwenden, solange noch ein alter Scheduler-Job läuft.

Bei GPU-Out-of-Memory zuerst in der verwendeten YAML:

- `tts_batch_size` von `50` auf `28` oder `14` senken,
- `augmentation_batch_size` von `16` auf `8` senken.

Nicht unter TTS-Batchgröße 7 gehen. Jede Änderung zusammen mit dem Ergebnis
dokumentieren.

Das erwartete Produktionsartefakt ist:

```text
output/robot_training/robot.onnx
```

## 13. ONNX-Kandidaten prüfen

```bash
.venv/bin/python scripts/06_validate_onnx.py \
  --model output/robot_training/robot.onnx \
  --report logs/onnx_validation.json
```

Geprüft werden unter anderem:

- gültiger ONNX-Graph,
- erwarteter Input `[1, 16, 96]`,
- endliche Sigmoid-Ausgaben in `[0, 1]`,
- ONNX-Runtime-Inferenz,
- openWakeWord-Streaming mit Stille,
- Dateigröße und SHA-256.

Der Status bleibt `candidate_unvalidated`.

## 14. Optional offline Schwellenwerte testen

Beispiel Samson:

```bash
.venv/bin/python scripts/07_evaluate_recordings.py \
  --model output/robot_training/robot.onnx \
  --background-dir recordings/calibration_samson_background \
  --wake-dir recordings/calibration_samson_wake \
  --report logs/evaluation_samson.json
```

Für Jieli mit dessen getrennten Ordnern wiederholen. Das Script wertet
`0.50` bis `0.90` in `0.05`-Schritten aus und simuliert vor jedem Test den
Streaming-Vorlauf. Es gibt nur bei mindestens 60 Minuten Hintergrund und 20
Wake-Trials eine Empfehlung aus. Die Offline-Empfehlung ersetzt nicht die
abschließende Live-Prüfung durch den ROS-ASR-Pfad.

## 15. Ergebnisarchiv erzeugen und herunterladen

```bash
bash scripts/08_export_candidate.sh
```

Zurückgegeben wird:

```text
results/robot_candidate_<UTC-Zeit>.tar.gz
results/robot_candidate_<UTC-Zeit>.tar.gz.sha256
```

Dieses kleine Archiv enthält:

- `robot.onnx`,
- Produktions-YAML,
- ONNX-Prüfbericht,
- Asset- und Quellenprovenienz,
- SHA-256-Werte,
- Python-Paketstand,
- Logs und GPU-Informationen.

Lade genau dieses Archiv **und die gleichnamige `.sha256`-Datei** vom GPU-Hub
herunter und übergib beide zurück ins Roboter-Workspace. Nicht das gesamte
17-GB-Arbeitsverzeichnis übertragen.

## 16. Abschließende Live-Abnahme

Erst nach Rücktransfer:

1. Kandidat auf dem Jetson installieren und Paket bauen.
2. Command-Router und Bewegungsnodes nicht starten.
3. `/user_speech` auf ein isoliertes Testtopic remappen.
4. Einen gemeinsamen Schwellenwert für Samson und Jieli bestimmen.
5. Niedrigsten Wert akzeptieren, der in der Kalibrierung null Fehltrigger und
   mindestens 95 % Wake-Erkennung erreicht.
6. Danach auf neuen Abnahmedaten je Mikrofon mindestens 19/20 Wake-Treffer und
   null Aktivierungen in 60 Minuten Wake-freiem Audio nachweisen.
7. Zusätzlich pro Mikrofon und mit mehreren Stimmen, deutschen Akzenten,
   Distanzen, Winkeln, Lautstärken und Robotergeräusch prüfen:
   - Ablehnung von `the robot`, `a robot`, `robots`, `robotic`, `robotics`,
     `robo`, `Robert`, `rowboat` und `Roboter`,
   - Ablehnung von „Robot needle holder“ in einem Atemzug,
   - Akzeptanz von isoliertem „Robot“ mit mindestens 0,35 s Pause,
   - Verhalten knapp unter und knapp über 0,35 s Pause,
   - Befehl nach ungefähr 5,5 s sowie Timeout nach mehr als 6 s,
   - einen schon während der Wake-Verifikation begonnenen Befehl,
   - keine verspätete Verarbeitung alter Sitzungssegmente.
8. Erst dann `robot.model.yaml` auf einen akzeptierten Zustand aktualisieren
   und den aktiven Jetson-Launch freigeben.

## Quellen und Lizenz

Die Pipeline folgt:

- openWakeWord `v0.6.0`,
- dem offiziellen Automatic-Training-Notebook,
- der offiziellen `custom_model.yml`,
- Piper Sample Generator `v2.0.0`,
- der offiziellen PyTorch-2.10-Paarung aus PyTorch und TorchAudio,
- dem für PyTorch 2.10 dokumentierten Legacy-ONNX-Export mit
  `dynamo=False`.

Der openWakeWord-Code ist Apache-2.0. Die empfohlenen vorverarbeiteten
Features und offiziellen Modelle stehen unter CC BY-NC-SA 4.0; das erzeugte
Modell deshalb nicht pauschal als Apache-2.0 deklarieren. Quellen und Hinweise
in `LICENSE_NOTICES.md` und den Manifesten erhalten.
