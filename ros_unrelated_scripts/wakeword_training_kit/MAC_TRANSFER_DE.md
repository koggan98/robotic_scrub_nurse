# Trainingskit über macOS zum GPU-Hub übertragen

Diese Anleitung beschreibt den vollständigen Transportweg:

```text
Linux-Rechner → Mac → Hochschul-VPN → GPU-Hub
GPU-Hub → Mac → Linux-Rechner
```

Der Browser beziehungsweise JupyterLab ist der Standardweg zum GPU-Hub.
`rsync` über SSH ist nur eine Alternative, wenn die Hochschule ausdrücklich
einen SSH-Host für den GPU-Hub bereitstellt.

Wichtig:

- Passwörter niemals in einen Befehl oder eine Datei schreiben. `ssh`, `scp`
  und `rsync` fragen sie bei Bedarf interaktiv ab.
- Die ungefähr 17,5 GB unveränderlichen Trainingsdaten nicht über den Mac
  transportieren. `scripts/02_download_assets.sh` lädt und prüft sie direkt
  auf dem GPU-Hub.
- Das Hochschul-VPN erst einschalten, nachdem die lokalen Dateien vom
  Linux-Rechner auf dem Mac angekommen sind. Manche VPNs sperren den Zugriff
  auf Geräte im lokalen Netzwerk.
- Dateien erst verwenden, wenn ihre SHA-256-Prüfung `OK` meldet.

## 1. Verwendete Platzhalter

In den Befehlen kommen folgende deutlich markierte Platzhalter vor:

- `<LINUX-IP>`: lokale IP-Adresse des Linux-Rechners, zum Beispiel
  `192.168.1.42`
- `<GPU-BENUTZER>`: Hochschul-Benutzername
- `<GPU-SSH-HOST>`: von der Hochschule dokumentierter SSH-Hostname
- `<GPU-SSH-PORT>`: von der Hochschule dokumentierter SSH-Port
- `<UTC-ZEIT>`: Zeitstempel im tatsächlichen Ergebnis-Dateinamen

Die Platzhalter einschließlich `<` und `>` vollständig ersetzen. Den
GPU-SSH-Host nicht aus der Browseradresse erraten.

## 2. Linux-Rechner vorbereiten

Das Trainingskit liegt auf dem Linux-Rechner unter:

```text
/home/mir/robotic_scrub_nurse_ws/files/robot_wakeword_training_kit.tar.gz
/home/mir/robotic_scrub_nurse_ws/files/robot_wakeword_training_kit.tar.gz.sha256
```

Auf dem Linux-Rechner Benutzer, lokale IP und SSH-Dienst prüfen:

```bash
whoami
hostname -I
systemctl is-active ssh
```

`whoami` soll hier `mir` anzeigen. Bei mehreren IP-Adressen normalerweise die
Adresse des lokalen WLAN- oder Ethernet-Netzes verwenden.

Falls `systemctl` meldet, dass der SSH-Dienst fehlt oder nicht läuft:

```bash
sudo apt update
sudo apt install openssh-server
sudo systemctl enable --now ssh
```

Keine Portweiterleitung am Router einrichten. Der Transfer findet nur im
lokalen, vertrauenswürdigen Netzwerk statt.

## 3. Trainingskit vom Linux-Rechner auf den Mac holen

Mac und Linux-Rechner mit demselben lokalen Netz verbinden. Das
Hochschul-VPN auf dem Mac noch ausgeschaltet lassen.

Im Terminal des Macs:

```bash
mkdir -p "$HOME/Downloads/robot-wakeword-transfer"
```

Archiv und zugehörige Prüfsumme holen:

```bash
scp mir@<LINUX-IP>:/home/mir/robotic_scrub_nurse_ws/files/robot_wakeword_training_kit.tar.gz \
  "$HOME/Downloads/robot-wakeword-transfer/"

scp mir@<LINUX-IP>:/home/mir/robotic_scrub_nurse_ws/files/robot_wakeword_training_kit.tar.gz.sha256 \
  "$HOME/Downloads/robot-wakeword-transfer/"
```

Beim ersten Verbindungsaufbau zeigt SSH einen Host-Fingerprint. Nur bestätigen,
wenn die IP tatsächlich zum eigenen Linux-Rechner gehört. Danach fragt `scp`
gegebenenfalls interaktiv nach dem Linux-Passwort.

Auf dem Mac prüfen:

```bash
cd "$HOME/Downloads/robot-wakeword-transfer"
shasum -a 256 -c robot_wakeword_training_kit.tar.gz.sha256
```

Erwartetes Ergebnis:

```text
robot_wakeword_training_kit.tar.gz: OK
```

Bei `FAILED` das Archiv nicht hochladen. Die fehlerhafte Mac-Kopie löschen,
erneut übertragen und wieder prüfen.

Das Archiv auf dem Mac nicht mit Finder entpacken und neu komprimieren.
Dadurch können zusätzliche `.DS_Store`- oder `._*`-Dateien entstehen.

## 4. Standardweg: über Browser oder JupyterLab hochladen

Jetzt das Hochschul-VPN auf dem Mac verbinden und den GPU-Hub im Browser
öffnen.

In einem persistenten Projektverzeichnis über die Upload-Funktion diese beiden
Dateien auswählen:

```text
robot_wakeword_training_kit.tar.gz
robot_wakeword_training_kit.tar.gz.sha256
```

Beide liegen auf dem Mac in:

```text
~/Downloads/robot-wakeword-transfer/
```

Im Terminal des GPU-Hubs in den Uploadordner wechseln. Falls der genaue Ort
unklar ist, kann die Datei im eigenen Home-Verzeichnis gesucht werden:

```bash
find "$HOME" -maxdepth 4 -type f \
  -name 'robot_wakeword_training_kit.tar.gz'
```

Mit `cd` in den angezeigten Ordner wechseln und die hochgeladene Datei prüfen:

```bash
sha256sum -c robot_wakeword_training_kit.tar.gz.sha256
```

Für das Training einen persistenten Speicherort mit mindestens 80 GB freiem
Platz wählen. Falls `$HOME/wakeword` diese Anforderungen erfüllt:

```bash
mkdir -p "$HOME/wakeword"
tar -xzf robot_wakeword_training_kit.tar.gz -C "$HOME/wakeword"
cd "$HOME/wakeword/robot_wakeword_training_kit"
```

Falls der Hub einen anderen großen Projekt- oder Scratch-Pfad vorgibt, dort
entpacken und diesen Pfad für alle weiteren Schritte verwenden. Nicht in
einem automatisch gelöschten `/tmp` trainieren.

Ab hier mit
[`START_HERE_DE.md`](START_HERE_DE.md) fortfahren. Die großen offiziellen
Trainingsdaten bleiben dabei auf dem GPU-Hub:

```bash
bash scripts/01_setup.sh
bash scripts/02_download_assets.sh
```

## 5. Alternative: GPU-Hub mit offiziellem SSH-Zugang

Diesen Abschnitt nur verwenden, wenn die Hochschule einen
`<GPU-SSH-HOST>`, `<GPU-BENUTZER>` und gegebenenfalls
`<GPU-SSH-PORT>` dokumentiert hat.

Nach Aktivierung des Hochschul-VPN die Verbindung auf dem Mac testen:

```bash
ssh -p <GPU-SSH-PORT> <GPU-BENUTZER>@<GPU-SSH-HOST>
```

Beim ersten Verbindungsaufbau den Host-Fingerprint mit der offiziellen
Hochschuldokumentation vergleichen.

Das Trainingskit mit fortsetzbarer Übertragung hochladen:

```bash
rsync -avP -e "ssh -p <GPU-SSH-PORT>" \
  "$HOME/Downloads/robot-wakeword-transfer/robot_wakeword_training_kit.tar.gz" \
  "<GPU-BENUTZER>@<GPU-SSH-HOST>:~/"

rsync -avP -e "ssh -p <GPU-SSH-PORT>" \
  "$HOME/Downloads/robot-wakeword-transfer/robot_wakeword_training_kit.tar.gz.sha256" \
  "<GPU-BENUTZER>@<GPU-SSH-HOST>:~/"
```

Bei SSH-Port 22 kann stattdessen jeweils `-e "ssh -p 22"` verwendet werden.
Nach einem VPN- oder Netzwerkabbruch denselben `rsync -avP`-Befehl erneut
ausführen; die Teildatei wird weiterübertragen.

Danach auf dem GPU-Hub:

```bash
cd "$HOME"
sha256sum -c robot_wakeword_training_kit.tar.gz.sha256
mkdir -p "$HOME/wakeword"
tar -xzf robot_wakeword_training_kit.tar.gz -C "$HOME/wakeword"
cd "$HOME/wakeword/robot_wakeword_training_kit"
```

## 6. Freigegebene Audioaufnahmen über den Mac übertragen

`START_HERE_DE.md` beschreibt Aufnahme, Datenschutz und Pflichtkontrolle. Nur
die danach freigegebenen Trainingshintergründe verpacken. Das dort erzeugte
Archiv heißt:

```text
approved_training_backgrounds.tar.gz
```

Im Ordner, in dem dieses Archiv liegt, auf dem Linux-Rechner zusätzlich eine
Prüfsumme erzeugen:

```bash
sha256sum approved_training_backgrounds.tar.gz \
  > approved_training_backgrounds.tar.gz.sha256
```

Das Hochschul-VPN auf dem Mac ausschalten. Danach vom Mac aus beide Dateien
holen; `<PFAD-ZUM-AUDIOARCHIV>` durch den tatsächlichen absoluten Linux-Pfad
ohne Dateinamen ersetzen:

```bash
rsync -avP \
  mir@<LINUX-IP>:<PFAD-ZUM-AUDIOARCHIV>/approved_training_backgrounds.tar.gz \
  "$HOME/Downloads/robot-wakeword-transfer/"

rsync -avP \
  mir@<LINUX-IP>:<PFAD-ZUM-AUDIOARCHIV>/approved_training_backgrounds.tar.gz.sha256 \
  "$HOME/Downloads/robot-wakeword-transfer/"
```

Auf dem Mac prüfen:

```bash
cd "$HOME/Downloads/robot-wakeword-transfer"
shasum -a 256 -c approved_training_backgrounds.tar.gz.sha256
```

Danach das Hochschul-VPN wieder verbinden. Archiv und Prüfsumme über die
Browser-Oberfläche hochladen oder – falls vorhanden – wie in Abschnitt 5 mit
`rsync -avP` übertragen.

Auf dem Hub beide Dateien in denselben Ordner legen und prüfen:

```bash
sha256sum -c approved_training_backgrounds.tar.gz.sha256
```

Im entpackten Trainingskit:

```bash
mkdir -p recordings
tar -xzf /PFAD/ZUM/UPLOAD/approved_training_backgrounds.tar.gz \
  -C recordings
```

Anschließend die Pflichtkontrolle und den in `START_HERE_DE.md` beschriebenen
Import ausführen. `data/background_clips/` muss flach sein und ausschließlich
freigegebene WAV-Dateien enthalten.

## 7. Modell vom GPU-Hub auf den Mac zurückholen

Nach Training und Export liegt im Kit-Verzeichnis unter `results/` ein
Archiv mit einem Zeitstempel, beispielsweise:

```text
robot_candidate_20260724T153000Z.tar.gz
```

Den tatsächlichen Namen auf dem Hub anzeigen:

```bash
cd "$HOME/wakeword/robot_wakeword_training_kit/results"
ls -lh
```

`scripts/08_export_candidate.sh` hat bereits die gleichnamige
`.tar.gz.sha256`-Datei erzeugt. Falls eine der beiden Dateien fehlt, den
Exportlauf nicht durch eine selbst gebaute Teillösung ersetzen, sondern den
Fehler im Exportlauf beheben und ihn erneut ausführen.

### Rückweg über Browser oder JupyterLab

In der Dateiansicht des GPU-Hubs diese beiden Dateien herunterladen:

```text
robot_candidate_<UTC-ZEIT>.tar.gz
robot_candidate_<UTC-ZEIT>.tar.gz.sha256
```

Beide auf dem Mac in diesen Ordner legen:

```text
~/Downloads/robot-wakeword-results/
```

Falls er noch nicht existiert:

```bash
mkdir -p "$HOME/Downloads/robot-wakeword-results"
```

### Rückweg über SSH und `rsync`

Bei vorhandenem offiziellen SSH-Zugang und aktivem Hochschul-VPN:

```bash
mkdir -p "$HOME/Downloads/robot-wakeword-results"

rsync -avP -e "ssh -p <GPU-SSH-PORT>" \
  "<GPU-BENUTZER>@<GPU-SSH-HOST>:~/wakeword/robot_wakeword_training_kit/results/robot_candidate_<UTC-ZEIT>.tar.gz" \
  "$HOME/Downloads/robot-wakeword-results/"

rsync -avP -e "ssh -p <GPU-SSH-PORT>" \
  "<GPU-BENUTZER>@<GPU-SSH-HOST>:~/wakeword/robot_wakeword_training_kit/results/robot_candidate_<UTC-ZEIT>.tar.gz.sha256" \
  "$HOME/Downloads/robot-wakeword-results/"
```

Auf dem Mac prüfen:

```bash
cd "$HOME/Downloads/robot-wakeword-results"
shasum -a 256 -c robot_candidate_<UTC-ZEIT>.tar.gz.sha256
```

Nur bei `OK` weiterarbeiten.

## 8. Ergebnis vom Mac zurück auf den Linux-Rechner kopieren

Das Hochschul-VPN wieder ausschalten. Dann im Mac-Terminal:

```bash
scp "$HOME/Downloads/robot-wakeword-results/robot_candidate_<UTC-ZEIT>.tar.gz" \
  mir@<LINUX-IP>:/home/mir/robotic_scrub_nurse_ws/files/

scp "$HOME/Downloads/robot-wakeword-results/robot_candidate_<UTC-ZEIT>.tar.gz.sha256" \
  mir@<LINUX-IP>:/home/mir/robotic_scrub_nurse_ws/files/
```

Auf dem Linux-Rechner abschließend prüfen:

```bash
cd /home/mir/robotic_scrub_nurse_ws/files
sha256sum -c robot_candidate_<UTC-ZEIT>.tar.gz.sha256
```

Danach liegen Modellkandidat, Protokolle und Herkunftsnachweise im Workspace
für die weitere Prüfung bereit.

## 9. Wenn der Mac den Linux-Rechner nicht erreicht

Zuerst prüfen:

- Mac und Linux-Rechner sind im selben lokalen Netz.
- Hochschul-VPN auf dem Mac ist ausgeschaltet.
- `<LINUX-IP>` ist die aktuelle lokale IP.
- `systemctl is-active ssh` meldet auf Linux `active`.

Als Alternative kann auf dem Mac unter
**Systemeinstellungen → Allgemein → Teilen → Entfernte Anmeldung** SSH
vorübergehend eingeschaltet werden. Die WLAN-IP des Macs anzeigen:

```bash
ipconfig getifaddr en0
```

Dann vom Linux-Rechner aus übertragen:

```bash
scp /home/mir/robotic_scrub_nurse_ws/files/robot_wakeword_training_kit.tar.gz \
  <MAC-BENUTZER>@<MAC-IP>:/Users/<MAC-BENUTZER>/Downloads/

scp /home/mir/robotic_scrub_nurse_ws/files/robot_wakeword_training_kit.tar.gz.sha256 \
  <MAC-BENUTZER>@<MAC-IP>:/Users/<MAC-BENUTZER>/Downloads/
```

Nach dem Transfer „Entfernte Anmeldung“ auf dem Mac wieder ausschalten.
Alternativ können die beiden Dateien über einen exFAT-formatierten USB-Stick
transportiert werden.

## 10. Häufige Fehler

### `Connection refused`

Der SSH-Dienst läuft auf dem Ziel nicht oder der Port ist falsch. Auf Linux
`systemctl is-active ssh` prüfen. Beim GPU-Hub ausschließlich die offiziellen
SSH-Angaben der Hochschule verwenden.

### `Operation timed out` oder `No route to host`

IP-Adresse oder Netzwerk stimmen nicht, oder das Hochschul-VPN blockiert das
lokale Netz. Für Linux ↔ Mac den VPN ausschalten; für Mac ↔ GPU-Hub den VPN
einschalten.

### `Permission denied`

Benutzername, Passwort oder Zielordner stimmen nicht. Auf Linux ist der
Benutzer in dieser Anleitung `mir`. Passwörter nur in die interaktive Abfrage
eingeben.

### `zsh: no matches found`

Eine Wildcard wurde vom Mac ausgewertet. Den exakten Dateinamen verwenden
oder den vollständigen Remote-Ausdruck in doppelte Anführungszeichen setzen.

### `No such file or directory`

Prüfen, ob alle Platzhalter ersetzt wurden. Pfade mit Leerzeichen immer in
doppelte Anführungszeichen setzen.

`"~/Downloads"` ist falsch, weil `~` innerhalb von Anführungszeichen nicht
expandiert wird. Deshalb wird in dieser Anleitung
`"$HOME/Downloads"` verwendet.

### Prüfsumme meldet `FAILED`

Die Datei nicht entpacken oder verwenden. Die fehlerhafte Zielkopie löschen,
nochmals übertragen und erneut prüfen.

### VPN oder Netzwerk bricht bei einer großen Datei ab

Genau denselben `rsync -avP`-Befehl erneut ausführen. Bei Browseruploads kann
eine Wiederaufnahme fehlen; große Audioarchive deshalb bevorzugt über den
offiziellen SSH-Zugang übertragen, falls dieser angeboten wird.

### Browser benennt einen Download zu `(1)` um

In einen leeren Zielordner herunterladen oder Datei und
Prüfsummendatei wieder auf ihre exakten ursprünglichen Namen bringen, bevor
`shasum -a 256 -c` ausgeführt wird.

### `Host key verification failed`

Den bekannten Schlüssel nicht blind löschen. Zuerst klären, ob der Rechner
oder Hochschulhost tatsächlich neu installiert wurde, und den neuen
Fingerprint über eine vertrauenswürdige Quelle prüfen.

### Der Hub hat nur eine Jupyter-Oberfläche

Dann kann `scp` nicht verwendet werden, solange die Hochschule keinen
separaten SSH-Endpunkt nennt. Die Browser-Upload- und Downloadfunktionen sind
in diesem Fall der richtige Weg.
