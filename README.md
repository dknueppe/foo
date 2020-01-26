# Überblick über die Softwarearchitektur


## Ziele und Anforderungen

Die Software muss in der Lage sein mit vielen Sensoren un Aktoren gleichzeitig
kommunizieren zu können. Desweiteren wird dieses Projekt auch als Basis für weitere
Arbeiten im Labor genutzt, weshalb es auch erweiterbar und wartbar sein muss.
Hierfür gibt es mehrere abstrakte Basisklassen welche später genauer erläutert
werden sollen. Diese garantieren ein großes Maß and Modularität, sowie
Datenkonsistenz in nebenläufigen Programmen. Für die Kommunikation zwischen den
periphärie Geräten, sowie die Datenverarbeitung kommen Entwurfsmuster zum Einsatz um
eine hohe Flexibilität zu gewährleisten.


## Projektstruktur


### autonomous-vehicle

Dies ist der Quellenordner, welcher auch im hiesigen Gitlab eingecheckt ist, alle
weiteren Ordner sind Unterordner.

Hier finden wir das **navigate.py** Script, mit dem alles gestartet wird. Desweiteren
sind hier Module zu finden, welche nützliche Funktionen zur Verfügung stellen, um den
Roboter erfolgreich navigieren zu können.

### doc

Dieser Ordner enthält sämtliche Dokumentationen zum Projekt, wie zum Beispiel dieses
Dokument. Allerdings auch aufgenommene Messungen und Skripte für die Auswertung.

### platforms

Dieser Ordner enthält das Modul **platform** mit den Basisklassen **Drive_train**,
**Command**, sowie der Klasse **Platform**. Die Platform-Klasse verwaltet den Zugriff
auf die Aktuatoren des Roboters und hat dafür eine Befehlswarteschlange _commands_,
an welche Befehle zur Ausführung angehangen werden können. Desweiteren befinden sich
in diesem Ordner auch alle Module, die die jeweiligen Aktuatoren eines Roboters
bedienen können. Gegebenen Falls sind hier auch noch Bibliotheken und Code von
Drittparteien oder Herstellern.

### sensors

Dieser Ordner enthält das Modul **sensor** mit der Basisklasse **Sensor**. Ausserdem
sind hier alle Module, die für das Auslesen von Sensoren genutzt werden, sowie
nach bedarf Bibliotheken und Code von Herstellern.

### strategies

Hier liegen die Strategien, also die Algorithmen der Datenverarbeitung der einzelnen
Gruppen, zwischen denen nach Bedarf gewechselt werden kann.


## Basisklassen

Basisklassen sind ein Werkzeug zur Abstraktion, indem sie gemeinsame Schnittstellen
schaffen, und gewisse Ramenbedingungen garantieren. Man kann sie nicht
instanzieieren und **muss** von ihnen Erben, die entsprechenden Funktionalitäten in
Konformität zu unserem Programm zu implementieren.

Zur Zeit gibt es drei Basisklassen, von welchen man Erben kann.

* **Sensor**, diese Klasse bietet threading sowie sichere Kommunikation über das
_data_ Feld.

* **Drive_train**, diese Klasse bietet eine Schnittstelle für Aktuatoren.

* **Command**, diese Klasse bietet eine Schnittstelle für die Platform, um den
Roboter asynchron steuern zu können. In tandem mit der Drive_train Klasse wird hier
das Kommando-Entwurfsmuster umgesetzt.

## Strategien

### Indoornavigation

Die erläuterung zu unserer Strategie für die Navigation im Innenbereich:

Zuerst werden die Übergebenen Referenzen zu den Sensor und Aktuator und Platform
Objekten gespeichert, im Anschluss werden Logdateien angelegt.

```py3
def __init__(self, sensors, actuators, platform):
    self.sensors = sensors
    self.actuators = actuators
    self.platform = platform
    self.pos = np.zeros(3)
    date = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
    dir = 'doc/measurements/indoor/'
    self.uwb_nr_log_file =  open('{}uwb_nr_log_{}.txt'.format(dir, date), 'a+')
    self.uwb_dwm_log_file = open('{}uwb_dwm_log_{}.txt'.format(dir, date), 'a+')
    self.uwb_raw_log_file = open('{}uwb_raw_log_{}.txt'.format(dir, date), 'a+')
    self.lidar_log_file =   open('{}lidar_log_{}.txt'.format(dir, date), 'a+')
```

Nun kommen wir zur eigentlichen Datenverarbeitung. Hier muss man leider sagen,
das nicht alles fertig geworden ist, und deshalb bis hierhin nur geloggt wurde,
und noch per Hand gesteuert wird.

Zuerst werden die aktuellen Messwerte abgefragt, anschließend werden sie geloggt
und weiter Verarbeitet. Die Messwerte werden als dictionary übergeben, danach wird
versucht zu jedem gefundenen Anker die Position aus einem dictionary mit bekannten
Ankerpositionen zu finden, um dann das resultierende Gleichungssystem nach dem
Newton-Raphson Verfahren zu lösen.

```python
anchor_pos = {0xDCAB :(-3010, 2549 ,  2313),
              0x40AB :(-2976, 7398 ,  2335),
              0x8083 :( 4298, 7333 ,  2685),
              0x8003 :( 4283, 2459 ,  2678),
              0x8DB1 :( 1550, 11713,  2322)}

def do_stuff(self):
    uwb_dists = self.sensors[1].data
    lidar_data = self.sensors[2].data
    dists = []
    ranging_anchor_pos = []
    if uwb_dists is not None:
        self.uwb_raw_log_file.write("{}\n".format(uwb_dists))
    if uwb_dists is not None and uwb_dists['qf'] > 70:
        for addr in uwb_dists:
            try:
                ranging_anchor_pos.append(anchor_pos[addr])
                dists.append(uwb_dists[addr])
            except KeyError:
                pass
        try:
            pos = mlat.newton_raphson(ranging_anchor_pos, dists, self.pos)
            self.uwb_nr_log_file.write("{}\n".format(pos))
            self.pos = pos
        except:
            pass
        self.uwb_dwm_log_file.write("{}\n".format(list(uwb_dists['position'])))
```
