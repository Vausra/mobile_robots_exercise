HTWG_Robot_Simulator_AIN_V1.0
05.11.2017
Oliver Bittel
Fakultät Informatik


Umgebung:
=========
Module wurden mit Python 3.5 getestet und
der Entwicklungsumgebung
PyCharm Community Edition 2016.2.3.


Installierte Pakete:
numpy (zum Rechnen mit Vekoren und Matrizen)
matplotlib (nützlich, falls Graphiken geplottet werden sollen; ähnlich wie plot von matlab)


Benutzung:
==========
1. demo_xxx:
demo_xxx sind kleine Hauptprogramme, die vorhandene Klassen testen.


2. graphics.py:
graphics.py ist ein kleines Paket zum Zeichnen von Graphikobjekten.
Dokumentation siehe graphics.pdf.
Das Paket stammt von http://mcsp.wartburg.edu/zelle/python/


3. Robot.py und World.py:
Diese beiden Klassen bilden den eigentlichen Simulator.
Eine Simulation besteht aus genau einem Objekt myRobot der Klasse Robot und einem
Objekt myWorld der Klasse World.
myRobot wird in der Welt myWorld mit dem Aufruf
myWorld.setRobot(myRobot,[x,y,theta]) positioniert.

Der Roboter lässt sich über myRobot.move() steuern.
Über myRobot.sense() bzw. myRobot.senseBoxes() können
die Sensordaten abgefragt werden.
Siehe demo_Simulator_1.py


4. CursorController.py:
Der Roboter kann manuell (zu Demozwecken) über die Cursortasten gesteuert werden.
Siehe demo_Simulator_2.py


5. xxxWorld.py
Verschiedene bereits vordefinierte Welten.


6. OccupancyGrid.py:
Klasse für Belegtheitsgitter.
Mit der Methode World.getOccupancyGrid() kann aus den
Weltdaten ein Belegtheitsgitter erzeugt werden.
Mit der Methode World.extendToDistanceGrid() kann ein
Belegtheitsgitter zu einem Distanzgitter erweitert werden.
Siehe demo_Simulator_4.py.

7. SensorUtilities
Enthält Funktionen um aus Sensordaten in Polarkoordinaten
einen Liste von Liniensegmente zu extrahieren.
Siehe demo_Simulator_5.py.
