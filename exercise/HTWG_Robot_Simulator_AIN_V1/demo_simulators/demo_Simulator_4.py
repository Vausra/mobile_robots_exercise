import officeWorld


# Office world erzeugen:
myWorld = officeWorld.buildWorld()

# Aus office world distance grid generieren udn zeichnen:
myGrid = myWorld.getDistanceGrid()
print("distance grid generated")
myGrid.drawGrid()


# Alle Raeume der Office world ausgeben:
print(myWorld.getRooms())

# Simulation schliessen:
myWorld.close()