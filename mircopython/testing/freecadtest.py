import FreeCAD as App
import Part
import Gear

doc = App.newDocument("PlanetaryGear")

# --- Parameters ---
sun_teeth = 12
planet_teeth = 18
ring_teeth = sun_teeth + 2 * planet_teeth
module = 1.0
num_planets = 3

# --- Create Sun Gear ---
sun = Gear.makeInvoluteGear(
    numTeeth=sun_teeth, 
    module=module, 
    pressureAngle=20, 
    width=5
)
sun.Label = "Sun"
sun.Placement.Base = App.Vector(0, 0, 0)

# --- Create Ring Gear (internal) ---
ring = Gear.makeInvoluteGear(
    numTeeth=ring_teeth, 
    module=module, 
    pressureAngle=20, 
    width=5,
    internal=True
)
ring.Label = "Ring"
ring.Placement.Base = App.Vector(0, 0, 0)

# --- Create Planet Gears ---
import math

planet_radius = module * (sun_teeth + planet_teeth) / 2

for i in range(num_planets):
    angle = i * 360.0 / num_planets
    rad = math.radians(angle)
    x = planet_radius * math.cos(rad)
    y = planet_radius * math.sin(rad)

    planet = Gear.makeInvoluteGear(
        numTeeth=planet_teeth, 
        module=module, 
        pressureAngle=20, 
        width=5
    )
    planet.Label = f"Planet_{i+1}"
    planet.Placement.Base = App.Vector(x, y, 0)

App.ActiveDocument.recompute()
