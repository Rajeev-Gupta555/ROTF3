import numpy as np
from stl import mesh

# Using an existing closed stl file:
your_mesh = mesh.Mesh.from_file('/home/rajeev-gupta/catkin_ws/src/rotf3/mesh/base.stl')
v1, v2, v3, v4 = your_mesh.get_mass_properties_with_density(1)
print(v1)
print(v2)
print(v3)
print(v4)
volume, cog, inertia = your_mesh.get_mass_properties()
print("mass =                                  = {0}".format(Mass))
print("Volume                                  = {0}".format(volume))
print("Position of the center of gravity (COG) = {0}".format(cog))
print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
print("                                          {0}".format(inertia[1,:]))
print("                                          {0}".format(inertia[2,:]))