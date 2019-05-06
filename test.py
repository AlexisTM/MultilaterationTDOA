#!/usr/bin/env python
# -*- coding: utf-8 -*-

from multilateration_tdoa import Engine
from math import sqrt

engine = Engine(goal=[None, None, None]) # To fix that the resulting height should be 2 meters
# P=multilateration.Project() # for 3D, simply do not set a goal
engine.add_anchor('anchor_A',(1,0,1))
engine.add_anchor('anchor_B',(-1,0,1))
engine.add_anchor('anchor_C',(1,1,1))
engine.add_measure_id('anchor_A',sqrt(2))
engine.add_measure_id('anchor_B',sqrt(2))
engine.add_measure_id('anchor_C',sqrt(3))
print(engine.solve())

# Or using anchors linked to measurements directly
engine2 = Engine(goal=[None, None, 2.0])
# If the ID is not given, it will be generated to str(position_of_anchor)
engine2.add_measure((1,0,1), sqrt(2), ID="anchor_A")
engine2.add_measure((-1,0,1), sqrt(2), ID="anchor_B")
engine2.add_measure((1,1,1), sqrt(3))

print(engine2.solve())

engine3 = Engine(goal=[None, None, None]) # To fix that the resulting height should be 2 meters
# P=multilateration.Project() # for 3D, simply do not set a goal
engine3.add_anchor('anchor_A',(1,0,1))
engine3.add_anchor('anchor_B',(-1,0,1))
engine3.add_anchor('anchor_C',(1,1,1))
engine3.add_anchor('anchor_D',(-1,-1,-1))
engine3.add_measure_id('anchor_A',sqrt(2))
engine3.add_measure_id('anchor_B',sqrt(2))
engine3.add_measure_id('anchor_C',sqrt(3))
engine3.add_measure_id('anchor_D',sqrt(3))
print(engine3.solve())
