#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 15:35:36 2024

@author: riley
"""

import pandas as pd
import matplotlib.pyplot as plt
plt.close("all")
kdf = pd.read_csv("cmake-build-debug/output.csv")
rdf = pd.read_csv("cmake-build-debug/tracks.csv")
rdf120 = rdf[rdf["track_id"] == 120]


plt.xlabel("Azimuth")
plt.ylabel("Elevation")


raz, kaz = rdf120["az"], kdf["az"]
rel, kel = rdf120["el"], kdf["el"]

plt.plot(raz, rel, label = "Radar Input")
plt.plot(kaz, kel, label = "Kalman Output")
