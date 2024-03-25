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
rdfID = rdf[rdf["track_id"] == kdf.at[1,"ID"]]

print(rdf.track_id.mode())


plt.xlabel("Azimuth")
plt.ylabel("Elevation")


raz, kaz = rdfID["az"], kdf["az"]
rel, kel = rdfID["el"], kdf["el"]

plt.plot(raz, rel, label = "Radar Input")
plt.plot(kaz, kel, label = "Kalman Output")
