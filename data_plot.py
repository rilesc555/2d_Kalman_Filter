#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 15:35:36 2024

@author: riley
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

plt.close("all")
plt.ion()
kdf = pd.read_csv("cmake-build-debug/output.csv")
rdf = pd.read_csv("cmake-build-debug/tracks.csv")
rdfID = rdf[rdf["track_id"] == kdf.at[0,"ID"]]


raz, kaz = rdfID["az"], kdf["az"]
rel, kel = rdfID["el"], kdf["el"]

time_range = pd.Series(range(0, len(kaz)))

plt.plot(time_range, raz)
plt.plot(time_range, kaz)


azmse = np.mean((np.array(raz)-np.array(kaz)) ** 2)
elmse = np.mean((np.array(rel) - np.array(kel)) ** 2)

print("AZ MSE: ", azmse)
print("EL MSE: ", elmse)






