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


    
    
    
rdf = pd.read_csv("cmake-build-debug/tracks.csv")


for i in range(10):
    filename = "cmake-build-debug/191/output" + str(i) + ".csv"
    kdf = pd.read_csv(filename)
    rdfID = rdf[rdf["track_id"] == kdf.at[0,"ID"]]
    raz, kaz = rdfID["az"], kdf["az"]
    rel, kel = rdfID["el"], kdf["el"]
    time_range = pd.Series(range(0, len(kaz)))
    
    plt.figure(i)
    plt.title(filename)
    plt.subplot(1,2,1)
    plt.plot(time_range, raz)
    plt.plot(time_range, kaz)
    plt.title("Azimuth")
    
    plt.subplot(1,2,2)
    plt.plot(time_range, rel)    
    plt.plot(time_range, kel)
    plt.title("Elevation")
    
    azmse = np.mean((np.array(raz)-np.array(kaz)) ** 2)
    elmse = np.mean((np.array(rel) - np.array(kel)) ** 2)
    
    print("With Q at 0." + str(i) + ":")
    print("AZ MSE: ", azmse)
    print("EL MSE: ", elmse, "\n")
    
plt.show()









