import numpy as np
import pandas as pd
import os

def save_csv(df, folder, filename):
    os.makedirs(folder, exist_ok=True)

    base, ext = os.path.splitext(filename)
    path = os.path.join(folder, filename)

    i = 1
    while os.path.exists(path):
        path = os.path.join(folder, f"{base}_{i}{ext}")
        i += 1

    df.to_csv(path, index=False)
    return path
