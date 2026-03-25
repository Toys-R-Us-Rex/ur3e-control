import json

from matplotlib import pyplot as plt
from matplotlib.patches import Wedge

import numpy as np

import seaborn as sns

def main():
    results_path: str = "support_placement_results.json"
    length: float = 1.1
    width: float = 0.7
    min_radius: float = 0.20
    max_radius: float = 0.60
    min_angle: float = -np.pi
    max_angle: float = 0
    pts = []
    
    print("Loading results...")
    with open(results_path, "r") as f:
        results = json.load(f)
        print(f"Loading {len(results)} test results")
        for result in results:
            x, y, z = result["pos"]
            ratio = result["ratio"]
            rot = int(result["angle"] // 90)
            pts.append((x, y, ratio, rot))
    
    pts = np.array(pts)
    
    import pandas as pd
    df = pd.DataFrame(pts, columns=["x", "y", "ratio", "angle"])
    
    fig, ax = plt.subplots()
    table = plt.Rectangle((-length / 2, -width), length, width, alpha=0.1)
    ax.add_patch(table)
    
    reach = Wedge(
        (0, 0),
        max_radius,
        np.degrees(min_angle),
        np.degrees(max_angle),
        width=max_radius - min_radius,
        facecolor="red",
        alpha=0.2
    )
    ax.add_patch(reach)
    
    s = ax.scatter(df["x"], df["y"], c=df["ratio"], cmap="viridis", vmin=0, vmax=1)
    plt.colorbar(s, label="Ratio")
    plt.show()
    
    g = sns.FacetGrid(df, col="angle", col_wrap=2, height=4, aspect=1.2)
    g.map(sns.histplot, "ratio", kde=True)
    
    g.set_axis_labels("Ratio", "Frequency")
    g.set_titles("Distribution for angle {col_name}")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
