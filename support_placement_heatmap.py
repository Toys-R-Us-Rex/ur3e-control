import json

from matplotlib import pyplot as plt
from matplotlib.patches import Wedge

import numpy as np

import seaborn as sns

def main():
    results_path: str = "/tmp/results3_.json"
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
            pts.append((x, y, z, ratio, rot))
    
    pts = np.array(pts)
    
    import pandas as pd
    base_df = pd.DataFrame(pts, columns=["x", "y", "z", "ratio", "angle"])
    
    for i in range(4):
        df = base_df[base_df["angle"] == i]
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        
        table_x = [-length / 2, length / 2, length / 2, -length / 2, -length / 2]
        table_y = [-width, -width, 0, 0, -width]
        table_z = [0, 0, 0, 0, 0]
        ax.plot_surface(
            np.array([[- length / 2, length / 2], [-length / 2, length / 2]]),
            np.array([[-width, -width], [0, 0]]),
            np.zeros((2, 2)),
            alpha=0.1, color='blue'
        )
        angles = np.linspace(min_angle, max_angle, 100)
        for r in np.linspace(min_radius, max_radius, 10):
            ax.plot(r * np.cos(angles), r * np.sin(angles), 0,
                    color='red', alpha=0.2)

        s = ax.scatter(df["x"], df["y"], df["z"], c=df["ratio"], cmap="viridis", vmin=0, vmax=1)
        plt.colorbar(s, label="Ratio")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        
        plt.title(f"Support placement scores (angle={i * 90})")
        plt.tight_layout()

        plt.show()
    
    #######
    axis_pairs = [("x", "y"), ("x", "z"), ("y", "z")]
    n_bins = 80

    df_binned = base_df.copy()
    bin_labels = {}
    for col in ["x", "y", "z"]:
        df_binned[col], bins = pd.cut(base_df[col], bins=n_bins, labels=False, retbins=True)
        midpoints = (bins[:-1] + bins[1:]) / 2
        bin_labels[col] = [f"{m:.2f}" for m in midpoints]

    every_n = 8
    for col_a, col_b in axis_pairs:
        pivot = df_binned.pivot_table(values="ratio", index=col_b, columns=col_a, aggfunc="mean")
        fig, ax = plt.subplots(figsize=(6, 5))
        x_labels = [l if i % every_n == 0 else "" for i, l in enumerate(bin_labels[col_a])]
        y_labels = [l if i % every_n == 0 else "" for i, l in enumerate(bin_labels[col_b])]
        sns.heatmap(
            pivot,
            cmap="viridis",
            vmin=0, vmax=1,
            ax=ax,
            square=True,
            xticklabels=x_labels, yticklabels=y_labels
        )
        ax.tick_params(axis="x", rotation=45)
        ax.tick_params(axis="y", rotation=0)
        ax.set_title(f"Mean ratio over {col_a.upper()}/{col_b.upper()}")
        plt.tight_layout()
        plt.show()
    #######
    
    sns.scatterplot(base_df, x="z", y="ratio")
    plt.show()
    
    g = sns.FacetGrid(base_df, col="angle", col_wrap=2, height=4, aspect=1.2)
    g.map(sns.histplot, "ratio", kde=True)
    
    g.set_axis_labels("Ratio", "Frequency")
    g.set_titles("Distribution for angle {col_name}")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
