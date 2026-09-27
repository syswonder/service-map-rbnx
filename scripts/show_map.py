#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Display or save a PRBonn occupancy map in its world-axis orientation."""

from pathlib import Path
import argparse

import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    args = parser.parse_args()

    dataset = args.dataset_dir.expanduser().resolve()

    occmap = np.load(dataset / "occmap.npy")

    print("shape:", occmap.shape)
    print("min:", occmap.min())
    print("max:", occmap.max())

    # PRBonn's map convention is effectively [x, y].
    # Transpose it so matplotlib displays x horizontally and y vertically.
    image = occmap.T

    plt.figure(figsize=(10, 10))

    plt.imshow(
        image,
        origin="lower",
        cmap="gray_r",
        vmin=0.0,
        vmax=1.0,
    )

    plt.title(f"{dataset.name} occupancy map")
    plt.xlabel("map x cell")
    plt.ylabel("map y cell")

    plt.tight_layout()

    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(
            output,
            dpi=200,
            bbox_inches="tight",
        )
        print("saved:", output)
    else:
        plt.show()


if __name__ == "__main__":
    main()