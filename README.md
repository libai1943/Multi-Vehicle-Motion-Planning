# Cooperative lane changes by incremental dynamic optimization

[Download ZIP](https://github.com/libai1943/Multi-Vehicle-Motion-Planning/archive/refs/heads/master.zip) · [Getting started](#run-the-four-lane-example) · [Guide](#follow-one-solve) · [Citation](#citation)

Twelve connected vehicles start in four lanes. Each is assigned a destination lane, and the planner must get them there without overlapping while keeping the maneuver short and steering effort low. This MATLAB/AMPL implementation accompanies the *Journal of Intelligent Transportation Systems* paper on incrementally constrained dynamic optimization.

The interesting part of this example is how it admits vehicle-to-vehicle constraints. One solve supplies the initial guess for the next. The last model expands the set of time elements on which it enforces pairwise separation, and MATLAB checks the resulting motion after each pass.

## Run the four-lane example

Use MATLAB on Windows with a compatible AMPL/IPOPT installation and a license covering this model size. The repository includes Windows solver binaries and protected MATLAB functions. Keep the files together in a **writable working copy**. The driver recreates scenario, initialization and result files in its current directory. Its supplied IPOPT options select MA57, which must be available in the solver build.

From the repository root in MATLAB:

```matlab
rng(1)          % Fix MATLAB's random stream for the scenario draw
Run_Me
```

The script draws the vehicle counts, permutes destination assignments, solves the initial models and makes up to ten incremental passes. It calls the supplied visualization helper after each pass and once more with the label `Final_Result` after the final collision check clears. If the final checker returns 1, the script raises `Solution Process Ends with a Failure`. Inspect the solver messages and final check together before using a run's output. [Experiment driver](Run_Me.m)

## Follow one solve

| Step | Source | What to look for |
| --- | --- | --- |
| Build the task | [`Run_Me.m`](Run_Me.m), `PXPY_generater.p` | `OLD1`–`OLD4` and `N1`–`N4` record the lane counts. `A1`–`A4` contain the vehicle IDs assigned to the four destination lanes. |
| Seed the collocation variables | [`case00.mod`](case00.mod), [`r00.run`](r00.run) | Construct a simple constant-speed seed with a fixed 15 s duration and export the initialization for the next model. |
| Solve the lane-assignment problem | [`case01.mod`](case01.mod), [`r01.run`](r01.run) | Enforce the lane endpoints and vehicle dynamics. The objective combines maneuver time with accumulated squared steering angle. |
| Introduce collision constraints | [`case02.mod`](case02.mod), [`r02.run`](r02.run) | At iteration `k`, require separation for every vehicle pair on time elements 1 through `k`. Each pair has four front/rear disc combinations. |
| Check and visualize | `collision_checker.p`, `lane_change_video_generation.p` | Check the current candidate and render the iteration identified by the driver. These helpers are distributed as executable MATLAB P-code. |

The ten-element mesh and twelve-vehicle count are encoded in the AMPL models. A change in `NV` in MATLAB requires matching changes in the model index sets and pair loops. To inspect the optimization itself, start with the four constraints indexed by `BAI` in [the final model](case02.mod).

## Read the output

The three AMPL run files reuse the same outputs. Position is exported in `x.txt` and `y.txt`, heading in `t.txt`, speed and acceleration in `v.txt` and `a.txt`, and steering angle and rate in `p.txt` and `w.txt`. The actual maneuver duration is in **`tf.txt`**. The nested output loops write a sequence for each car, then each time element, then the collocation nodes within that element. The AMPL assignments in `nima02.INIVAL` supply the next solve's warm start.

For a reproducible experiment, keep the generated lane assignments, `PXPY`, final outputs and random seed together. Run simultaneous experiments in separate copies since their filenames are shared.

## Citation

Please cite the following article when using this code to produce new research. [Published paper](https://doi.org/10.1080/15472450.2018.1562349)

```bibtex
@article{Li2019IncrementallyConstrained,
  author = {Li, Bai and Jia, Ning and Li, Pu and Ran, Xudong and Li, Yan},
  title = {Incrementally constrained dynamic optimization: A computational framework for lane change motion planning of connected and automated vehicles},
  journal = {Journal of Intelligent Transportation Systems},
  year = {2019},
  volume = {23},
  number = {6},
  pages = {557--568},
  doi = {10.1080/15472450.2018.1562349}
}
```

Copyright © 2018 Bai Li. Research inquiries: [libaioutstanding@163.com](mailto:libaioutstanding@163.com).
