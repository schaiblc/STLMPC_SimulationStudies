# Response-letter framing notes

Working notes for the response to reviewers. Covers (a) the two items deliberately
answered in the letter rather than with new experiments, and (b) the changes made to
the released code, which the letter should disclose since reviewers can run it.

---

## 1. R2.3 — body-discretization density vs. computational overhead vs. safety margin

**Reviewer asked:** "When modeling dynamic vehicles by discretizing body outlines into
sampling points, the quantitative tradeoff between sampling density, computational
overhead and collision safety margin is not analyzed."

**Not answered with a new sweep.** Suggested framing:

- The subsampling spacing `d_sep` is a *computational tractability* parameter, not a
  safety parameter. Its role is to cap the number of obstacle terms entering the MPC
  objective; the implementation additionally caps the count at `max_obs = 50` and
  widens the spacing adaptively until that cap is met, so the compute cost per control
  step is bounded by construction rather than by the nominal `d_sep`.
- The safety consequence is bounded and has been measured: the subsampled set is a
  subset of the observed surface points, so the clearance it reports differs from the
  raw-scan clearance by a measured **0.048 m on the mean and 0.013 m on the per-run
  minimum** (Table II footnote). That is the worst-case optimism the discretization can
  introduce into the reported margins, and it is small relative to the ~0.6-1.5 m
  clearances the planner maintains.
- The compute side of the tradeoff is reported directly: mean 98 objective evaluations
  per control step, 36.0 ms mean solve time, 29% of steps reaching the 50 ms cap.
- The parameter sensitivity study (Fig. 6) sweeps the six parameters that govern
  planner *behaviour*; `d_sep` governs problem size and is bounded by `max_obs`, so it
  was not included among them.

If the reviewer presses, the sweep is cheap to add (5 values x 10 seeds on the dynamic
map, reporting clearance, minTTC and solve time together).

---

## 2. R2.4 — "state-of-the-art gap-based MPC" baseline

**Reviewer asked** for comparison against SOTA gap-based MPC and lightweight MPPI
racing planners.

**What is provided:** FGM (canonical gap-following, Sezer & Gokasan), MPPI on the
identical objective (both closed-loop at two compute budgets and per-step on matched
problem instances), plus DWA / TEB / mpc_local_planner / PD on hardware.

**No third-party gap-based MPC is implemented.** Suggested framing:

- STLMPC *is* a gap-based MPC: the tracking-line heading derives from the gap
  construction, and the MPC layer optimizes over it. A separate "gap-based MPC"
  baseline would therefore be a variant of the proposed method rather than an
  independent comparator.
- The baselines instead bracket the method's components from both sides: FGM removes
  the QP and MPC layers, PD removes the MPC layer while keeping a single QP line, MPPI
  substitutes the optimizer while holding objective and constraints fixed, and the ROS
  planners situate the method against established local planning.
- The MPPI comparator is deliberately built on the *same* objective and constraints
  rather than adopting a literature racing MPPI, so that the comparison isolates the
  optimizer rather than confounding it with a different cost formulation.

---

## 3. Code changes to disclose

The released implementation was corrected during revision. These should be mentioned,
since a reviewer running the code will otherwise see behaviour differing from the
first submission.

| Change | Why it matters |
|---|---|
| Supervisory stop is now **recoverable** (releases with 0.1 m hysteresis once forward clearance re-opens) | Previously latched permanently, requiring a restart. The manuscript's near-stop recovery description (Sec. V, answering R1.3) now matches the code. |
| FGM baseline: bounding beams clamped to the front window; window-edge case handled | Two defects made the gap-centre heading point rearward and saturated steering on 90% of control steps. **The corrected baseline performs substantially better**, and the reported FGM results are from the corrected version. |
| MPPI baseline: warm-started nominal sequence, per-step RNG reseeding, temperature scaled to sample cost spread | The untuned version was not a fair comparator (mean speed 0.75 -> 1.22 m/s after correction). |
| Constraint feasibility tolerance stated as 1e-4 | Manuscript previously stated 1e-8; 1e-4 is what runs. Verified that this choice is not responsible for the s_theta convergence limit (tested 1e-2 / 1e-4 / 1e-8, all give J_final/J_init = 1). |

**Note on the FGM result.** The corrected FGM is competitive with constant-velocity
STLMPC on static clearance. This is reported as found. The paper's position is that
gap-centring is close to clearance-maximizing in a corridor, so a static course does
not discriminate; the QP and MPC layers instead provide speed modulation (2.58 vs
1.50 m/s) and obstacle prediction (minTTC 2.50 vs 1.35 s). Reporting this honestly is
preferable to a baseline that cannot be reproduced from the released code.
