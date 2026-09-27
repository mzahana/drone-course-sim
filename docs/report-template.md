# Capstone report — template

**Audience:** students. **Length:** 4–6 pages, including the tables. Marked as part of the 15
manual points in [`rubric.md`](rubric.md).

Copy this file into your repository as `REPORT.md`, keep the headings, and replace everything in
italics with your own text. The italic line under each heading says what that section is for;
delete it when you submit.

**What this is marked on.** Not what you did, and not how hard it was — *whether you understood
the system you built*. Every section below asks for a number you measured and a reason it came
out that way. A report that narrates the week in order, without explaining why anything is the
way it is, gets about half the marks even if the run scored well. Two pages of honest reasoning
beats six pages of description.

Write it after your last scored run, not during. You need the numbers.

---

## 1. The system in one page

*Who talks to whom, and who is allowed to write what — so a reader can follow the rest without
opening your code.* **Half a page, plus one diagram.**

Name your five nodes, the topics each subscribes to and publishes, and the rate each runs at.
Draw the graph — ASCII is fine, and preferred over a screenshot.

Then answer these two directly, in a sentence each:

- Which node owns `/mavros/setpoint_raw/local`, and what stops anything else writing to it while
  `FOLLOW` is active?
- Which node decides the aircraft's state, and what are the exact conditions on each transition
  out of `FOLLOW`?

---

## 2. The estimator you built

*Why your filter has the numbers it has — the section that separates tuning from guessing.*
**About one page.**

Do not restate the Kalman filter equations. Assume the reader knows them. Explain your choices:

- **`R`, the measurement covariance.** Where does it come from? If you derived it from the
  step-4 model (`σ_position ≈ R_slant · σ_angle`), show the terms and the slant range you used.
  If you tuned it by hand, say so and say against what.
- **`Q`, the process noise.** What target acceleration does your `Q` implicitly assume, in m/s²?
  Check it against what the target actually does: tier 2 corners and tier 1 U-turns are the
  accelerations your filter has to survive.
- **The Mahalanobis gate.** What threshold, and what did you observe it rejecting? If you never
  saw it fire, say that — it is a finding, not a gap.
- **Coasting.** How long does your filter predict without a measurement before you give up and
  declare `LOST`, and how did you choose that number? Relate it to the 3-second blackout and to
  what the grader does with an estimate older than 1 s.

Finish with the one question that matters most here: **what did you change, and what did it do to
your RMS?** One before-and-after pair of measured numbers is worth more than a paragraph of
justification.

---

## 3. Measured error budget for your own run

*Whether you know where your error comes from, or only how large it is.* **About one page,
including the table.**

Fill this in from **your** run, not from the lecture slides. The right-hand column is what makes
the section worth marking: say how you obtained each number — measured directly, computed from a
logged quantity, or estimated and why that estimate is defensible.

| Error source | Contribution (m) | How you got it |
|---|---|---|
| Vehicle yaw / attitude | | |
| Gimbal pointing | | |
| Detection box centre | | |
| Pixel quantisation | | |
| Timestamp skew (TF looked up at the wrong time) | | |
| Other (name it) | | |
| **RSS total (predicted)** | | |
| **Measured RMS from `course score`** | | |

Then the part that carries the marks: **do the two bottom rows agree?**

- If the measured RMS is *larger* than your RSS prediction, something is in your system that is
  not in your budget. Name your best candidate and say how you would test it.
- If it is *smaller*, at least one of your terms is pessimistic. Which, and why?

State your flight geometry — altitude, standoff, look-down angle, slant range — because every
number in that table scales with it. If you flew geometry other than 12 m / 18 m / 34°, say why,
and what it cost you in detector confidence.

---

## 4. The failure, and how you found it

*Whether you can diagnose, or only fix by changing things until it stops.* **Half to one page.**

Pick **one** real failure — the worst one, not the easiest one to explain. Then, in this order:

1. **The symptom**, exactly as you first saw it. What was on screen, what number was wrong, what
   did the aircraft do.
2. **What you first believed the cause was**, and what evidence made you abandon that.
3. **How you actually localised it.** Which topic did you echo, which log did you read, which
   transform did you compare against which. The tool matters: `ros2 topic hz`, a TF lookup at two
   different timestamps, and the PX4 log are all different kinds of evidence.
4. **The cause**, in one sentence.
5. **The fix, and the measurement that proved it.** A fix without an after-number is a hope.

The silent failures are the interesting ones: a QoS mismatch that delivers no messages and no
error, a TF lookup at "now" instead of at the image stamp, two publishers on one setpoint topic,
an estimate that is fresh-looking but stale. If yours was silent, say how you would have caught
it a day earlier.

---

## 5. Your score, and your weakest category

*Whether you can read your own result critically.* **Half to one page.**

Paste the output of `course score` **verbatim**, in a code block, for each tier you ran. Do not
retype it, do not tidy it, do not round it. Include the `automated_total` line.

```
(paste here)
```

Then a summary line per tier:

| Tier | Time on station /30 | Estimation /20 | In frame /20 | Robustness /15 | Automated /85 |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |

Then, for **the lowest-scoring category across your runs**:

- What the number physically means — not "we scored 14/30" but "we were more than 8 m from the
  standoff point for 53% of the scored samples."
- **Why.** Point at a mechanism: lag on the straights, the gimbal hitting its ±135° yaw limit in
  a corner, the estimate coasting away during the blackout, the stopped-target case where `v̂_T`
  is undefined. Support it with something from `/tmp/score.json` — `coverage`,
  `worst_error_m`, `min_altitude_m`, `max_radius_m` or `mission_states_seen`.
- **What you would change, and what you predict it would score.** A prediction you can be wrong
  about is worth more here than a safe generality.

If you lost a whole 5-point robustness block, say which one and whether it was a real safety
breach or an artefact — for example, never publishing `/mission/state`, which makes your own
landing count as an altitude-floor violation.

Finally, compare against the reference solution's numbers in [`rubric.md`](rubric.md) §4. Where
you beat it, say by how much and why. Where you did not, say which of the two explanations is
true: a tuning difference, or a structural one.

---

## 6. Sim-to-real

*Whether the transfer claim of this course holds for your code specifically.* **Half a page.**

List, concretely, every change that would be needed to run your four nodes against the real
aircraft. For each one, say whether it is a change to **your** code or to something behind a
topic name you never touch.

Then answer honestly: what in your solution is tuned to this simulator and would need retuning
outdoors? The 5–6° yaw bias, the detector's confidence at this look-down angle, and the target's
top speed are the obvious candidates. Choose the one you are least confident about.

---

## 7. What you would do with another ten hours

*A short, specific, ranked list — not a wish list.* **A few lines.**

Three items, in the order you would actually do them, each with the score category it would move.
