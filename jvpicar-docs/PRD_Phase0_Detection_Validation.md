# PRD — Phase 0: Cat Detection Validation (Mac)

Project: PiCar-X "Chase the Cat"
Phase: 0 of 3
Status: Not started
Owner: (you)
Last updated: 2026-06-02

---

## 1. Purpose

Validate the single riskiest, most uncertain component of the entire project —
**whether an off-the-shelf object detector can reliably recognize *your specific cat*,
in *your home*, under *real conditions*** — in complete isolation from the robot.

This phase runs entirely on the Mac. No PiCar-X, no vilib, no motors, no network
control loop. A webcam (or the Mac's camera, walked around on a laptop) points at the
cat; a detector runs; results and metrics are recorded. The robot is deliberately not
involved so that a detection failure is unambiguous — it cannot be blamed on Pi frame
rate, camera mounting, or control-loop timing.

### Why this phase exists

- **De-risking.** If detection of the cat is not viable, that must be discovered in an
  afternoon on a fast machine, not after days of robot integration.
- **Your cat is a hard case.** She is a gray/blue long-haired tabby. Against gray
  couches, shadows, and hardwood she has low color contrast, and when curled up asleep
  in dim light she loses her cat silhouette. The easy frames (facing camera, well lit)
  will pass trivially; this phase exists to find the *failure boundary*.
- **It prototypes the Phase 2 detector.** The script's output contract is identical to
  the detector interface the whole system depends on (Section 6). Phase 0 is throwaway
  in its UI but permanent in its contract.

---

## 2. Scope

### In scope
- Run **multiple** candidate detection models against live and recorded video of the cat.
- Quantitatively compare the models on detection rate, false positives, confidence
  distribution, and Mac frame rate.
- Filter detections to the `cat` class and explicitly handle the `person` class.
- Log everything: structured per-frame records + saved annotated frames on key events.
- Produce a written recommendation: which model to carry to Phase 2, at what confidence
  threshold, and whether tagless cat detection is viable at all.

### Out of scope
- Any PiCar-X hardware, vilib, motors, servos, or networking.
- Any model *training* or fine-tuning. Phase 0 uses pretrained models only. (Fine-tuning
  on the cat is a documented fallback, Section 9, not a Phase 0 deliverable.)
- Real-time performance optimization for the Pi (that is Phase 2's problem).

---

## 3. Success criteria (exit conditions)

Phase 0 is **complete** when all of the following hold:

1. At least **three** candidate models have been run against the same evaluation set and
   scored on every metric in Section 7.
2. A results table exists comparing the models quantitatively.
3. A **decision is recorded**: chosen model + chosen confidence threshold, with the
   numbers that justify both.
4. The **tagless-viability question is answered**: can the chosen model find the cat
   reliably enough (target: see below) without a tag, or does Phase 1's tag stepping
   stone need to persist into Phase 2?

Phase 0 is a **GO** for Phase 2 if the chosen model achieves, on the evaluation set:
- **Detection rate ≥ 80%** on "fair" frames (cat reasonably visible, any pose, normal
  indoor light), AND
- **Detection rate ≥ 50%** on "hard" frames (dim light, curled up, partially occluded), AND
- **False-positive rate low enough** that a confidence threshold cleanly separates true
  cat detections from furniture/shadow false hits.

These thresholds are the recommended default. They are explicitly a **tuning decision** —
record the actual achieved numbers and adjust the bar consciously rather than silently.

Phase 0 is a **conditional GO** (proceed, but Phase 1's tag persists) if detection works
but only above the "fair" bar, not the "hard" bar.

Phase 0 is a **NO-GO / pivot** if no model clears the "fair" bar even on the Mac. The
pivot options are in Section 9.

---

## 4. The evaluation set

The quality of this phase depends entirely on testing against *realistic* frames, not
cherry-picked ones. Build the evaluation set deliberately to span the conditions the
robot will actually face.

### 4.1 Capture protocol
Walk the Mac (or a webcam on a long cable) around the home pointing at the cat. Capture
short clips, not single stills, so motion and angle variation are represented. Aim to
cover this matrix:

| Dimension | Values to capture |
|---|---|
| Lighting | bright daylight, normal indoor, dim/evening, backlit |
| Distance | close (<1 m), mid (1–3 m), far (>3 m) |
| Pose | standing/walking, sitting upright, curled/loaf, lying flat |
| Occlusion | fully visible, partially behind furniture, only head/only body |
| Background | against contrasting surface, against gray/similar surface |
| Motion | still, slow walk, fast move |
| Distractors | cat alone, cat + person in frame, person alone (no cat) |

### 4.2 Frame difficulty labels
Each clip/frame is labeled by difficulty so success criteria can be measured per-tier:
- **Easy** — cat large, well lit, clearly cat-shaped, facing camera.
- **Fair** — cat reasonably visible, any pose, normal light, maybe slightly turned.
- **Hard** — dim, curled, partially occluded, far, or low-contrast against background.

### 4.3 Ground truth
For metric computation, each evaluation frame needs a ground-truth label: *is there a cat
in this frame, yes/no*, and ideally a rough bounding box. For a project of this scale,
manual labeling of a few hundred sampled frames is sufficient — full pixel-perfect
annotation is not required. The "person alone, no cat" clips are critical: they measure
false positives directly.

---

## 5. Candidate models

The PRD does not pre-pick a winner. It specifies a **bake-off** across candidates that
all expose `cat` as a native class (all are COCO-trained or equivalent). Run at least
three. Suggested slate:

| Candidate | Family | Why it's in the running | Notes |
|---|---|---|---|
| YOLOv8n / YOLOv11n (nano) | Ultralytics YOLO | Easiest to stand up, visualizes out of the box, fast | Likely baseline |
| YOLOv8s / YOLOv11s (small) | Ultralytics YOLO | Higher accuracy if nano misses the hard frames | Slower, heavier |
| MobileNet-SSD (TFLite) | TF / TFLite | The natural Pi-friendly candidate; quantizable for Phase 2 | Lower accuracy ceiling |
| (optional) EfficientDet-Lite | TFLite | Mid-point accuracy/speed | If time permits |

The point of including a TFLite candidate even on the Mac is that **Phase 2 runs on the
Pi**, where TFLite + quantization is the realistic deployment path. Knowing how the
Pi-friendly model compares *now*, on accuracy, avoids a nasty surprise at migration.

For each model, record exact version/weights and input resolution — these are part of the
result and must be reproducible.

---

## 6. The detector interface contract (carries to all phases)

This is the single most important artifact Phase 0 produces, beyond the metrics. Every
candidate model, regardless of family, must be wrapped to emit the **same** output tuple
per frame:

```
Detection result (per frame):
  found       : bool        # was a cat detected above threshold this frame
  x           : float       # cat bbox center, normalized 0.0–1.0 (left→right)
  y           : float       # cat bbox center, normalized 0.0–1.0 (top→bottom)
  confidence  : float       # detector confidence for the chosen detection, 0.0–1.0
  bbox        : (x1,y1,x2,y2) | None   # normalized, for logging/overlay
```

Design rules:
- **Coordinates are normalized 0–1, not pixels.** This decouples the contract from camera
  resolution (Mac webcam ≠ Pi camera). Downstream code (Phase 1/2 control loop) consumes
  normalized coords and never needs to know the frame size.
- **`cat` class only.** If the model emits multiple classes, filter to `cat`. If multiple
  cats are detected (unlikely but possible), pick the highest-confidence one for `x/y` and
  note multi-detection in the log.
- **`person` is explicitly suppressed for chase logic** but **logged**. The model will see
  any human in the room with high confidence; the contract must ignore `person` for
  targeting so the robot never chases a human, while still recording that a person was
  present (useful for debugging "why did it behave oddly when I walked in").

Because Phase 1 (color-blob) and Phase 2 (real model) both implement this exact tuple, the
control loop is written **once** against the contract and never changes when the detector
is swapped.

---

## 7. Metrics (the quantitative comparison)

For each candidate model, against the evaluation set, compute and record:

| Metric | Definition | Why it matters |
|---|---|---|
| Detection rate (Easy) | true positives / total easy frames | Sanity floor; should be near 100% |
| Detection rate (Fair) | TP / total fair frames | The headline number; gates GO |
| Detection rate (Hard) | TP / total hard frames | Gates tagless viability |
| False-positive rate | false cat detections / non-cat frames | The "is it chasing a pillow" number |
| Confidence separation | distribution of confidence for TP vs FP | Where the threshold gets set |
| Mac FPS | frames/sec at chosen input resolution | The *ceiling*; Pi will be slower |
| Latency per frame | ms per inference | Informs Pi feasibility |

### On confidence separation
This is the metric that sets the operating threshold. Plot (or tabulate) the confidence of
true detections vs false detections. The threshold goes in the gap between them. A model
with a clean gap (true hits at 0.7+, false hits below 0.4) is far more useful than a model
with higher raw accuracy but overlapping distributions, because the clean-gap model gives a
threshold that suppresses false chases. **Record this explicitly per model.**

### On Mac FPS
The Mac frame rate does **not** predict Pi frame rate (the Mac is far faster). It is
recorded as a *ceiling* and a *relative* comparison between models. If a model is marginal
on accuracy even with the Mac's full compute, the Pi will not save it — that is itself a
finding that pushes toward the offload option in a later phase.

---

## 8. Logging requirements

Logging is a first-class requirement from Phase 0 onward (committed early so the discipline
is built in, not retrofitted). Two streams:

### 8.1 Structured per-frame log
Machine-parseable (JSON-lines or CSV), one record per processed frame:
```
timestamp, model_id, frame_id, found, x, y, confidence,
  bbox, person_present, ground_truth_label, difficulty_tier
```
This is the tuning instrument. It lets the session be replayed and queried after the fact
("what confidence was it acting on when it false-fired", "what was the hard-frame hit rate
for model X").

### 8.2 Event-triggered frame capture
Save the actual frame, **with the detection box drawn on it**, on key events:
- detection gained (no-cat → cat)
- detection lost (cat → no-cat)
- false positive (detection where ground truth says no cat)
- high-confidence person detection

Frame capture is **event-triggered, not continuous** (continuous saving is disk/CPU heavy
and unnecessary). Behind a flag so it can be switched off once tuning is done.

The saved frames are what turn "confidence 0.71" into "oh, it locked onto the throw pillow."

---

## 9. Risks and fallbacks

| Risk | Likelihood | Fallback |
|---|---|---|
| No pretrained model clears the "fair" bar on the cat | Low–Med | Fine-tune a model on a few hundred labeled frames of the actual cat (collect during Phase 0 capture). Documented pivot, not default. |
| Models clear "fair" but fail "hard" frames | Med | Conditional GO: keep Phase 1's color tag in play through Phase 2; chase only commits on fair-quality detections. |
| Pi-friendly (TFLite) model far worse than YOLO | Med | Phase 2 decision point: accept lower accuracy, add a Coral USB accelerator, or offload detection to the Mac. Flag now, decide in Phase 2. |
| Person consistently detected over cat | High (expected) | Already handled by contract: `person` suppressed for targeting. Verify suppression works in this phase. |
| Evaluation set too easy / unrepresentative | Med | Deliberately over-weight hard frames in capture; the difficulty labels make this measurable. |

---

## 10. Deliverables

1. **Capture set** — labeled clips/frames spanning the Section 4 matrix, with difficulty
   tiers and ground-truth labels.
2. **Detector wrappers** — each candidate model wrapped to emit the Section 6 contract.
3. **Eval harness** — runs a model over the eval set, computes Section 7 metrics, writes
   the Section 8 logs.
4. **Results table** — the quantitative model comparison.
5. **Decision record** — chosen model, chosen threshold, tagless-viability answer, with
   supporting numbers. This is the hand-off artifact into Phase 2.

---

## 11. Open questions / assumptions

- **ASSUMPTION:** Pretrained COCO `cat` class is the starting point; no fine-tuning unless
  the fallback triggers. Confirm acceptable.
- **OPEN:** Exact model versions/weights to slate beyond the suggested three — confirm the
  candidate list before building wrappers.
- **OPEN:** How many evaluation frames constitute "enough" for confidence in the metrics?
  Recommend a few hundred sampled across tiers; confirm appetite for labeling effort.
