# Hybrid Planning (TAMP) Report

**4-Line Surprise Summary:**
What surprised me most was how blind the symbolic planner truly was to the geometric world. The initial plan was syntactically perfect but physically impossible. Seeing the `monitor` node act as a truth-teller that updated the planner's beliefs was a powerful demonstration. The system's ability to correct its own world model and generate a completely new, correct plan was the key takeaway.

---

## 1. Experimental Design

The experiment was designed to force a symbolic planning failure that could only be resolved with geometric validation.

* **Symbolic State:** A PDDL problem was defined where the goal was `(obj_at obj_b loc_target)`. The initial state *falsely* claimed `(path_clear obj_b)`.
* **Geometric State:** In the MoveIt planning scene, `obj_a` was placed as a collision object, physically blocking the path to `obj_b`.
* **Sequential Mode:** The planner generated `(pick obj_b) ...`. This action was sent for execution, where it would (in a
    real system) collide and fail. The plan stops, unresolved.
* **Interleaved Mode:** The `monitor_motion` node intercepted `(pick obj_b)`, checked it with MoveIt's `/check_state_validity` service, and received `valid: false`. The monitor then updated the Knowledge Base with `(not (path_clear obj_b))` and `(blocker obj_a obj_b)`. The `replanner` node triggered a new plan. The planner, using this new knowledge, generated the correct plan: `(move_blocker obj_a obj_b ...)` -> `(pick obj_b ...)` -> `(place obj_b ...)`.

## 2. Infeasibility Event Analysis

* **Observed Replans:** 1
* **Infeasibility Event:**
    1.  **Plan 1 (Symbolic):** `(pick panda obj_b loc_b)`
    2.  **Monitor Check:** `monitor_motion.py` intercepted this action. It constructed a `RobotState` for the pick pose and sent it to `/check_state_validity`.
    3.  **MoveIt Response:** The service returned `valid: false` and listed `obj_a` as the colliding object.
    4.  **Refinement:** The monitor published "action_failed" and added `(not (path_clear obj_b))` and `(blocker obj_a obj_b)` to the PlanSys2 Knowledge Base.
    5.  **Replan:** The `replanner` node detected the failure and triggered a new planning request.
    6.  **Plan 2 (Hybrid):** `(move_blocker panda obj_a obj_b loc_a loc_away)`, `(pick panda obj_b loc_b)`, `(place panda obj_b loc_target)`

## 3. Comparison Metrics

| Planning Mode | Success | Total Time (s) | # of Replans | Final Plan |
| :--- | :--- | :--- | :--- | :--- |
| **Sequential** | ❌ **Fail** | ~5.0s (until failure) | 0 | `(pick obj_b)` |
| **Interleaved** | ✅ **Success** | ~12.5s (plan + check + replan + execute) | 1 | `(move_blocker ...), (pick ...), (place ...)` |

## 4. Analysis & Reflection

### When would symbolic replanning alone fail?

Symbolic replanning *alone* (i.e., without geometric feedback) would fail in this exact scenario. If the `monitor` node had only reported "failure" *without* updating the Knowledge Base with *why* it failed (the `blocker` predicate), the symbolic planner would have no new information. It would simply try to generate the *exact same plan* `(pick obj_b)` again, leading to an infinite failure loop. The refinement step—translating the geometric collision into a symbolic fact—is the most critical part.

### If your robot had perfect motion feasibility checks, would symbolic planning still need refinement?

**Yes, absolutely.** This is the central concept of TAMP.

Having "perfect checks" is different from having a "perfect symbolic model." The symbolic model (PDDL) is an *abstraction* of the continuous world.

1.  **Abstraction is Lossy:** PDDL model *cannot* and *should not* contain the precise coordinates of every object and all the robot's links. The symbolic model says `(path_clear obj_b)`, which is its *belief*. The "perfect check" (MoveIt) queries the *ground truth* of the geometric world and discovers this belief is wrong.
2.  **Refinement is Translation:** The TAMP loop's job is to **refine the symbolic model** based on geometric ground truth. The "refinement" *is* the process of translating the continuous-space failure (e.g., "collision with `obj_a`") into a discrete, symbolic fact (e.g., `(not (path_clear obj_b))`) that the symbolic planner can understand and reason about.

Without this refinement, the symbolic planner is "blind and deaf." It can generate plans, and the perfect check can tell it "no," but the planner will never understand *why* and will be incapable of generating a *smarter* plan next time.