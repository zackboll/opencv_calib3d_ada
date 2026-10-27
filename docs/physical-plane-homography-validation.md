# Task 029 qualification and developer numerical source review

## Verified baseline

Fetched `origin/main` before implementation:

* Commit: `b119bce056dea706e9744e85ae8e258c70809aea`.
* Tree: `67f8758ba73f1fcc91a2530781bbd1b72d621fd8`.
* Clean starting worktree. Created
  `feature/029-physical-plane-homography-verification` directly from fetched main.
* Read all six repository rules and Task 026/028 contracts, result representations,
  availability accessors, construction, normalization and validation implementations.
* Actual baseline: **252/252 AUnit groups**, all **16 examples**, production/fault
  native boundary and Ada fault campaigns, **38 Python tests**, repository inventory
  and numerical profile checks passed before editing.
* Exact-main hosted qualification inspected:
  [cross-platform 39335951230](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39335951230)
  (repository-checks, linux, macos, linux-sanitizers all success), and
  [Windows 39335951208](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39335951208)
  (windows success). Both report the exact baseline commit above. No unrelated
  Geometry workflow was used as a prerequisite.

## Implementation and ownership review

Public contract and limitations are in
`/home/zboll/git/opencv/calib3d/docs/physical-plane-homography-contract.md`.
Production changes are confined to the existing Ada spec/body:

| Review concern | Source / conclusion |
|---|---|
| Entire-request validation | `Validate_Consistency_Request` extracted without changing original conditions/order/messages; shared reciprocal `Validate` calls both Task 025 validators then separation validation. Integrated call invokes both before declaring any native-backed results. |
| Independent execution | Integrated function has exactly one Task 028 call, followed by exactly one Task 026 call on held-out arrays only. No conditional skips physical verification. |
| Estimator provenance | Task 028 still contains one empirical estimator call on normalized fitting arrays only. Undefined prediction retains its original no-fit behavior. |
| Ownership/discriminants | Private outer discriminants directly constrain complete nested Task 028 result and final held-out array. No indefinite record component without a constraint, access field, native handle or extra allocator. |
| Complete diagnostics | Each evaluated record copies the complete original algebraic and reciprocal results, with original positional identity. No rejected fields are cleared. |
| Combined acceptance | Per-point conjunction of original held-out acceptance and reciprocal acceptance. Aggregate conjunction of original aggregate acceptance and every physical result. Algebraic-first summary precedence does not change either nested diagnostic. |
| Availability | Two new guarded nested diagnostic accessors reject default unavailable records. Every original nested value guard remains unchanged. |
| Numerical independence | No new geometry, norm, matrix arithmetic, threshold conversion or rounding policy. Task 026 independently derives both intersections and retains their separation; no fused position. |
| Failure containment | No exception handler in integrated function. Any native failure propagates before publishing the return value; saved owned assignment tested. |
| No scope expansion | No production C++, internal ABI, dependencies, Core/Geometry API, native backend selection or Windows workflow edits. Private ABI remains **48 matching C declarations/Ada imports**. |

This is developer source review, not independent external approval or a claim of
formal proof of OpenCV. Independent reviewer approval remains required at the open
PR review gate. No automatic approval, merge, release, tag or version bump.

## Added deterministic AUnit registrations

**14 new registrations; 266 total**. A source comparison with fetched main verified
all **252 exact original registration expressions** remain unique and in original
order. All fixtures are file-free. Existing `Consistency_Data`, `Consistency_Request`,
`Plane_Oracle_Pixel`, scalar projection helpers and original geometry fixtures are
reused; no test duplicates production ray/intersection/homography algorithms.

| New registration / helper | Evidence |
|---|---|
| `Physical_Exact` | Complete acceptance; independent original operation equality; scalar world X oracle; full owned copies; no input changes; source mutation cannot affect results. |
| `Physical_Brown` | Two distinct calibrations and independent nonzero Brown models; noncommuting poses; tilted plane; independent scalar world observation/coordinate oracle. |
| `Physical_Depth` | Algebraically correct `x2=-x1,y2=-y1` with negative second-camera physical depth, rejected reverse source and guarded unavailable separation. |
| `Physical_Wrong_Models` | Wrong plane or relative pose with plausible empirical H; both-stage rejection precedence; independently available world disagreement. |
| `Physical_Pure_Rotation` | Common-center nonidentity rotation with through-center plane rejects physically; forward reachable plane accepts; no baseline heuristic. |
| `Physical_Center_Plane` | Planes through distinct first/second centers retain exact undefined-prediction cause and complete physical diagnostics. Absent-model guard. |
| `Physical_Directional` | Forward accepted, backward source outside range; no invented separation. |
| `Physical_Separation` | Both directions accepted yet separation fails world-unit limit; scalar separation oracle; single rejection among accepted neighbors; accepted-landmark guard. |
| `Physical_Support` | Found model but insufficient final support; per-point acceptance differs from aggregate; no empirical model still preserves accepted physical evidence; absent metric guard. |
| `Physical_Bounds` | Four distinct lower bounds, fitting/held-out arrays ending at Positive'Last, all three nested identity levels one-based. |
| `Physical_Guards` | Default result never accepted, both unavailable diagnostic accessors raise. |
| `Physical_Boundaries` | Eight positive measured normalized/pixel/separation/range limits: exact equality accepts; one representable step smaller rejects; complete diagnostics retained. |
| `Physical_Invalid` | Late nonfinite held-out pixels, malformed forward/backward/intersection/separation/algebraic options, invalid pose/plane, too few fitting points, empty/mismatched held-outs. |
| `Physical_Numerical` | Exact zero projective denominator, very large finite coordinates, unrepresentable finite-endpoint differences, mixed valid/unavailable neighbors and guarded targets. |

Two additional Python source-contract tests (40 total) assert validation/execution
order, exactly one call to each original operation, held-out-only physical input,
no new solver/native access/exception swallowing, discriminant-owned representation
and example workflow registration. Original source tests now follow the shared
validators and stop at the actual original function end rather than inspecting
unrelated later operations. Their substantive invariants are retained.

## Fault campaign

The existing actual-shim fault executable adds **90 first-hit native fault cases**:
18 existing checkpoints times five exception kinds, with saved-result assignment
atomicity and healthy recovery. For each armed fault, **12 invalid requests**
(both full directional policies, separation and late held-out pixel) are rejected
first: **1,080 pre-native validation checks**. Empirical checkpoints 17/18 establish
that reciprocal validation occurs before native estimation. Projection checkpoints
1/2 are reached only in physical verification in this integration, establishing
that failure after completed algebraic verification still publishes no partial
result. Other checkpoints are shared first-hit operations; this is not coverage
of every later repeated native occurrence or both directions separately.

During development, the fault list initially selected PnP-only checkpoints 7/8
rather than projection 1/2. The campaign correctly failed as unreachable; source
inspection corrected the test checkpoint inventory, not production behavior.
Compiler feedback also corrected a test-only accessor name and an attempted
aggregate of a private type. No thresholds or original assertions were weakened.

## Qualification ledger

Local environment: Linux, OpenCV 4.10.0/calib3d, Alire 2.1.1, GNAT 16.1.0,
gprbuild 26.0.1. All Alire build/test/example mutations are serial.
Execution logs are retained under `/tmp/calib3d-029/` for this developer session.

| Executed local campaign | Result |
|---|---|
| Full final AUnit | **266/266 PASS**, zero failed assertions/unexpected errors |
| Baseline registrations retained | **252/252**, exact expressions, unique, original order |
| All synthetic examples | **17/17 PASS** |
| New physical example | **8/8 accepted**, then **0/8 accepted**, complete algebraic diagnostics unchanged |
| Production, validation and release warnings-as-errors builds | PASS |
| Actual-shim production/fault native boundary checks | PASS |
| Complete native and Ada fault campaigns | PASS |
| Actual-shim production/fault ASan/UBSan | PASS |
| Leak-enabled Ada sanitizer fault campaign including Task 029 | PASS |
| Python repository/configuration suite | **40/40 PASS** |
| Independent native numerical/profile checks | PASS |
| Repository/ABI inventory, documentation links, shell syntax, whitespace, git integrity | PASS |

GNATprove FSF 16.1.0 was run through the Alire environment with
`--mode=check --output=oneline --output-header -u opencv-calib3d.adb`.
The command completed successfully and produced
`/home/zboll/git/opencv/calib3d/obj/release/gnatprove/gnatprove.out` with its invocation
header. It explicitly reports the new operation, accessors and validators as
**skipped; SPARK_Mode => Off**, and says flow/proof were skipped because only
legality checking was requested. This existing thick-binding unit is not a SPARK
proof target. **No Task 029 formal proof, flow proof, runtime-safety proof, or
native numerical proof is claimed.** Numerical assurance here comes from reused
qualified policies, independent synthetic oracles, source review and executed
regressions; external numerical source review remains required.

The final executed local/hosted outcomes, exact tested commit/tree and PR URL are
recorded in the PR qualification ledger. Baseline success is not feature success;
pending jobs must not be represented as passed. The seventeenth example requires
8/8 accepted under the original physical policy, then 0/8 under changed backward
range while complete algebraic results remain equal.

Windows remains under the repository post-merge-only policy. The workflow is
untouched, so no feature-head Windows PR run is expected; baseline Windows success
does not qualify a new feature head. The PR must remain open, non-draft, unmerged,
with auto-merge disabled for independent review.