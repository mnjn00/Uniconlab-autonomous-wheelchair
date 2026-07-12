---
schema_version: 1
artifact_id: A00-review-lineage
owner: WP0 governance owner
reviewer_role: independent safety architect and verification critic
status: approved_for_wp0_software_contracts
source_plan_sha256: bd1b9454bc34f68714e6b986e80466535f817a42c25f662db0990adc79ca601e
provenance: approved RALPLAN final plan
---

# WP0 review lineage

This receipt governs software-contract materialization only. It grants no hardware-motion, campus, or passenger authority.

## Immutable consensus receipts

| Stage | Disposition | SHA-256 | Effect |
|---|---|---|---|
| Planner Stage 1 | superseded | `684ccad18ccf69fd556f1410dfff50dbe8aec45807c1140f914be20582a8cfdc` | Replaced by Stage 2 after review findings. |
| Architect review 1 | BLOCK / REQUEST CHANGES | `391ae17c81e557a5857c3ff98f656326c461416676bff408fd9ec3debb41bada` | Required independent safety ownership and frozen contracts. |
| Critic review 1 | ITERATE | `a449e1bfba96011b51d4eae55759ecabd6e1e533462a823fb2bd58f33de15cf7` | Required falsifiable negative cases and evidence limits. |
| Planner Stage 2 revision | proposed immutable revision | `c8148c5f5d03a646c839e2966ee7fb5c57a433ab34a8cbfcce1d5ab69cc69068` | Resolves first-pass findings and is the approved plan body. |
| Architect review 2 | CLEAR / APPROVE | `7d91bb0e2182e38fc30b74cb096ab7ff2ea590ebe62934a5e6f8125bba2f8548` | Approved boundaries and independent ownership. |
| Critic review 2 | OKAY | `3844ab8a0594b888340459c57393c42bbd084c1282a34716ea6bdbfc59958d3a` | Approved falsifiability and negative-case coverage. |
| Intent reconciliation | reconciled-clean | `a31092d64375ae1958fa77ea8adb682ffd1232d11f01865109fcb0112fb3607f` | Confirms Noetic runtime and offline GLIM Option A. |
| Final approved plan | approved execution source | `bd1b9454bc34f68714e6b986e80466535f817a42c25f662db0990adc79ca601e` | Sole source authority for these WP0 contracts. |

## Disposition of material findings

| Finding | Contract disposition |
|---|---|
| Runtime middleware ambiguity | Ubuntu 20.04 and ROS 1 Noetic are the deployment runtime. ROS 2/GLIM is offline-only. |
| Coupled geofence authority | `wheelchair_route_safety` independently owns geofence status and permission; navigation cannot publish or remap it. |
| Thin collision/TTC semantics | Geometry, visibility, velocity, stopping envelope, immediate stop, and release hysteresis are frozen and adversarially tested. |
| Unspecified Livox conversion | Source IDL/hash, time, point layout, transaction behavior, ordering, and deterministic evidence are frozen before conversion. |
| Missing slope semantics | Independent slope supervision starts UNKNOWN/STOP; uncalibrated, stale, or ambiguous evidence cannot clear. |
| Self-reported localization confidence | Native localizer output is untrusted; an independent guard alone grants localization permission. |
| Prose-only ROS ABI | Exact v1 source inventory and canonical hashing are frozen in A02. |
| Unknown driver and target NUC facts | They remain explicit blockers; no value is inferred from simulation, replay, README text, or the current workstation. |
| Simulation overclaim risk | Simulation and replay qualify software behavior only and cannot authorize hardware or passengers. |

## Authority boundary

- `hardware_motion_authorized: false`
- `passenger_operation_authorized: false`
- `team_selected: false`
- Approved execution mode: Ultragoal with native executor slices.
- Unknown physical driver, braking, extrinsic, platform, target-NUC, route-survey, and operator facts remain blocked pending measured evidence and separate review.
