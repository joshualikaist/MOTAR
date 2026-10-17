# EQ-0 G3: perception, tracker, actor firewall and control

Overall: **LIMITED_VALIDITY**. Actual OCC-P2 source `5ff285ad053f181c5a1c86dddb2462bb44e6327b`. Current audit HEAD `7cea3c52d8c8d7ab6dbeef5fae6e929acfe279a2`; all18 scoped source/config files match actual-run commit bytes. No GPU/simulation/checkpoint loading or repository/result writes.

The independent raw-wire/algebra audit is [independent_record_algebra.json](independent_record_algebra.json). Detailed gates, source hashes and future-only proposals are [findings.json](findings.json).

## Gates

### G3-PERCEPTION-REALISM: LIMITED_VALIDITY (HIGH)

Clean flat-color scene makes target availability effectively an appearance-coded oracle. This is not evidence of a direct forbidden GT-to-actor data edge.

Evidence:
- navrl_detector.py:894-967 paints target RGB (.88,.08,.045), neutral background, returns only RGB/depth
- navrl_perception.py:615-637 fixed bootstrap RGB score; :1536-1602 threshold .55/min2pixel and training-only perturbations
- navrl_task_config.py:183-206 and :274-284 defaults camera160x90; clean false perturb switch
- all2320985 quota frames independently match camera_valid == rendered_target_pixels>=2; zero disagreements

Limits:
- No distractors, shading, appearance variation, delay, learned detector or P9 error in OCCP2
- Mask availability equivalence is not a claim of perfect range/bearing/state estimates
- Configured dropout.3/RGBnoise.015/depthnoise.02 are inactive without PERCEPTION_PERTURB=1

Impact: OCCP1/P2 remain conditional clean-simulator closed-loop measurements; cannot qualify realistic redetection, identity persistence or real sensors.

### G3-FUSION-ALGEBRA: PASS (INFO)

Recorded fused flag equals camera OR accepted LiDAR association on every quota frame; accepted fused flags always have an active track.

Evidence:
- independent_record_algebra.json groups:0 fused_union_mismatch,0 fused_active_mismatch,0 lidar_camera_overlap
- static91167+moving95947=187114 lidar-only frames; zero camera/lidar overlap expected because source gates ~camera_visible

Limits:
- PASS concerns trace algebra, not correct physical target association.

### G3-LIDAR-ASSOCIATION-IDENTITY: NOT_TESTED (HIGH)

Association uses prior predicted bearing/range plus nearest raw range, not independently identified target returns; a bar with matching range can satisfy the same gate.

Evidence:
- navrl_perception.py:1925-1973: requires active track, no camera, predicted range<12, measured<.995range, abs(range-predicted)<min(.35+2sigma,1)
- navrl_perception.py:1983-2000: default full diagonal 3D correction; only range is fresh, bearing/vertical are predicted echoes
- navrl_perception.py:1808-1816 target-like carve-out uses ±15deg and ±.55m; last_target_like feeds speed governor
- warp_lidar_kernels.py:183-249 analytic sphere target added at current pose; closer bar occludes it; semantic channel not fed to perception

Limits:
- No raw LiDAR or semantic per-association provenance retained to verify187114 associations
- Historical default lidar_range_only_update=False and silent_correct=False; accepted LiDAR resets track age and flags fused visible
- Covariance/statistical calibration NOT_TESTED; source does not justify independently observed 3D information.

Impact: LiDAR can prolong memory and alter obstacle carving/governor; fused-ever-acquired/loss differ from camera visibility. Recovery success cannot use fused flags as physical identity proof.

### G3-TRACKER-EXPIRY-CONTRACT: PASS (INFO)

Expiry is strict age>5 seconds since last accepted camera OR LiDAR correction, not5seconds of geometric occlusion or camera absence.

Evidence:
- navrl_perception.py:670-694 advances float32 age, camera correct resets, comparesstrictly>memory
- navrl_perception.py:697-725 and :1993-2000 LiDAR accepted correction resets age by default
- independentfloat32sum:50misses4.999997615814209,51misses5.099997520446777; expiry51stmiss

Limits:
- Exact GPU rounding and expiry ordering independently end-to-end replay NOT_TESTED; algebra is CPU NumPy not production execution.
- Rawtrace lacks age scalar but flags show41movingactive→inactive transitions; static0.

### G3-TRACKER-5S-RECOVERY-EXPOSURE: LIMITED_VALIDITY (HIGH)

OCCP2 supplies no observed5s primary geometric in-view interval; other loss modes do cause tracker expiry.

Evidence:
- analysis.json groupdurationminriskat5=0 for all8groups
- rawquota moving41episodes>=51consecutivefusedmisses and41deactivations; staticmax36misses/noexpiry
- moving117episodescameraabsence>=50frames, compared41fusedexpiryepisodes; these are all-cause postacquisition absence, not geometry-only
- currenttargetfeaturesmaskedbyactive but5historicaltokens sampled.5s preserve past states for about2s afterdeactivation

Limits:

Impact: Neither noexpiry nor no5sabsence should be claimed globally; controlled postacquisition geometricocclusion needed for5s stress/recovery mechanism qualification.

### G3-ACTOR-WIDTH-AND-CHECKPOINT: PASS (INFO)

Resolved actor width898/critic906 and PolicyF hash agree with frozen source and recorded logs.

Evidence:
- 4*72 +5*8*12 +5*10 +5*16=898; critic adds8
- navrl_perception.py:455-463 and :2243 widthguard; navrl_transformer_network.py:inputwidthguard/17tokens
- all112 cell logs resolved actor898/critic906 (representative source/log verified; actual log-wide count stored separately if available)
- checkpointSHA matchesf702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40

Limits:
- Checkpoint hash read only; never torch.load in EQ0
- No actorobservationdump in OCCP2; tensorcontent/normalizationstate digest NOT_RECORDED.

### G3-ACTOR-GT-FIREWALL: LIMITED_VALIDITY (HIGH)

Static source inspection finds no direct targetGT/semantic/GTvisibility input to actual perception actor path. Dynamic end-to-end counterfactual isolation not yet independently tested.

Evidence:
- navrl_task.py:6450-6481 sends targetpose only to renderer; perception receives RGB/depth/ranges/ego inputs; actor observations assigned before evaluation diagnostics
- navrl_task.py:6520-6539 privilegedGT appended to physically separate states buffer only
- runner.py:70-90 ExtractObsWrapper --play returns observations and dropsstates
- navrl_perception.py:2179-2242 concatenates sensor-derived/static/ego/track history, search/geofence/corridoroff
- OCCP2SEARCHvar absent=>build_search_supervisorNone; ifB1 used, TaskSearch:712 passes fused,trackactive,lidarmin only

Limits:
- Simulator targetGT legitimately enters sensor generation, rewards, terminals, reset treatment and critic
- Perfect simulated ego state/pose used; odometry error qualification NOT_TESTED
- No independent intervention keeping sensor frames fixed while altering all privileged/evaluator fields was run
- Evaluation geometry source usesGT but this audit alone does not prove noninterference of every memoryalias/sideeffect.

Impact: No discovered leakage invalidates P1/P2; avoid upgrading structural review to complete dynamic proof.

### G3-NORMALIZATION-ACTION-MODE: LIMITED_VALIDITY (MEDIUM)

Source/config strongly bind deterministic boundedmean and checkpoint RMS normalization, but raw result configured is not a resolved actionmode receipt.

Evidence:
- YAML64normalize_inputTrue and119-122player.deterministicTrue; OCCP2 preflight removes all inheritedNAVRL knobs; EVAL_ACTION_MODEabsent all112receipts
- runner.py:887-896 onlyexplicitNAVRL_EVAL_ACTION_MODEoverridesdefault; navrl_players.py:_model_action/get_action selectsdeterministic_actions
- squashedmodel navrl_action_models.py:183normalizesobs thenreturnsboundedtanhlatentmean asdeterministic_actions
- installedrl_games players.py:38-42model.eval; :71-79checkpoint restoremodel+RMS; running_mean_std.py:forwardupdatesonlytraining andclipsnormalizedinput±5
- normalizefrontscale targetposition/20,targetvelocity/2,covposition/400,covvelocity/4,age/5; active gates current16Dtargetfeature

Limits:
- No normalization-state before/after hash recorded in run; installeddependencyfile hashes are current audit not historical receipt
- NonNAVRL external runtimevariables inherited e.gPYTHON/AERIAL_GYM_SIM_NAME/GPU4GB/LOCAL_RANK; actualBaseSim/importorigins echoed but exhaustive runtimefingerprintabsent
- Model eval may still sample internal Gaussian before deterministic selection, consuming RNG; no stochastic action execution implied.

### G3-CONTROL-SEMANTICS: PASS (MEDIUM)

This is horizontal body-frame desiredvelocity plus yawrate with independent altitudePI, not unconstrained four-axis physical flight control.

Evidence:
- navrl_task.py:4952-5009 clipsXYperaxis±2.5,governorXY,overwritesz=clip(4*(1-z)+integral,±2.5); yaw=clipa4±1*3rad/s
- navrl_task.py:5050-5059 prev_action copiesraw4Drequest and replacesgovernedXYonly; z auxiliary policyoutput remains inhistory
- velocity_control.py:tilt45deg andverticalpriority thrust compensation; base_lee_controller.py:111-125rotatesyawframecommand toworld thenvelocityfeedback
- recordedconditionpolicy_z_output_overwritten_by_altitude_piTrue andpersisted_in_prev_actionTrue acrossall112cells
- rawquotaaltitudemin.1014/max2.7928,p99=1.1197; actualspeedp50=2.9985,p99=3.4277,max4.7853m/s

Limits:
- 2.5m/s is per commandedaxis; diagonalrequestnormmax3.5355m/s; measuredactualvelocity mayovershoot
- Policy z is indirect memory/input channel; removing it changes policy contract
- Camerayaw-only frame deliberatelyignoresbodyroll/pitch; virtualstabilization, not rigidbodyfixedcamera realism
- Yaw changes sensorFOVandworlddirection of XYcommands; pure visibilitycausality/instantstop assumptions invalid

Impact: Controlledrecovery must measure actualvelocity, altitude, yaw andvisibility; cannot treat command0 as instantaneousstationary hover.

### G3-RISKCAP-STOPPING-GUARANTEE: FAIL (HIGH)

Actual riskcap law does not enforce nonnegative stopping margin; this is an independent algebraic counterexample to a safetyguarantee, not a newly introduced implementation defect.

Evidence:
- speed_governor.py:465-475 cap=2+clip((clearance-3)/2,0,1)*(3.53553390593-2); output=min(request,cap)
- cap2m/s evenatclearance0/.1/.45/1/3; hardmargin.45/brake2.9608856678/reaction.1 affectstoppingtelemetry notriskcaplaw
- requiredsurfaceclearance at2m/s=.45+.2+4/(2*2.9608856678)=1.3254735658m
- speed_governor.py:505-531 reportsnegative stoppingmargin; riskcapdoesnotrotatecommand andstraightcorridornotturningarc
- sample4301static70 result.speed_governor contactclearance.112712m/executedcommand2m/s,negativeexecutedstoppingmargin-.875474m; thisisLEGACYAGGREGATE diagnosticpopulation,notquotaoutcomeevidence

Limits:
- No collisionfree guarantee promised by canonicaldocs; failure applies to such qualification only
- Currentriskcapimplementation fidelity PASSsource/config; no change authorized
- Target-like carve-out can remove a matchingbar; invalid LiDAR is converted to maxrange by task; sensorfailure safety NOT_TESTED
- Directionalraycorridor checksinstantcommandgeometry, not independentlyvalidateddynamicreachabletube orinterstepcollisionproof

Impact: P1/P2closedlooprates stillvalid underthisfilter; recoveryinterventions cannot assume existingriskcapmakessearch/turn/stop safe.

## Independent measurements

All112cellframes SHAchecked; exactly57,344quotaepisodes/2,320,985frames. Camera flag matches ≥2renderedpixels on everyframe; no fusedunion or fusedactive mismatch. LiDAR-only187,114frames. Moving41trackdeactivation transitions; static0. These are all-cause tracklosses, not5sgeometric in-view spells.

OCC-P2 qualifying statements remain about intervalentry rates underthissameclean frozenclosedloop. The100%resumption fraction is conditional on14,257geometry-restoredintervals, notall18,485intervals. No learnedrecovery effectiveness, physicalidentity, realisticredetection, calibrateduncertainty or safetyguarantee is established.

Sourcepasses are bounded by staticinspection and independent recorded algebra; dynamic actorGTintervention andnormalizationinvariance are NOT_TESTED. Four-output policyz is indirectinputmemory evenwhileverticalcommandcomesfromaltitudePI.

## EQ-1 proposals only

### EQ1-G3-A: CPU independent information-flow and normalization audit

Fixed cachedRGBD/ranges/ego/trackstate; vary privilegedtargetpose/velocity/GTvisibility/semanticIDs, criticstates andevaluationgeometry independently; N=1/2/128, permutedenvorder, resetboundaries. Instrumentactorinput/RMS beforeafter andselectedactions.

Acceptance: Actor898Dtensor and selecteddeterministicactions bit-identical when onlyprivileged/evaluatorfieldschange; criticextrasmaychange. RMSmean/var/count hashes unchanged inplay. RejectanyactoraliasedGTinput orupdatedRMS.

Resources: CPUonly,<=10minruntime, existingcheckpointread-only if separatelyapproved forCPUloading; noGPU/simulatorneeded forcachedfixtures. Independentreference expectations rather than comparing two productionhelpers.

### EQ1-G3-B: CPU independent tracker and association references

Hand-derivedCV/KFfixture with cameraacquisition then0/49/50/51/60misses; validLiDARonly, unrelatedbaratmatchingrange, range-onlymeasurementwithknownunobservedtangentialaxis; reset/reacquire.

Acceptance: Explicitageclock exactcomparator/dtype agreement; no resurrectionwithoutcamera wheninactive; cameraabsence separatefromfusion; flagsidentity alwayscorrect. Reportassociationfalsepositives andlegacycovarianceoverconfidence asFAIL/LIMITED ratherthantuninghistory. Unobservedaxis covariancecannotshrinksolelyfromechoingpredictedbearing inindependentrange-onlyreference.

Resources: CPUonly <=10min, no productionfunction-as-oracle; anybehaviorfixneedsnewlineage/prereg, historicalsourceunchanged.

### EQ1-G3-C: Independent appearance/identity qualification before recovery claims

Prespecifiedcachedimagefixtures: targetshade/scale/backgroundred+distractors, partial/fullocclusion andnoocclusioncontrols; geometrymatched independentannotatedpixels; comparecamera detection/3Destimate/identity errors, notjustcenterLOS.

Acceptance: Firstprove detectorconstructagainstindependentlabels; realisticaccuracythresholds require externallyjustifiedtaskrequirements/calibration, currentlyNOT_DEFINED. NoPASSfromcleanredmaskidentityorallrestoredconditionalfraction.

Resources: CPU fixtureevaluation<=15min plus annotation; existingrealimagepolicyconnection remainsNOT_TESTED. NewGPUrendererfixture onlyif separatelypreregistered/approved.

### EQ1-G3-D: Future controlled GPU observability/control canary; PROPOSAL ONLY

At most16fixedscenes x8env; commoninitialacquiredprefix, independentlyverifiedoccluder onset; durations.5/1/2/4.9/5.0/5.1/6s plusFOVexit andcapture/crashcompetitorcontrols; cameradefined vsfuseddefined endpoints separately. Fixedcontroller/meanactor/checkpoint; noPPO.

Acceptance: Independentgeometry/sensoralignment andsidecar episodejoins PASS; eachconditionactuallyrealizesprespecifiedeligibleexposureorNOT_REALIZED (noresampling). RecordtrueGTseparatelyfromactor, age/cov/history, actualvel/z/yaw, pixelcount, associations, command/governor/collision. No recoveryeffectdecisionfromthisqualification; prereg independentphysicalerror/clearance tolerances beforelaunch.

Resources: RTX3070 futurecap0.5GPUh,32envorless; expected<=0.25h isplanningonlyNOT_MEASURED; firstintegrity/nonfinite/unsafeeventstops preserveallraw. Requiresseparateexplicitapproval; noEQ1executiondone.

## Git Traceability

{
  "branch": "codex/mt-p1",
  "base_actual_run_sha": "5ff285ad053f181c5a1c86dddb2462bb44e6327b",
  "new_commit": "NONE auditartifacts/tmp only parentwillrecordapprovedreport",
  "tracked_worktree_status": "",
  "remote_push": false,
  "main_merge": false,
  "raw_checkpoint_integrity": "RawframeSHAchecked112cells;checkpointSHAverified; nevermodified"
}

## Bounded G0 noninterference evidence and reproducibility addendum

Existing G0 noninterference is **PASS**, not wholly untested: eight telemetry OFF/ON pairs (seed4201, Static/Moving, allfour densities), with bitwise sampledactorobservation equality, fulltrajectorydigest equality, ledger/recorder equality and results equal exceptnonce. 11520 actorobservationrows werecompared. The frozen G0 reportSHA matches the OCCP2 binding; see [bounded_g0_evidence.json](bounded_g0_evidence.json). This bounded inheritance does not establish arbitrary privilegedGT counterfactual isolation or empirical sensor realism.

The NEW faithfully reconstructed standalone [reproducer](reproduce_independent_audit.py) was executed successfully with CUDA hidden and no project imports. All previously computed group/frame/algebra summaries match exactly; all41 recorded deactivations occur at51 consecutive fused misses. Logs resolve898/906 inall112cells. See [reproduced_checks.json](reproduced_checks.json). It is explicitly not presented as the original in-memory command text.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B /tmp/eq0_g3_20261017/reproduce_independent_audit.py
```
