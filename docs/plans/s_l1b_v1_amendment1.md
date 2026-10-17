# S-L1B v1 amendment 1: elevation eligibility for the common-frame carve

Recorded during CPU runtime development, before any S-L1B GPU process or measurement. The original
[plan](s_l1b_v1.md) and [JSON](s_l1b_v1.json) bytes are unchanged; the
[amendment JSON](s_l1b_v1_amendment1.json) governs this one override.

**Finding.** The CPU fixture B_TIGHT puts a tall bar 2.77 m in front of an on-axis target 3 m away.
It exposed a flaw in the preregistered azimuth-only angular limit:

- The ±10° and +20° LiDAR beams cannot meet the target.
- Those beams still returned the bar at 2.813 m and 2.948 m.
- Both returns fell inside [s_L, s_L + 0.55) with s_L ≈ 2.80 m, so they were carved. That is a
  false clear of a real bar.

S-L1A's frozen carve uses the same azimuth-only test. Its GPU fixtures (S14, A_FOREGROUND) did not
exercise it, because their bars stand nearer than the window at every elevation. This is now a
recorded S-L1A limitation; S-L1A's bytes and results are unchanged.

**Change.** A LiDAR row may be carved only if its elevation lies within the minimum and maximum
body-frame elevations of the valid candidate pixels. Those pixels are the camera's own current
target support, the same pixels that define anchor_L. The following are unchanged:

- the azimuth limit;
- the window [s_L, s_L + 0.55) with zero tolerance below s_L;
- the support and refusal rules;
- the camera blank mask.

The change only removes carve eligibility, so carving becomes stricter. It selects nothing from
measured performance, because no measurement exists. Every gate, threshold, fixture, run limit and
verdict rule is unchanged. The independent oracle implements the same rule separately.
