# Focused-run provenance correction, before new measurements (2026-11-03)

Independent repair review identified that the first run wrapper bound only the new root's Python files. That does
not establish unchanged actual dependencies during a baseline run using P0R_SRC, or the independent snapshot
oracle/catalog. Earlier receipts remain unchanged and are limited evidence on this point.

The revised wrapper records explicit resolved P0R_SRC and the relevant numeric/environment variables. Before and
after each new run it hashes both the selected source and local Python inputs, snapshot oracle/catalog/data,
the R1 contracts/amendments, check_static's older coverage references and the bars_h3 URDFs. It refuses to overwrite
results. New evidence uses final_* result directories; old outputs are retained. Acceptance criteria, examples,
thresholds and sample counts are unchanged. The baseline is expected to fail; successful reproduction is not a
passing repair. An unchanged host-event list is required for every measured command.
