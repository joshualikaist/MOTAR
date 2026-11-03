# Review amendment: preserve explicit public signatures (2026-11-03)

Before final validation, independent review identified a possible new boundary surface in the initial r1 generic
numeric decorator: redispatching `function(*args, **kwargs)` can invoke a str-subclass keyword's equality method
inside the protected call. Add a stack-instrumented regression with valid packet/certificate/map controls and
an explicit external equality positive control. Initial Python binding in the caller is distinguished from an
additional binding inside the core. Reproduce against the initial repaired source in a new evidence directory,
then use explicit-signature public methods with numeric/warning contexts inside their bodies. This adds a
regression for the existing no-caller-code boundary criterion; old records and gate thresholds remain unchanged.
