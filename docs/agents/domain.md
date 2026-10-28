# Domain Docs

## Before exploring

- Read root `CONTEXT.md` when present.
- Read relevant decisions under `docs/adr/`.
- If `CONTEXT-MAP.md` exists in the future, follow its links
  to the contexts relevant to the task.

If these files are absent, proceed silently. Do not suggest
creating them upfront. The domain-modeling skill creates
them when terminology or decisions are resolved.

## Layout

This repo uses a single-context layout:

- `CONTEXT.md`: domain terminology and context.
- `docs/adr/`: architectural decision records.

## Vocabulary

Use the terms defined in `CONTEXT.md` in issue titles,
proposals, hypotheses, and tests.

If a concept is missing, reconsider whether it belongs in
the domain or note the gap for domain-modeling.

## ADR conflicts

Explicitly flag proposals that contradict an existing ADR,
identify the decision, and explain why it should be revisited.
