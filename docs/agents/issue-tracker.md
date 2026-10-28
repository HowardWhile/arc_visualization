# Issue tracker: Local Markdown

Issues and specs (PRDs) live as Markdown files in `.scratch/`.

## Conventions

- One feature per directory: `.scratch/<feature-slug>/`.
- Spec: `.scratch/<feature-slug>/spec.md`.
- One file per implementation ticket:
  `.scratch/<feature-slug>/issues/<NN>-<slug>.md`.
- Number tickets from `01`; never combine all tickets into one file.
- Record triage state as a `Status:` line near the top.
  Use the role strings in `triage-labels.md`.
- Append conversation history under `## Comments`.

## Publishing and reading

When a skill says "publish to the issue tracker", create the
appropriate Markdown file under `.scratch/<feature-slug>/`.

When a skill says "fetch the relevant ticket", read the referenced
file. If only a number is provided, resolve it within the relevant
feature directory; ask when ambiguous.

## Wayfinding operations

- Map: `.scratch/<effort>/map.md`, containing Notes,
  Decisions-so-far, and Fog.
- Child ticket: `.scratch/<effort>/issues/NN-<slug>.md`.
- Record ticket type in `Type:`:
  `research`, `prototype`, `grilling`, or `task`.
- For wayfinding tickets, use `Status: open`, `Status: claimed`,
  or `Status: resolved` for execution state. Record any triage
  role separately as `Triage:` to avoid overwriting that state.
- Record dependencies as `Blocked by: NN, NN`.
  A ticket is unblocked when all listed tickets are resolved.
- Frontier: select the first numbered open, unblocked,
  unclaimed ticket.
- Claim: save `Status: claimed` before starting work.
- Resolve: append the result under `## Answer`, set
  `Status: resolved`, and add a summary and link to the map's
  Decisions-so-far.
