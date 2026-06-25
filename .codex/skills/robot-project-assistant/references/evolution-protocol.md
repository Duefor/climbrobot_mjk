# Evolution Protocol

Use this protocol when durable knowledge should be added to the skill.

## What To Add

- Verified workspace architecture and package responsibilities.
- Canonical build, launch, test, calibration, and deployment commands.
- ROS graph conventions: topics, services, actions, parameters, TF frames, message types, and namespaces.
- Hardware constraints, safety checks, controller limits, and operator procedures.
- Coding conventions and design patterns observed repeatedly in the repo.
- Recurring failures and root-cause fixes.

## What Not To Add

- One-off task progress.
- Unverified guesses written as facts.
- Long logs, copied source files, or generated build output.
- Generic robotics explanations that Codex already knows.
- Secrets, credentials, private tokens, or machine-specific paths unless essential and approved.

## Update Style

- Keep entries short and actionable.
- Prefer bullets grouped by package or subsystem.
- Include dates only when chronology matters.
- Mark inferred notes with `inferred`.
- Move stale or contradicted notes into an open question or replace them with verified knowledge.

## Suggested Update Flow

1. Finish the immediate task first.
2. Identify knowledge that will help future tasks.
3. Update the smallest relevant reference file.
4. Re-run skill validation if `SKILL.md` or metadata changed.
