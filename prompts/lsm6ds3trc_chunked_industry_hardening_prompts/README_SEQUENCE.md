# LSM6DS3TR-C chunked hardening prompt sequence

Send these prompts one by one, in order. Do not send the next prompt until the previous one is committed and synced. Each prompt tells the coder to spawn subagents, run checks, commit, and push.

0. Start branch, baseline, AGENTS.md, hardening plan.
1. Core framework neutrality, timebase, status diagnostics.
2. Raw register safety, bank safety, partial-apply dirty state, recover/resync.
3. Reset/boot, ODR/filter settling, freshness, scaling, and self-test prep.
4. FIFO dataset model, overrun/status, bounded drain.
5. Interrupt/DRDY contracts, CLI diagnostics, Arduino adapter polish.
6. Pure ESP-IDF example, CI, no-Arduino build coverage.
7. Documentation and hardware validation matrix.
8. Final integration review and hardening final report.
