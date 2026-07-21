# Contributing

Thank you for considering contributing to this project!

## Quick Start

1. Fork the repository.
2. Create a feature branch: `git checkout -b feature/my-feature`.
3. Make a focused change and update its public contract.
4. Run the native suite, documentation checks, and supported target builds.
5. Commit with a clear message: `git commit -m "feat: add X"`.
6. Push and open a pull request.

## Guidelines

### Code Style

- Follow existing code style (see `.clang-format`)
- Use `constexpr` instead of macros for constants
- Prefer explicit over implicit
- No heap allocations in steady-state library code

### Commits

- Use [Conventional Commits](https://www.conventionalcommits.org/) format:
  - `feat:` new feature
  - `fix:` bug fix
  - `docs:` documentation only
  - `refactor:` code change that neither fixes a bug nor adds a feature
  - `test:` adding or updating tests
  - `chore:` maintenance tasks

### Pull Requests

- Keep PRs focused (one feature/fix per PR)
- Update public Doxygen and user documentation with API or behavior changes
- Add changelog entry under `[Unreleased]`
- Run `python tools/build_docs.py`; stale pages are removed and warnings fail
- Run `python scripts/generate_version.py check` when metadata is involved
- Ensure the complete CI matrix passes

### What We Accept

- Bug fixes
- Documentation improvements
- Performance improvements (with benchmarks)
- New examples (if they demonstrate a common use case)

### What We Probably Won't Accept

- Breaking API changes without discussion
- Heavy dependencies
- Platform-specific code in the library core
- Features that add heap allocations in steady state

## Questions?

Open a GitHub Discussion or Issue for questions.
