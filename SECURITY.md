# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 2.0.x   | :white_check_mark: |
| 1.x     | :x:                |

## Reporting a Vulnerability

If you discover a security vulnerability within this library, please follow responsible disclosure:

1. **Do NOT** open a public GitHub issue.
2. Email the maintainer at: `info@thymos.cz`.
3. Include:
   - A description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Any suggested fixes (optional)

We will acknowledge receipt within 48 hours and aim to provide a fix or mitigation within 14 days for critical issues.

## Scope

This library is designed for embedded systems. Security considerations include:

- No dynamic memory allocation in steady state (reduces attack surface)
- No network code (networking is out of scope for this library)
- No persistent storage in the library core

## Security Best Practices for Users

- Validate external inputs before constructing `DriverConfig`, `DeviceProfile`,
  or operation request values.
- Serialize all non-const driver calls and keep transport access in one owner.
- Bound every transport callback with the configured timeout; do not hide
  retries or ambiguous writes inside callbacks.
- Use hardware watchdogs in production deployments.
- Keep dependencies updated.
