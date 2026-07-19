# LSM6DS3TR TunnelMonitor Fit Report

## Library Mechanics

Version 2 supplies the mechanics required by an external single I2C owner:

- zero-I2C `bind()` and `unbind()`;
- staged probe, configure, sample, reset, boot, recover, reconcile,
  power-down, self-test, calibration, and FIFO-purge jobs;
- caller-provided absolute deadline and 64-bit monotonic time;
- caller-selected transport budget per `poll()`; one is the normal owner
  budget;
- a hard total callback ceiling for every accepted operation;
- bus-silent cancellation;
- 64-bit nonzero operation tokens and exactly-once terminal results;
- valid/fresh masks, a 64-bit sequence, configuration generation, read uptime,
  and full scale in each atomic raw sample;
- explicit verified, settling, and unknown configuration state;
- partial/ambiguous hardware-effect reporting;
- passive diagnostics without driver-owned offline, retry, recovery, or health
  admission policy;
- pure fixed-unit conversion that does not consult mutable driver state.

The application still owns bus locking, pins, task scheduling, queue deadlines,
transport timeout, retries, bus recovery, absence/backoff, and health policy.

## Supported First-Scope Shape

The general library supports a polling snapshot with accelerometer, gyro, and
temperature validity represented independently; a ready-checked request that
contains temperature always requires TDA. Production profiles require
FIFO and interrupts disabled. The destructive FIFO purge is maintenance only;
it is not waveform acquisition.

This shape can be called from TunnelMonitor's `I2cTask` private adapter without
adding a second task or exposing library types through public firmware
contracts. Each normal owner poll should grant one transport callback and
retain the exact application request identity together with the library token.
The 64-bit token and sample sequence provide practical non-reuse/order for one
driver lifetime but are not persistent firmware identities across restart.

## Product Integration Is Not Approved

The TunnelMonitor-node checkout inspected at
`0897f12c1a1369367747d1063936906005391580` has no authoritative LSM6DS3TR-C
product contract. It does not freeze:

- selected part/profile and required/optional role;
- SA0 strap/address;
- polling versus interrupt operation or an interrupt GPIO;
- mounting transform and rotation convention;
- ODR, ranges, filters, power modes, or accepted vector skew;
- calibration storage/validity policy;
- snapshot, tilt, event, statistics, or waveform meaning;
- sample fields, units, schema/profile version, status, or health mapping.

Current fixed product contracts also have no motion `DeviceId` or
`I2cOperation`, and the configured health and measurement-source capacities
are already full. These are TunnelMonitor product changes, not library changes.
No application contract was invented or modified during the library work.

## Integration Rules

- Only `I2cTask` may call the library or transport.
- Start calls must remain zero-I2C; multi-step work advances through owner
  polls.
- Queue expiry must cancel the library job and consume its matching terminal
  result; it must not merely stop polling.
- The adapter must opt out of `I2cTask`'s generic automatic backend retry for
  each LSM6 transport callback. One callback is one physical attempt and has
  already advanced the library operation. After taking a terminal result, the
  owner may recover the bus and explicitly start probe, reconcile, recover, or
  a new requested operation. It must not replay an ambiguous write.
- Firmware result identity must retain its existing request ID, submission
  token, device, and operation and correlate those with the library token.
- Raw register access must not enter the production adapter.
- TunnelMonitor maps precise library outcomes into its own stable error,
  health, and copied status contracts.
- A high-rate FIFO stream must not use the ordinary scalar `I2cResult` queue.

See [TUNNELMONITOR_NODE_SUITABILITY_AUDIT.md](TUNNELMONITOR_NODE_SUITABILITY_AUDIT.md)
for the complete finding disposition and remaining product/HIL gates.
