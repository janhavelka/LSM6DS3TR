#!/usr/bin/env python3
"""Host-only parser and result-contract tests for the targeted HIL runner."""

from __future__ import annotations

import unittest
from unittest import mock

import run_hil


class HilParserTests(unittest.TestCase):
    def test_hil_target_options_preserve_s3_defaults_and_accept_s2_fixture(self) -> None:
        with mock.patch("sys.argv", ["run_hil.py", "--port", "COM30"]):
            defaults = run_hil.parse_args()
        self.assertEqual(defaults.chip, "esp32s3")
        self.assertFalse(defaults.assert_dtr)
        self.assertFalse(defaults.skip_accel_calibration)
        self.assertEqual(defaults.expected_flash_bytes, 4 * 1024 * 1024)
        self.assertEqual(defaults.expected_psram_bytes, 2 * 1024 * 1024)

        with mock.patch(
            "sys.argv",
            [
                "run_hil.py", "--port", "COM8", "--chip", "esp32s2",
                "--assert-dtr", "--expected-psram-bytes", "0",
                "--skip-accel-calibration",
            ],
        ):
            s2 = run_hil.parse_args()
        self.assertEqual(s2.chip, "esp32s2")
        self.assertTrue(s2.assert_dtr)
        self.assertTrue(s2.skip_accel_calibration)
        self.assertEqual(s2.expected_psram_bytes, 0)

    def test_bus_and_full_diagnostics_parsers(self) -> None:
        transcript = (
            "bus ready=yes selected_address=0x6A bound_address=0x6A "
            "frequency_hz=400000 timeout_ms=50\n"
            "bus_init code=0 detail=0 message=OK\n"
            "last_error present=yes code=27 detail=2 message=address NACK "
            "time_ms=1234 age_ms=7\n"
            "mismatch present=yes register=0x10 expected=0x40 observed=0x44\n"
        )
        self.assertEqual(
            run_hil.parse_bus(transcript),
            (True, 0x6A, "0x6A", 400000, 50),
        )
        bus_init = run_hil.BUS_INIT_RE.search(transcript)
        self.assertIsNotNone(bus_init)
        assert bus_init is not None
        self.assertEqual((bus_init.group(1), bus_init.group(3)), ("0", "OK"))
        error = run_hil.LAST_ERROR_RE.search(transcript)
        self.assertIsNotNone(error)
        assert error is not None
        self.assertEqual((error.group(1), error.group(3), error.group(4)),
                         ("yes", "2", "address NACK"))
        mismatch = run_hil.MISMATCH_RE.search(transcript)
        self.assertIsNotNone(mismatch)
        assert mismatch is not None
        self.assertEqual((mismatch.group(1), mismatch.group(2), mismatch.group(3),
                          mismatch.group(4)), ("yes", "10", "40", "44"))

    def test_transport_error_evidence_accepts_zero_bytes_received(self) -> None:
        transcript = (
            "last_error present=yes code=25 detail=0 "
            "message=I2C read length mismatch time_ms=17183 age_ms=499\n"
        )
        run_hil.require_last_transport_error(transcript)

        with self.assertRaises(run_hil.HilFailure):
            run_hil.require_last_transport_error(
                "last_error present=yes code=0 detail=0 message=OK "
                "time_ms=17183 age_ms=499\n"
            )

    def test_profile_parser_accumulates_all_staged_lines(self) -> None:
        transcript = (
            "profile staged valid=yes code=0 detail=0 message=OK\n"
            "profile staged xl_odr=104 xl_fs_g=2 xl_power=hp xl_lpf2=off "
            "xl_slope_hp=off xl_6d_lpf=off\n"
            "profile staged g_odr=208 g_fs_dps=500 g_power=hp g_lpf1=on "
            "g_hpf=off g_hpf_mode=0.260 g_sleep=off\n"
            "profile staged bdu=on offset_weight_mg=16 offset_x=-1 offset_y=2 "
            "offset_z=3 fifo=off interrupts=off\n"
        )
        parsed = run_hil.parse_profile(transcript)
        self.assertEqual(parsed["xl_odr"], "104")
        self.assertEqual(parsed["g_hpf_mode"], "0.260")
        self.assertEqual(parsed["offset_x"], "-1")
        self.assertEqual(parsed["interrupts"], "off")

    def test_profile_parser_rejects_incomplete_transcript(self) -> None:
        with self.assertRaises(run_hil.HilFailure):
            run_hil.parse_profile(
                "profile staged xl_odr=104\n"
                "profile staged g_odr=104\n"
                "profile staged offset_x=0\n"
            )

    def test_latest_status_code_wins(self) -> None:
        transcript = (
            "status=5 detail=0 message=IN_PROGRESS\n"
            "status=0 detail=0 message=OK\n"
        )
        self.assertEqual(run_hil.parse_status_code(transcript), 0)

    def test_successful_stress_summary_is_exact(self) -> None:
        transcript = (
            "stress summary requested=3 completed=3 ok=3 fail=0 cancelled=no "
            "aborted=no elapsed_ms=30 transport_ok_delta=6 transport_fail_delta=0 "
            "probes=0 reconciles=0 samples=3\n"
        )
        parsed = run_hil.require_stress_success(transcript, "stress", 3)
        self.assertEqual(parsed["samples"], 3)

    def test_latest_stress_summary_is_selected(self) -> None:
        transcript = (
            "stress summary requested=1 completed=1 ok=0 fail=1 cancelled=no "
            "aborted=no elapsed_ms=1 transport_ok_delta=0 transport_fail_delta=1 "
            "probes=0 reconciles=0 samples=1\n"
            "stress summary requested=2 completed=2 ok=2 fail=0 cancelled=no "
            "aborted=no elapsed_ms=2 transport_ok_delta=4 transport_fail_delta=0 "
            "probes=0 reconciles=0 samples=2\n"
        )
        parsed = run_hil.parse_stress_summary(transcript, "stress")
        self.assertEqual(parsed["requested"], 2)
        self.assertEqual(parsed["ok"], 2)

    def test_failed_or_incomplete_stress_is_rejected(self) -> None:
        failed = (
            "stress summary requested=3 completed=3 ok=2 fail=1 cancelled=no "
            "aborted=no elapsed_ms=3 transport_ok_delta=4 transport_fail_delta=1 "
            "probes=0 reconciles=0 samples=3\n"
        )
        incomplete = failed.replace("completed=3", "completed=2").replace(
            "ok=2 fail=1", "ok=2 fail=0"
        )
        with self.assertRaises(run_hil.HilFailure):
            run_hil.require_stress_success(failed, "stress", 3)
        with self.assertRaises(run_hil.HilFailure):
            run_hil.require_stress_success(incomplete, "stress", 3)

    def test_zero_transaction_stress_is_rejected(self) -> None:
        transcript = (
            "stress summary requested=3 completed=3 ok=3 fail=0 cancelled=no "
            "aborted=no elapsed_ms=3 transport_ok_delta=0 transport_fail_delta=0 "
            "probes=0 reconciles=0 samples=3\n"
        )
        with self.assertRaises(run_hil.HilFailure):
            run_hil.require_stress_success(transcript, "stress", 3)

    def test_job_parser_requires_complete_result_and_sample_evidence(self) -> None:
        transcript = (
            "accepted token=7\n"
            "result token=7 kind=sample state=succeeded transactions=2/66 "
            "changed=no started_ms=100 completed_ms=102\n"
            "status=0 detail=0 message=OK\n"
            "sample sequence=9 generation=3 valid=0x07 fresh=0x07 read_ms=101 "
            "quality=ready_checked xl_fs_g=2 g_fs_dps=250\n"
            "  raw accel=1,2,3 gyro=4,5,6 temp=7\n"
            "  accel_ug x=61 y=122 z=183\n"
            "  gyro_udps x=35000 y=70000 z=105000\n"
            "  temperature_mC=25027\n"
        )
        cli = object.__new__(run_hil.SerialCli)
        sent: list[str] = []
        cli.send = sent.append  # type: ignore[method-assign]

        def read_until(done, timeout_s):  # type: ignore[no-untyped-def]
            self.assertEqual(timeout_s, 1.0)
            self.assertTrue(done(transcript))
            return transcript

        cli.read_until = read_until  # type: ignore[method-assign]
        result = cli.job("sample all ready", timeout_s=1.0)
        self.assertEqual(sent, ["sample all ready"])
        self.assertEqual((result.token, result.started_ms, result.completed_ms),
                         (7, 100, 102))
        run_hil.require_success(result, "sample")
        parsed = run_hil.parse_sample(result, 0x07, True)
        self.assertEqual(parsed["quality"], "ready_checked")

    def test_require_success_rejects_invalid_transaction_time_and_effect_evidence(self) -> None:
        def result(**changes):  # type: ignore[no-untyped-def]
            values = {
                "command": "sample all ready",
                "token": 1,
                "kind": "sample",
                "state": "succeeded",
                "transactions": 2,
                "transaction_limit": 66,
                "changed": False,
                "status_code": 0,
                "status_message": "OK",
                "started_ms": 100,
                "completed_ms": 101,
                "output": "",
            }
            values.update(changes)
            return run_hil.JobResult(**values)

        run_hil.require_success(result(), "sample")
        for invalid in (
            result(transactions=0),
            result(transactions=67),
            result(completed_ms=99),
            result(changed=True),
            result(command="configure", kind="configure", changed=False),
        ):
            with self.assertRaises(run_hil.HilFailure):
                run_hil.require_success(invalid, invalid.kind)

    def test_job_parser_rejects_token_mismatch(self) -> None:
        transcript = (
            "accepted token=7\n"
            "result token=8 kind=probe state=succeeded transactions=2/2 "
            "changed=no started_ms=1 completed_ms=2\n"
            "status=0 detail=0 message=OK\n"
            "  address=0x6A who_am_i=0x6A\n"
        )
        cli = object.__new__(run_hil.SerialCli)
        cli.send = lambda command: None  # type: ignore[method-assign]
        cli.read_until = lambda done, timeout_s: transcript  # type: ignore[method-assign]
        with self.assertRaises(run_hil.HilFailure):
            cli.job("probe", timeout_s=1.0)

    def test_job_parser_correlates_latest_accept_result_and_status(self) -> None:
        transcript = (
            "accepted token=6\n"
            "result token=6 kind=probe state=failed transactions=1/2 "
            "changed=no started_ms=1 completed_ms=2\n"
            "status=27 detail=2 message=stale NACK\n"
            "  address=0x6A who_am_i=0x00\n"
            "accepted token=7\n"
            "result token=7 kind=probe state=succeeded transactions=1/2 "
            "changed=no started_ms=3 completed_ms=4\n"
            "status=0 detail=0 message=OK\n"
            "  address=0x6A who_am_i=0x6A\n"
        )
        cli = object.__new__(run_hil.SerialCli)
        cli.send = lambda command: None  # type: ignore[method-assign]

        def read_until(done, timeout_s):  # type: ignore[no-untyped-def]
            self.assertTrue(done(transcript))
            return transcript

        cli.read_until = read_until  # type: ignore[method-assign]
        result = cli.job("probe", timeout_s=1.0)
        self.assertEqual((result.token, result.state, result.status_code),
                         (7, "succeeded", 0))
        self.assertEqual((result.started_ms, result.completed_ms), (3, 4))

    def test_sample_parser_rejects_wrong_scale_or_missing_conversion(self) -> None:
        base = run_hil.JobResult(
            command="sample accel ready", token=1, kind="sample",
            state="succeeded", transactions=2, transaction_limit=66,
            changed=False, status_code=0, status_message="OK", started_ms=10,
            completed_ms=12, output=(
                "sample sequence=1 generation=1 valid=0x01 fresh=0x01 read_ms=11 "
                "quality=ready_checked xl_fs_g=4 g_fs_dps=250\n"
                "  raw accel=1,2,3 gyro=0,0,0 temp=0\n"
            ),
        )
        with self.assertRaises(run_hil.HilFailure):
            run_hil.parse_sample(base, 0x01, True)

    def test_scan_summary_contract(self) -> None:
        transcript = (
            "scan address=0x6A ack=yes code=0 detail=0 message=OK\n"
            "scan address=0x6B ack=no code=27 detail=2 message=NACK\n"
            "scan summary attempted=2 found=1 failures=0 cancelled=no elapsed_ms=2\n"
        )
        match = run_hil.SCAN_SUMMARY_RE.search(transcript)
        self.assertIsNotNone(match)
        assert match is not None
        self.assertEqual(tuple(int(match.group(i)) for i in (1, 2, 3)), (2, 1, 0))


if __name__ == "__main__":
    unittest.main()
