#!/usr/bin/env python3
"""Mutation regressions for the native IDF mapper-boundary contract."""

from __future__ import annotations

import contextlib
import io
import re
import unittest
from unittest import mock

import check_idf_example_contract as contract


class MapperContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.main_path = contract.IDF_MAIN_DIR / "main.cpp"
        self.main_text = self.main_path.read_text(encoding="utf-8")
        # The fixture's outer braces are at column zero, unlike nested blocks.
        # Extract independently of the checker's definition/body parser.
        self.mappers: dict[str, str] = {}
        for name in ("mapEspError", "mapEspProbeError"):
            match = re.search(
                rf"^Status {name}\([^\n]*\) \{{.*?^\}}",
                self.main_text, re.MULTILINE | re.DOTALL,
            )
            self.assertIsNotNone(match, f"missing fixture definition: {name}")
            self.mappers[name] = match.group(0)

    def check_mappers(self, definitions: str, expected_error: str = "") -> None:
        mutated = self.main_text
        for definition in self.mappers.values():
            mutated = mutated.replace(definition, "", 1)
        mutated = mutated.replace(
            "Status i2cWrite(", definitions + "\n\nStatus i2cWrite(", 1
        )
        original_read = contract.read

        def read(path, label):
            return mutated if path == self.main_path else original_read(path, label)

        output = io.StringIO()
        with (
            mock.patch.object(contract, "read", side_effect=read),
            contextlib.redirect_stdout(output),
        ):
            if expected_error:
                with self.assertRaises(SystemExit) as failure:
                    contract.main()
                self.assertEqual(1, failure.exception.code)
                self.assertIn(expected_error, output.getvalue())
            else:
                self.assertEqual(0, contract.main())

    def mapper_orders(self, generic: str) -> tuple[str, str, str]:
        probe = self.mappers["mapEspProbeError"]
        declaration = "Status mapEspError(esp_err_t error, const char* message);"
        return (
            generic + "\n\n" + probe,
            probe + "\n\n" + generic,
            declaration + "\n\n" + probe + "\n\n" + generic,
        )

    def test_valid_mappers_pass_in_every_order(self) -> None:
        orders = self.mapper_orders(self.mappers["mapEspError"])
        for order, definitions in enumerate(orders):
            with self.subTest(order=order):
                self.check_mappers(definitions)

    def test_bad_generic_classification_fails_in_every_order(self) -> None:
        for error in ("I2C_NACK_ADDR", "I2C_BUSY"):
            generic = self.mappers["mapEspError"].replace(
                "Err::I2C_ERROR", f"Err::{error}"
            )
            for order, definitions in enumerate(self.mapper_orders(generic)):
                with self.subTest(error=error, order=order):
                    self.check_mappers(
                        definitions,
                        "generic native I2C errors must not invent NACK or busy context",
                    )

    def test_forward_declarations_cannot_replace_definitions(self) -> None:
        for name in self.mappers:
            with self.subTest(name=name):
                definitions = "\n".join(
                    definition[:definition.index("{")].rstrip() + ";"
                    if mapper_name == name else definition
                    for mapper_name, definition in self.mappers.items()
                )
                self.check_mappers(definitions, "definition not found")


if __name__ == "__main__":
    unittest.main()
