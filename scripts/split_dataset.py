"""
Del feature-datasettet inn i train, val og test.

Dette scriptet er en enkel snarvei til:
ml/training/split_dataset.py

Standard:
- Leser ml/datasets/processed/triad_features.csv
- Skriver train.csv, val.csv og test.csv til ml/datasets/processed/
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def har_argument(argumenter: List[str], flagg: str) -> bool:
    """Sjekker om et terminalargument allerede finnes."""
    for argument in argumenter:
        if argument == flagg:
            return True
        if argument.startswith(flagg + "="):
            return True
    return False


def legg_til_standard_argumenter(argumenter: List[str]) -> List[str]:
    """Legger til vanlige filstier hvis brukeren ikke har valgt andre selv."""
    if not har_argument(argumenter, "--input"):
        argumenter.extend(["--input", "ml/datasets/processed/triad_features.csv"])

    if not har_argument(argumenter, "--out-dir"):
        argumenter.extend(["--out-dir", "ml/datasets/processed"])

    return argumenter


def main(argv: Optional[List[str]] = None) -> int:
    from ml.training.split_dataset import main as ekte_main  # type: ignore

    if argv is None:
        argumenter = list(sys.argv[1:])
    else:
        argumenter = list(argv)

    argumenter = legg_til_standard_argumenter(argumenter)

    # Den ekte main-funksjonen forventer argumenter i sys.argv.
    gammel_argv = sys.argv[:]
    try:
        sys.argv = [gammel_argv[0]] + argumenter
        return int(ekte_main())
    finally:
        sys.argv = gammel_argv


if __name__ == "__main__":
    raise SystemExit(main())
