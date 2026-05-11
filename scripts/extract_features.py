"""
Kjør feature-ekstraksjon for Triad-data.

Dette scriptet er bare en enkel snarvei. Selve logikken ligger i:
ml/training/extract_triad_features.py

Standard input/output:
- Leser metadata fra ml/datasets/metadata.csv
- Leser råfiler under ml/datasets/
- Skriver features til ml/datasets/processed/triad_features.csv
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def har_argument(argumenter: List[str], flagg: str) -> bool:
    """Sjekker om brukeren allerede har sendt inn et bestemt terminalflagg."""
    for argument in argumenter:
        if argument == flagg:
            return True
        if argument.startswith(flagg + "="):
            return True
    return False


def legg_til_standard_argumenter(argumenter: List[str]) -> List[str]:
    """
    Legger til standardstier hvis brukeren ikke har skrevet dem selv.

    Dette gjør at scriptet kan kjøres enkelt uten mange lange argumenter.
    """
    if not har_argument(argumenter, "--metadata"):
        argumenter.extend(["--metadata", "ml/datasets/metadata.csv"])

    if not har_argument(argumenter, "--raw-dir"):
        argumenter.extend(["--raw-dir", "ml/datasets"])

    if not har_argument(argumenter, "--output"):
        argumenter.extend(["--output", "ml/datasets/processed/triad_features.csv"])

    return argumenter


def main(argv: Optional[List[str]] = None) -> int:
    from ml.training.extract_triad_features import main as ekte_main  # type: ignore

    if argv is None:
        argumenter = list(sys.argv[1:])
    else:
        argumenter = list(argv)

    argumenter = legg_til_standard_argumenter(argumenter)

    # Den importerte main-funksjonen leser sys.argv, derfor setter vi den midlertidig.
    gammel_argv = sys.argv[:]
    try:
        sys.argv = [gammel_argv[0]] + argumenter
        return int(ekte_main())
    finally:
        sys.argv = gammel_argv


if __name__ == "__main__":
    raise SystemExit(main())
