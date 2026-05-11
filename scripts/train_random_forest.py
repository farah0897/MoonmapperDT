"""
Tren Random Forest-modellen på Triad-features.

Dette scriptet er en enkel snarvei til:
ml/training/train_random_forest.py

Standard:
- Leser ml/datasets/processed/triad_features.csv
- Skriver modellen til ml/models/random_forest.joblib
- Skriver label encoder til ml/models/label_encoder.joblib
"""

from __future__ import annotations

import _bootstrap_sys_path  # noqa: F401

import sys
from typing import List, Optional


def har_argument(argumenter: List[str], flagg: str) -> bool:
    """Sjekker om brukeren allerede har sendt inn et flagg."""
    for argument in argumenter:
        if argument == flagg:
            return True
        if argument.startswith(flagg + "="):
            return True
    return False


def legg_til_standard_argumenter(argumenter: List[str]) -> List[str]:
    """Legger til standard input/output for modelltrening."""
    if not har_argument(argumenter, "--input"):
        argumenter.extend(["--input", "ml/datasets/processed/triad_features.csv"])

    if not har_argument(argumenter, "--model-output"):
        argumenter.extend(["--model-output", "ml/models/random_forest.joblib"])

    if not har_argument(argumenter, "--encoder-output"):
        argumenter.extend(["--encoder-output", "ml/models/label_encoder.joblib"])

    return argumenter


def main(argv: Optional[List[str]] = None) -> int:
    from ml.training.train_random_forest import main as ekte_main  # type: ignore

    if argv is None:
        argumenter = list(sys.argv[1:])
    else:
        argumenter = list(argv)

    argumenter = legg_til_standard_argumenter(argumenter)

    # Vi setter sys.argv midlertidig fordi treningsscriptet bruker argparse selv.
    gammel_argv = sys.argv[:]
    try:
        sys.argv = [gammel_argv[0]] + argumenter
        return int(ekte_main())
    finally:
        sys.argv = gammel_argv


if __name__ == "__main__":
    raise SystemExit(main())
