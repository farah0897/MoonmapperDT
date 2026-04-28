"""
Evaluate a trained model artifact against test data.

Skeleton script:
- Loads a joblib model
- Loads a test CSV
- Prints confusion matrix and classification report

TODO:
- Align feature columns with your processed dataset schema
- Add probability calibration, confidence thresholds, and per-class metrics
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
from sklearn.metrics import classification_report, confusion_matrix


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate saved model (skeleton).")
    parser.add_argument(
        "--model_path",
        type=str,
        default="ml/models/random_forest.joblib",
        help="Path to joblib model artifact.",
    )
    parser.add_argument(
        "--test_csv",
        type=str,
        default="ml/datasets/processed/test.csv",
        help="Path to test CSV (TODO: generate via split_dataset.py).",
    )
    parser.add_argument(
        "--label_column",
        type=str,
        default="label_object",
        help="Label column name.",
    )
    args = parser.parse_args()

    model_path = Path(args.model_path)
    test_path = Path(args.test_csv)

    if not model_path.exists():
        print(f"Model not found: {model_path}")
        print("TODO: Train and save a model first (train_random_forest.py).")
        return 0

    if not test_path.exists():
        print(f"Test CSV not found: {test_path}")
        print("TODO: Create train/val/test split first (split_dataset.py).")
        return 0

    try:
        import joblib  # type: ignore
    except Exception:  # noqa: BLE001
        print("joblib is not available. TODO: install dependencies in your ML environment.")
        return 0

    model = joblib.load(model_path)
    df = pd.read_csv(test_path)

    if args.label_column not in df.columns:
        raise ValueError(f"Missing label column: {args.label_column!r}")

    # TODO: make explicit feature columns
    feature_columns = [c for c in df.columns if c != args.label_column]
    X = df[feature_columns]
    y_true = df[args.label_column]

    y_pred = model.predict(X)
    print("Confusion matrix:")
    print(confusion_matrix(y_true, y_pred))
    print()
    print("Classification report:")
    print(classification_report(y_true, y_pred))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

