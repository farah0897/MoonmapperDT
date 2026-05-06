"""
Evaluate a trained model artifact against a feature CSV.

Features should come from:
  ml/training/extract_triad_features.py

This script:
- Loads a model + label encoder (joblib)
- Reads a CSV (default: ml/datasets/processed/test.csv)
- Drops non-feature columns automatically
- Prints confusion_matrix and classification_report
- Saves confusion matrix as PNG *if matplotlib is available*

Dependencies are NOT installed automatically.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from typing import List, Set


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate saved model (skeleton).")
    parser.add_argument(
        "--model_path",
        type=str,
        default="ml/models/random_forest.joblib",
        help="Path to joblib model artifact.",
    )
    parser.add_argument(
        "--encoder_path",
        type=str,
        default="ml/models/label_encoder.joblib",
        help="Path to joblib label encoder.",
    )
    parser.add_argument(
        "--input",
        type=str,
        default="ml/datasets/processed/test.csv",
        help="Path to feature CSV (e.g. test.csv from split_dataset.py).",
    )
    parser.add_argument(
        "--label-column",
        type=str,
        default="label_object",
        help="Label column name.",
    )
    parser.add_argument(
        "--cm-out",
        type=str,
        default="ml/datasets/processed/confusion_matrix.png",
        help="Where to write confusion matrix PNG (if matplotlib is available).",
    )
    args = parser.parse_args()

    model_path = Path(args.model_path)
    enc_path = Path(args.encoder_path)
    input_path = Path(args.input)

    if not model_path.exists():
        print(f"Model not found: {model_path}")
        print("Train and save a model first (train_random_forest.py).")
        return 0

    if not enc_path.exists():
        print(f"Label encoder not found: {enc_path}")
        print("Train and save an encoder first (train_random_forest.py).")
        return 0

    if not input_path.exists():
        print(f"Input CSV not found: {input_path}")
        print("Create train/val/test splits first (split_dataset.py).")
        return 0

    try:
        import joblib  # type: ignore
    except Exception:  # noqa: BLE001
        print("joblib is not available. TODO: install dependencies in your ML environment.")
        return 0

    try:
        import pandas as pd  # type: ignore
        from sklearn.metrics import classification_report, confusion_matrix  # type: ignore
    except Exception as exc:  # noqa: BLE001
        print("Missing dependencies for evaluation. Install pandas + scikit-learn.")
        print(f"Import error: {exc}")
        return 0

    model = joblib.load(model_path)
    encoder = joblib.load(enc_path)
    df = pd.read_csv(input_path)

    if args.label_column not in df.columns:
        raise ValueError(f"Missing label column: {args.label_column!r}")

    non_feature: Set[str] = {
        "sample_id",
        "label_material",
        "label_object",
        "sand_type",
        "lysforhold",
        "avstand_cm",
        "position_id",
        "angle_id",
        "triad_file",
        "triad_file_resolved",
        "diameter_mm",
        "size_group",
        "surface_condition",
        "buried_level",
        "run_id",
    }

    df = df.dropna(subset=[args.label_column]).copy()
    feature_columns: List[str] = []
    for c in df.columns:
        if c == args.label_column:
            continue
        if c in non_feature:
            continue
        if pd.api.types.is_numeric_dtype(df[c]):
            feature_columns.append(c)

    if not feature_columns:
        raise ValueError("No numeric feature columns found in input CSV.")

    X = df[feature_columns].fillna(0.0)
    y_true_raw = df[args.label_column].astype(str)

    # Encode labels to match training encoding
    try:
        y_true = encoder.transform(y_true_raw)
    except Exception:  # noqa: BLE001
        # Fallback: if encoder doesn't match, evaluate on raw labels where possible
        y_true = y_true_raw

    y_pred = model.predict(X)
    print("Confusion matrix:")
    cm = confusion_matrix(y_true, y_pred)
    print(cm)
    print()
    print("Classification report:")
    try:
        target_names = list(getattr(encoder, "classes_", []))
        if target_names:
            print(classification_report(y_true, y_pred, target_names=target_names))
        else:
            print(classification_report(y_true, y_pred))
    except Exception:  # noqa: BLE001
        print(classification_report(y_true, y_pred))

    # Optional: save confusion matrix plot
    try:
        import matplotlib.pyplot as plt  # type: ignore

        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(cm, interpolation="nearest")
        ax.set_title("Confusion matrix")
        ax.set_xlabel("Predicted")
        ax.set_ylabel("True")
        plt.tight_layout()
        out = Path(args.cm_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=150)
        plt.close(fig)
        print(f"Saved confusion matrix PNG: {out}")
    except Exception:
        print("matplotlib not available (or failed). Skipping PNG output.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

