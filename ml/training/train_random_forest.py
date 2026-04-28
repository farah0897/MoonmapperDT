"""
Train a baseline Random Forest classifier for Triad-only features.

This is a **skeleton script** intended for the early phase of the project.
It is safe to open/read even when no dataset exists yet.

Expected input (TODO):
- A processed CSV with extracted features per sample.
- A label column (default: taken from configs/labels.yaml primary_label).

Outputs:
- A saved model artifact via joblib (default: ml/models/random_forest.joblib)

TODO:
- Define the processed feature CSV schema (column naming conventions).
- Add support for reading `ml/configs/labels.yaml` and `ml/configs/feature_config.yaml`.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report
from sklearn.model_selection import train_test_split


def train(
    df: pd.DataFrame,
    *,
    label_column: str,
    feature_columns: list[str] | None = None,
    random_state: int = 42,
) -> tuple[RandomForestClassifier, pd.DataFrame, pd.Series, pd.DataFrame, pd.Series]:
    if label_column not in df.columns:
        raise ValueError(f"Missing label column: {label_column!r}")

    if feature_columns is None:
        # Default heuristic: everything except label is feature (TODO: make explicit).
        feature_columns = [c for c in df.columns if c != label_column]

    X = df[feature_columns]
    y = df[label_column]

    X_train, X_test, y_train, y_test = train_test_split(
        X,
        y,
        test_size=0.2,
        random_state=random_state,
        stratify=y if y.nunique() > 1 else None,
    )

    model = RandomForestClassifier(
        n_estimators=200,
        random_state=random_state,
        n_jobs=-1,
        class_weight="balanced",
    )
    model.fit(X_train, y_train)

    return model, X_train, y_train, X_test, y_test


def main() -> int:
    parser = argparse.ArgumentParser(description="Train Random Forest (Triad baseline, skeleton).")
    parser.add_argument(
        "--processed_csv",
        type=str,
        default="ml/datasets/processed/triad_features.csv",
        help="Path to processed features CSV (TODO: update when dataset exists).",
    )
    parser.add_argument(
        "--label_column",
        type=str,
        default="label_object",
        help="Label column name (TODO: align with ml/configs/labels.yaml).",
    )
    parser.add_argument(
        "--model_out",
        type=str,
        default="ml/models/random_forest.joblib",
        help="Where to write the trained model artifact.",
    )
    args = parser.parse_args()

    processed_path = Path(args.processed_csv)
    if not processed_path.exists():
        print(f"Processed CSV not found: {processed_path}")
        print("TODO: Generate processed dataset first (feature extraction + merge with labels).")
        return 0

    df = pd.read_csv(processed_path)
    model, _, _, X_test, y_test = train(df, label_column=args.label_column)

    y_pred = model.predict(X_test)
    acc = accuracy_score(y_test, y_pred)
    print(f"Accuracy: {acc:.4f}")
    print(classification_report(y_test, y_pred))

    # Save model artifact
    try:
        import joblib  # type: ignore
    except Exception:  # noqa: BLE001
        print("joblib is not available. TODO: install dependencies in your ML environment.")
        return 0

    model_out = Path(args.model_out)
    model_out.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, model_out)
    print(f"Saved model: {model_out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

