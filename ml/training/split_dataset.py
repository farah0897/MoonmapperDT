"""
Split a dataset into train/val/test CSVs.

Skeleton script:
- Reads a metadata CSV or processed feature CSV
- Splits into train/val/test
- Uses stratify when label is available

TODO:
- Decide whether split input is metadata_template.csv or processed feature table
- Add group-aware splitting (e.g., by run_id) to prevent leakage
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
from sklearn.model_selection import train_test_split


def split_dataframe(
    df: pd.DataFrame,
    *,
    label_column: str | None = None,
    train_frac: float = 0.7,
    val_frac: float = 0.15,
    test_frac: float = 0.15,
    random_state: int = 42,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    if abs((train_frac + val_frac + test_frac) - 1.0) > 1e-6:
        raise ValueError("train_frac + val_frac + test_frac must sum to 1.0")

    stratify = None
    if label_column and label_column in df.columns and df[label_column].nunique() > 1:
        stratify = df[label_column]

    df_train, df_temp = train_test_split(
        df,
        test_size=(1.0 - train_frac),
        random_state=random_state,
        stratify=stratify,
    )

    # Split remaining into val/test
    remaining = val_frac + test_frac
    val_ratio = val_frac / remaining if remaining > 0 else 0.5

    stratify_temp = None
    if label_column and label_column in df_temp.columns and df_temp[label_column].nunique() > 1:
        stratify_temp = df_temp[label_column]

    df_val, df_test = train_test_split(
        df_temp,
        test_size=(1.0 - val_ratio),
        random_state=random_state,
        stratify=stratify_temp,
    )

    return df_train, df_val, df_test


def main() -> int:
    parser = argparse.ArgumentParser(description="Split dataset into train/val/test CSVs (skeleton).")
    parser.add_argument(
        "--input_csv",
        type=str,
        default="ml/datasets/processed/triad_features.csv",
        help="Input CSV (metadata or processed features).",
    )
    parser.add_argument(
        "--label_column",
        type=str,
        default="label_object",
        help="Optional label column for stratification.",
    )
    parser.add_argument(
        "--out_dir",
        type=str,
        default="ml/datasets/processed/splits",
        help="Output directory for split CSVs.",
    )
    args = parser.parse_args()

    input_path = Path(args.input_csv)
    if not input_path.exists():
        print(f"Input CSV not found: {input_path}")
        print("TODO: Provide a real processed dataset CSV first.")
        return 0

    df = pd.read_csv(input_path)

    label_column = args.label_column if args.label_column in df.columns else None
    if label_column is None:
        print("Label column not found (or single-class). Splitting without stratify.")

    df_train, df_val, df_test = split_dataframe(df, label_column=label_column)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    train_path = out_dir / "train.csv"
    val_path = out_dir / "val.csv"
    test_path = out_dir / "test.csv"
    df_train.to_csv(train_path, index=False)
    df_val.to_csv(val_path, index=False)
    df_test.to_csv(test_path, index=False)

    print(f"Wrote: {train_path} ({len(df_train)})")
    print(f"Wrote: {val_path} ({len(df_val)})")
    print(f"Wrote: {test_path} ({len(df_test)})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

