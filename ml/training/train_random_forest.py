"""
Trener en enkel Random Forest-klassifikator for features hentet fra Triad-data.

Scriptet er laget for den prosesserte feature-tabellen fra:
  ml/training/extract_triad_features.py

Input:
- En prosessert CSV-fil (standard: ml/datasets/processed/triad_features.csv)
- En label-kolonne (standard: label_object)

Oppførsel:
- Fjerner metadata-kolonner automatisk, slik at bare numeriske features brukes
- Koder tekst-labels til tall med LabelEncoder og lagrer encoderen sammen med modellen
- Deler data i train/test med stratify når det er mulig
- Skriver ut accuracy, classification_report og confusion_matrix

Output:
- ml/models/random_forest_triad.joblib
- ml/models/label_encoder_triad.joblib

Avhengigheter installeres ikke automatisk. Hvis pandas/sklearn/joblib mangler,
skriver scriptet en tydelig feilmelding og avslutter uten traceback.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Tuple


NON_FEATURE_COLUMNS_DEFAULT = {
    # Metadata og labels skal ikke brukes som input-features til modellen.
    "sample_id",
    "label_material",
    "label_object",
    "sand_type",
    "lysforhold",
    "avstand_cm",
    "position_id",
    "angle_id",
    "run_id",
    "triad_file",
    "triad_file_resolved",
    "diameter_mm",
    "size_group",
    "surface_condition",
    "buried_level",
}


def _try_import_joblib() -> Tuple[object | None, str | None]:
    """Importerer joblib bare når scriptet faktisk skal lagre modellfiler."""
    try:
        import joblib  # type: ignore

        return joblib, None
    except Exception as exc:  # noqa: BLE001
        return None, str(exc)


def _select_feature_columns(df, *, label_column: str) -> list[str]:
    """
    Velger numeriske feature-kolonner fra den prosesserte Triad-CSV-en.
    Krever pandas.
    """
    import pandas as pd  # local import (optional dependency)

    cols: list[str] = []
    for c in df.columns:
        # Label-kolonnen er fasiten vi trener mot, og må ikke lekke inn i X.
        if c == label_column:
            continue
        # Metadata beskriver målingen, men er ikke sensor-features.
        if c in NON_FEATURE_COLUMNS_DEFAULT:
            continue
        cols.append(c)

    # Random Forest-modellen forventer numeriske verdier, så tekstkolonner filtreres bort.
    numeric_cols = [c for c in cols if pd.api.types.is_numeric_dtype(df[c])]
    return numeric_cols


def train_and_evaluate(df, *, label_column: str, random_state: int = 42, train_all: bool = False):
    """
    Trener og evaluerer en RandomForestClassifier på en dataframe.
    Krever pandas og scikit-learn.
    """
    import pandas as pd  # local import (optional dependency)
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
    from sklearn.model_selection import train_test_split
    from sklearn.preprocessing import LabelEncoder

    if label_column not in df.columns:
        raise ValueError(f"Missing label column: {label_column!r}")

    # Rader uten label kan ikke brukes til supervised learning.
    df = df.dropna(subset=[label_column]).copy()
    if df.empty:
        raise ValueError("No rows left after dropping missing labels.")

    feature_columns = _select_feature_columns(df, label_column=label_column)
    if not feature_columns:
        raise ValueError(
            "No numeric feature columns found. "
            "Expected columns like mean_0..mean_35, std_0.., min_0.., max_0.., rms_*, ratio_*."
        )

    X = df[feature_columns].fillna(0.0)
    y_raw = df[label_column].astype(str)

    # LabelEncoder gjør tekst-labels om til tallklasser som scikit-learn kan bruke.
    encoder = LabelEncoder()
    y = encoder.fit_transform(y_raw)

    model = RandomForestClassifier(
        n_estimators=300,
        random_state=random_state,
        n_jobs=-1,
        class_weight="balanced_subsample",
    )

    if train_all:
        # Når vi allerede har laget train/val/test filer, vil vi trene på hele train.csv.
        # Da skal ikke scriptet ta bort enda flere rader til en ny intern test.
        model.fit(X, y)
        print(f"Trained on all rows in input CSV: {len(X)} rows")
        return model, encoder, X, pd.Series(y), X.iloc[0:0], pd.Series(dtype=int)

    # Stratify holder klassefordelingen mest mulig lik i train og test.
    stratify = y if len(set(y.tolist())) > 1 else None
    X_train, X_test, y_train, y_test = train_test_split(
        X,
        y,
        test_size=0.2,
        random_state=random_state,
        stratify=stratify,
    )

    model.fit(X_train, y_train)

    # Evaluer modellen på holdout-testen som ikke ble brukt under trening.
    y_pred = model.predict(X_test)
    acc = accuracy_score(y_test, y_pred)
    print(f"Accuracy: {acc:.4f}")
    print()
    all_label_indices = list(range(len(encoder.classes_)))

    print("Confusion matrix (labels are encoded indices):")
    print(confusion_matrix(y_test, y_pred, labels=all_label_indices))
    print()
    print("Classification report:")
    print(
        classification_report(
            y_test,
            y_pred,
            labels=all_label_indices,
            target_names=list(encoder.classes_),
            zero_division=0,
        )
    )

    return model, encoder, X_train, pd.Series(y_train), X_test, pd.Series(y_test)


def main() -> int:
    # Kommandolinjeargumenter gjør scriptet fleksibelt for ulike datasett og output-stier.
    parser = argparse.ArgumentParser(description="Train Random Forest (Triad baseline).")
    parser.add_argument(
        "--input",
        type=str,
        default="ml/datasets/processed/triad_features.csv",
        help="Path to processed features CSV.",
    )
    parser.add_argument(
        "--label-column",
        type=str,
        default="label_object",
        help="Label column name (default: label_object).",
    )
    parser.add_argument(
        "--model-output",
        type=str,
        default="ml/models/random_forest.joblib",
        help="Where to write the trained model artifact (.joblib).",
    )
    parser.add_argument(
        "--encoder-output",
        type=str,
        default="ml/models/label_encoder.joblib",
        help="Where to write the label encoder artifact (.joblib).",
    )
    parser.add_argument(
        "--train-all",
        action="store_true",
        help="Train on all rows in the input CSV. Useful when input is already train.csv.",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Input CSV not found: {input_path}")
        print("Hint: generate synthetic data and run extract_triad_features.py first.")
        return 0

    joblib, joblib_err = _try_import_joblib()
    if joblib is None:
        print("joblib is not available. Install it in your ML environment to save models.")
        if joblib_err:
            print(f"Import error: {joblib_err}")
        return 0

    # pandas importeres her slik at brukeren får en ryddig feilmelding hvis miljøet mangler pakken.
    try:
        import pandas as pd  # type: ignore
    except Exception as exc:  # noqa: BLE001
        print("pandas is not available. Install pandas + scikit-learn to train the model.")
        print(f"Import error: {exc}")
        return 0

    # Les feature-tabellen som skal brukes til trening.
    try:
        df = pd.read_csv(input_path)
    except Exception as exc:  # noqa: BLE001
        print(f"[train_random_forest] Could not read CSV: {exc}")
        return 2
    try:
        # Selve treningen er samlet i én funksjon for å gjøre main enkel å lese/teste.
        model, encoder, _, _, _, _ = train_and_evaluate(
            df,
            label_column=args.label_column,
            train_all=args.train_all,
        )
    except Exception as exc:  # noqa: BLE001
        print("[train_random_forest] ERROR while training. Likely missing scikit-learn or invalid data.")
        print(str(exc))
        return 2

    # Lagre modellen og label-encoderen som .joblib-filer til senere prediksjon.
    model_out = Path(args.model_output)
    model_out.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, model_out)  # type: ignore[union-attr]
    print(f"Saved model: {model_out}")

    enc_out = Path(args.encoder_output)
    enc_out.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(encoder, enc_out)  # type: ignore[union-attr]
    print(f"Saved label encoder: {enc_out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

