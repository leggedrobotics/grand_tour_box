#!/usr/bin/env python3
"""Extract opted-in email addresses from the GrandTour CSV for a BCC field."""

import argparse
import csv
import re
import sys
from pathlib import Path


EMAIL_COLUMN = 5  # Column F (zero-based index)
ANSWER_COLUMN = 6  # Column G (zero-based index)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Print a comma-separated BCC list for rows that answer Yes but not No in column G."
    )
    parser.add_argument(
        "csv_file",
        type=Path,
        help="CSV containing the opt-in responses (columns F and G)",
    )
    return parser.parse_args()


def extract_emails(csv_file):
    emails = []
    seen = set()
    response_counts = {"yes": 0, "no": 0, "ambiguous": 0}

    with csv_file.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None or len(header) <= ANSWER_COLUMN:
            raise ValueError("The CSV does not contain the expected columns F and G.")

        for row_number, row in enumerate(reader, start=2):
            if len(row) <= ANSWER_COLUMN:
                print(f"Warning: skipping incomplete row {row_number}.", file=sys.stderr)
                continue

            answer = row[ANSWER_COLUMN].strip().casefold()
            has_yes = bool(re.search(r"\byes\b", answer))
            has_no = bool(re.search(r"\bno\b", answer))

            if has_yes and has_no:
                response_counts["ambiguous"] += 1
            elif has_yes:
                response_counts["yes"] += 1
            elif has_no:
                response_counts["no"] += 1

            if not has_yes or has_no:
                continue

            email = row[EMAIL_COLUMN].strip()
            if not email:
                print(f"Warning: opted-in row {row_number} has no email in column F.", file=sys.stderr)
                continue
            if not re.fullmatch(r"[^\s@,;<>]+@[^\s@,;<>]+\.[^\s@,;<>]+", email):
                print(f"Warning: skipping invalid email in row {row_number}.", file=sys.stderr)
                continue

            deduplication_key = email.casefold()
            if deduplication_key not in seen:
                seen.add(deduplication_key)
                emails.append(email)

    return emails, response_counts


def main():
    args = parse_args()
    try:
        emails, response_counts = extract_emails(args.csv_file)
    except (OSError, ValueError, csv.Error) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    # A single line that can be pasted directly into a BCC field.
    print(", ".join(emails))
    print("Response statistics:", file=sys.stderr)
    print(f"  Yes: {response_counts['yes']}", file=sys.stderr)
    print(f"  No: {response_counts['no']}", file=sys.stderr)
    print(f"  Ambiguous (Yes and No): {response_counts['ambiguous']}", file=sys.stderr)
    print(f"Extracted {len(emails)} unique email addresses.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
