#!/usr/bin/env python3
"""
Charset testing and conversion utility.
Detects file encodings and converts files to UTF-8.
"""

import os
import sys
from pathlib import Path

try:
    import chardet
except ImportError:
    print("Installing chardet...")
    os.system("pip install chardet")
    import chardet


def detect_encoding(file_path: str) -> dict:
    """Detect the encoding of a file."""
    with open(file_path, 'rb') as f:
        raw_data = f.read()
        result = chardet.detect(raw_data)
        return result


def is_utf8(file_path: str) -> bool:
    """Check if a file is UTF-8 encoded."""
    encoding_info = detect_encoding(file_path)
    return encoding_info['encoding'].lower().replace('-', '') == 'utf8'


def convert_to_utf8(file_path: str, original_encoding: str = None, backup: bool = True) -> bool:
    """
    Convert a file to UTF-8 encoding.

    Args:
        file_path: Path to the file to convert
        original_encoding: Source encoding (auto-detected if None)
        backup: Create a backup before converting

    Returns:
        True if successful, False otherwise
    """
    if backup:
        backup_path = f"{file_path}.bak"
        if not os.path.exists(backup_path):
            os.replace(file_path, backup_path)
            source_file = backup_path
        else:
            source_file = file_path
    else:
        source_file = file_path

    # Detect encoding if not provided
    if original_encoding is None:
        encoding_info = detect_encoding(source_file)
        original_encoding = encoding_info['encoding']
        confidence = encoding_info.get('confidence', 0)
        print(f"  Detected: {original_encoding} (confidence: {confidence:.1%})")

    try:
        # Read with original encoding
        with open(source_file, 'r', encoding=original_encoding) as f:
            content = f.read()

        # Write as UTF-8
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)

        print(f"  ✓ Converted to UTF-8")
        return True

    except Exception as e:
        print(f"  ✗ Error: {e}")
        # Restore backup if it exists
        if backup and os.path.exists(f"{file_path}.bak"):
            os.replace(f"{file_path}.bak", file_path)
        return False


def scan_directory(directory: str, extensions: list = None) -> list:
    """Scan directory for text files to check."""
    if extensions is None:
        extensions = ['.md', '.txt', '.java', '.py', '.json', '.xml', '.yaml', '.yml']

    files = []
    for root, dirs, filenames in os.walk(directory):
        # Skip common directories to ignore
        dirs[:] = [d for d in dirs if d not in {'.git', 'build', 'gradle', 'node_modules', '.idea'}]

        for filename in filenames:
            if any(filename.endswith(ext) for ext in extensions):
                files.append(os.path.join(root, filename))
    return files


def main():
    if len(sys.argv) < 2:
        print("Charset Converter Utility")
        print("\nUsage:")
        print("  python charset_converter.py test <file>          - Test file encoding")
        print("  python charset_converter.py convert <file>       - Convert file to UTF-8")
        print("  python charset_converter.py scan <directory>     - Scan directory for non-UTF8 files")
        print("  python charset_converter.py fix <directory>      - Convert all non-UTF8 files in directory")
        sys.exit(1)

    command = sys.argv[1]
    target = sys.argv[2] if len(sys.argv) > 2 else "."

    if command == "test":
        if os.path.isfile(target):
            info = detect_encoding(target)
            utf8 = is_utf8(target)
            print(f"File: {target}")
            print(f"  Encoding: {info['encoding']}")
            print(f"  Confidence: {info['confidence']:.1%}")
            print(f"  Is UTF-8: {'Yes' if utf8 else 'No'}")
        else:
            print(f"Error: {target} is not a file")

    elif command == "convert":
        if os.path.isfile(target):
            print(f"Converting: {target}")
            if is_utf8(target):
                print("  Already UTF-8, skipping.")
            else:
                convert_to_utf8(target)
        else:
            print(f"Error: {target} is not a file")

    elif command == "scan":
        if os.path.isdir(target):
            print(f"Scanning: {target}")
            files = scan_directory(target)
            non_utf8 = []

            for f in files:
                try:
                    if not is_utf8(f):
                        info = detect_encoding(f)
                        non_utf8.append((f, info['encoding'], info['confidence']))
                except Exception as e:
                    print(f"  Error reading {f}: {e}")

            if non_utf8:
                print(f"\nFound {len(non_utf8)} non-UTF8 files:")
                for f, enc, conf in non_utf8:
                    print(f"  {f}")
                    print(f"    Encoding: {enc} (confidence: {conf:.1%})")
            else:
                print("\n✓ All files are UTF-8")
        else:
            print(f"Error: {target} is not a directory")

    elif command == "fix":
        if os.path.isdir(target):
            print(f"Scanning and fixing: {target}")
            files = scan_directory(target)

            for f in files:
                try:
                    if not is_utf8(f):
                        print(f"\n{f}:")
                        convert_to_utf8(f)
                except Exception as e:
                    print(f"  Error: {e}")
        else:
            print(f"Error: {target} is not a directory")

    else:
        print(f"Unknown command: {command}")
        sys.exit(1)


if __name__ == "__main__":
    main()
