#!/usr/bin/env python3
import subprocess
from pathlib import Path
import sys
from typing import List, Set

def format_files(target_dirs: List[str], file_extensions: Set[str]) -> int:
    """Format files in the given directories under bv_parti_root with clang-format."""
    formatted_count = 0
    for sub_dir in target_dirs:
        dir_path = bv_parti_root / sub_dir
        if not dir_path.is_dir():
            print(f"Skipping non-existent directory: {dir_path}")
            continue
        
        for file_path in dir_path.rglob('*'):
            if file_path.is_file() and file_path.suffix.lower() in file_extensions:
                print(f"Formatting {file_path}")
                cmd_args = ['clang-format', '-i', '-style=file', str(file_path)]
                try:
                    # Set cwd to bv_parti_root so that clang-format starts searching for .clang-format there.
                    subprocess.run(cmd_args, cwd=bv_parti_root, check=True)
                    formatted_count += 1
                except subprocess.CalledProcessError as e:
                    print(f"Error formatting {file_path}: {e}", file=sys.stderr)
    return formatted_count

if __name__ == '__main__':
    # Determine the bv_parti root directory. This script is assumed to be under bv_parti/script.
    script_dir = Path(__file__).resolve().parent
    bv_parti_root = script_dir.parent
    if not (bv_parti_root.is_dir()):
        print(f"Error: Expected bv_parti root directory, got {bv_parti_root}")
        sys.exit(1)
    
    print(f'Formatting all .c and .cpp files under "{bv_parti_root}"')
    file_extensions: Set[str] = {'.c', '.h', '.cpp', '.hpp'}
    format_dirs: List[str] = [bv_parti_root / 'src' / 'partitioner' / 'src' / 'preprocess' / 'pass' / 'feasible_domain_propagator']
        
    files_formatted = format_files(format_dirs, file_extensions)
    print(f"Formatting completed. {files_formatted} file(s) formatted.")
