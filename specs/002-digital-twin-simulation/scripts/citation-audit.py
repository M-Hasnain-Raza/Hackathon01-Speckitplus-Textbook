#!/usr/bin/env python3
"""
Citation Audit Script for Module 2 Digital Twin Documentation

Validates that all factual claims in MDX files have proper APA citations.
Detects uncited technical claims and unresolved [CITE] placeholders.

Usage:
    python citation-audit.py <mdx_file_or_directory>

Exit Codes:
    0 - All checks passed
    1 - Warnings found (uncited claims)
    2 - Errors found (unresolved placeholders)
"""

import re
import sys
from pathlib import Path
from typing import List, Tuple, Dict


class CitationAuditor:
    """Audits MDX files for citation compliance."""

    # Patterns that indicate factual claims requiring citations
    FACTUAL_CLAIM_PATTERNS = [
        r'v\d+\.\d+',  # Version numbers (e.g., v1.2.3)
        r'ROS 2 (Humble|Iron|Jazzy)',  # ROS 2 distributions
        r'Gazebo (Fortress|Garden|Harmonic)',  # Gazebo versions
        r'Unity \d{4}\.\d+',  # Unity versions
        r'\bdefault\b.*?(engine|value|parameter)',  # Default configurations
        r'\btypical(ly)?\b.*?\d+',  # Typical values with numbers
        r'\b(minimum|maximum|min|max)\b.*?\d+',  # Min/max values
        r'\d+\.?\d*\s*(seconds|Hz|meters|m\/s|fps)',  # Numbers with units
        r'`[^`]*?(error|Error|ERROR|warning|Warning)[^`]*?`',  # Error messages
        r'\bPlugin:\s*`[^`]+`',  # Plugin names
        r'Message type:.*?`[^`]+`',  # Message types
    ]

    # Pattern for detecting APA citations
    CITATION_PATTERN = r'\([A-Z][^)]+,\s*\d{4}[^)]*\)'

    # Pattern for unresolved citation placeholders
    PLACEHOLDER_PATTERN = r'\[CITE\]'

    def __init__(self):
        self.warnings: List[Tuple[str, int, str]] = []
        self.errors: List[Tuple[str, int, str]] = []

    def audit_file(self, file_path: Path) -> None:
        """Audit a single MDX file for citation compliance."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Skip frontmatter and code blocks
            content_no_code = self._remove_code_blocks(content)
            content_no_frontmatter = self._remove_frontmatter(content_no_code)

            # Check for unresolved placeholders (ERROR)
            self._check_placeholders(file_path, content_no_frontmatter)

            # Check for uncited factual claims (WARNING)
            self._check_factual_claims(file_path, content_no_frontmatter)

        except Exception as e:
            print(f"Error processing {file_path}: {e}", file=sys.stderr)

    def _remove_frontmatter(self, content: str) -> str:
        """Remove YAML frontmatter from content."""
        frontmatter_pattern = r'^---\n.*?\n---\n'
        return re.sub(frontmatter_pattern, '', content, flags=re.DOTALL)

    def _remove_code_blocks(self, content: str) -> str:
        """Remove code blocks from content."""
        # Remove fenced code blocks (```...```)
        content = re.sub(r'```[^`]*?```', '', content, flags=re.DOTALL)
        # Remove inline code (`...`)
        content = re.sub(r'`[^`]+`', '', content)
        return content

    def _check_placeholders(self, file_path: Path, content: str) -> None:
        """Check for unresolved [CITE] placeholders."""
        lines = content.split('\n')
        for line_num, line in enumerate(lines, start=1):
            if re.search(self.PLACEHOLDER_PATTERN, line):
                excerpt = line[:60].strip() + ('...' if len(line) > 60 else '')
                self.errors.append((str(file_path), line_num, excerpt))

    def _check_factual_claims(self, file_path: Path, content: str) -> None:
        """Check for factual claims without nearby citations."""
        lines = content.split('\n')

        for line_num, line in enumerate(lines, start=1):
            # Check if line contains factual claim pattern
            has_factual_claim = any(
                re.search(pattern, line, re.IGNORECASE)
                for pattern in self.FACTUAL_CLAIM_PATTERNS
            )

            if not has_factual_claim:
                continue

            # Check if citation exists in current line or next 2 lines
            context_window = '\n'.join(lines[max(0, line_num-1):min(len(lines), line_num+2)])
            has_citation = re.search(self.CITATION_PATTERN, context_window)

            if not has_citation:
                excerpt = line[:60].strip() + ('...' if len(line) > 60 else '')
                self.warnings.append((str(file_path), line_num, excerpt))

    def print_report(self) -> None:
        """Print audit report to stdout."""
        print("Citation Audit Report")
        print("=" * 80)
        print()

        # Group warnings by file
        warnings_by_file: Dict[str, List[Tuple[int, str]]] = {}
        for file_path, line_num, excerpt in self.warnings:
            if file_path not in warnings_by_file:
                warnings_by_file[file_path] = []
            warnings_by_file[file_path].append((line_num, excerpt))

        # Group errors by file
        errors_by_file: Dict[str, List[Tuple[int, str]]] = {}
        for file_path, line_num, excerpt in self.errors:
            if file_path not in errors_by_file:
                errors_by_file[file_path] = []
            errors_by_file[file_path].append((line_num, excerpt))

        # Print errors
        if errors_by_file:
            print("ERRORS (Unresolved [CITE] placeholders):")
            print("-" * 80)
            for file_path, issues in sorted(errors_by_file.items()):
                print(f"\nFile: {file_path}")
                for line_num, excerpt in issues:
                    print(f"  [ERROR] Line {line_num}: {excerpt}")
            print()

        # Print warnings
        if warnings_by_file:
            print("WARNINGS (Potential uncited factual claims):")
            print("-" * 80)
            for file_path, issues in sorted(warnings_by_file.items()):
                print(f"\nFile: {file_path}")
                for line_num, excerpt in issues:
                    print(f"  [WARNING] Line {line_num}: {excerpt}")
            print()

        # Print summary
        print("=" * 80)
        print("Summary:")
        print(f"  Total files scanned: {len(set(f for f, _, _ in self.warnings + self.errors))}")
        print(f"  Uncited claims (warnings): {len(self.warnings)}")
        print(f"  Unresolved placeholders (errors): {len(self.errors)}")

        if self.warnings or self.errors:
            total_issues = len(self.warnings) + len(self.errors)
            print(f"  Citation compliance: NEEDS REVIEW ({total_issues} issues)")
        else:
            print("  Citation compliance: ✅ PASS")

    def get_exit_code(self) -> int:
        """Return appropriate exit code based on audit results."""
        if self.errors:
            return 2  # Errors found
        elif self.warnings:
            return 1  # Warnings found
        else:
            return 0  # All checks passed


def main():
    """Main entry point."""
    if len(sys.argv) != 2:
        print("Usage: python citation-audit.py <mdx_file_or_directory>", file=sys.stderr)
        sys.exit(3)

    path = Path(sys.argv[1])
    auditor = CitationAuditor()

    if path.is_file():
        if path.suffix in ['.md', '.mdx']:
            auditor.audit_file(path)
        else:
            print(f"Error: {path} is not an MDX/MD file", file=sys.stderr)
            sys.exit(3)
    elif path.is_dir():
        mdx_files = list(path.glob('**/*.mdx')) + list(path.glob('**/*.md'))
        if not mdx_files:
            print(f"Error: No MDX/MD files found in {path}", file=sys.stderr)
            sys.exit(3)
        for mdx_file in sorted(mdx_files):
            auditor.audit_file(mdx_file)
    else:
        print(f"Error: {path} does not exist", file=sys.stderr)
        sys.exit(3)

    auditor.print_report()
    sys.exit(auditor.get_exit_code())


if __name__ == '__main__':
    main()
