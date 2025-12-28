#!/usr/bin/env python3
"""
Word Count Validation Script for Module 2 Digital Twin Documentation

Validates that MDX sections meet RAG-optimized word count targets (800-1500 words).
Excludes frontmatter and code blocks from word count.

Usage:
    python validate-word-count.py <mdx_file_or_directory>

Exit Codes:
    0 - All sections within target range (800-1500)
    1 - Warnings (sections outside target but within acceptable range)
    2 - Failures (sections severely outside acceptable range)
"""

import re
import sys
from pathlib import Path
from typing import List, Tuple


class WordCountValidator:
    """Validates MDX files for word count compliance."""

    # Word count thresholds
    TARGET_MIN = 800
    TARGET_MAX = 1500
    FAIL_MIN = 600
    FAIL_MAX = 2000

    def __init__(self):
        self.results: List[Tuple[str, str, int, str]] = []

    def validate_file(self, file_path: Path) -> None:
        """Validate word count for a single MDX file."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Remove frontmatter and code blocks
            content_no_code = self._remove_code_blocks(content)
            content_no_frontmatter = self._remove_frontmatter(content_no_code)

            # Count words
            word_count = self._count_words(content_no_frontmatter)

            # Extract section ID from frontmatter
            section_id = self._extract_section_id(content)

            # Determine status
            status = self._determine_status(word_count)

            self.results.append((str(file_path), section_id, word_count, status))

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
        # Remove inline code (keep for word count as they're part of explanations)
        # But remove very long code snippets in inline code
        content = re.sub(r'`[^`]{50,}`', '', content)
        return content

    def _count_words(self, content: str) -> int:
        """Count words in content (whitespace-separated tokens)."""
        # Remove markdown formatting characters
        content = re.sub(r'[#*_\[\]\(\)]+', ' ', content)
        # Split on whitespace and count
        words = content.split()
        return len(words)

    def _extract_section_id(self, content: str) -> str:
        """Extract section ID from frontmatter."""
        match = re.search(r'^---\n.*?^id:\s*([^\n]+)', content, flags=re.MULTILINE | re.DOTALL)
        if match:
            return match.group(1).strip()
        return "unknown"

    def _determine_status(self, word_count: int) -> str:
        """Determine validation status based on word count."""
        if self.TARGET_MIN <= word_count <= self.TARGET_MAX:
            return "✅ PASS"
        elif self.FAIL_MIN <= word_count < self.TARGET_MIN:
            return f"⚠️ WARN (below target, aim for {self.TARGET_MIN}+)"
        elif self.TARGET_MAX < word_count <= self.FAIL_MAX:
            return f"⚠️ WARN (above target, aim for <{self.TARGET_MAX})"
        elif word_count < self.FAIL_MIN:
            return f"❌ FAIL (too short, minimum {self.FAIL_MIN})"
        else:  # word_count > FAIL_MAX
            return f"❌ FAIL (too long, maximum {self.FAIL_MAX})"

    def print_report(self) -> None:
        """Print validation report to stdout."""
        print("Word Count Validation Report")
        print("=" * 80)
        print()

        # Print individual file results
        for file_path, section_id, word_count, status in sorted(self.results, key=lambda x: x[1]):
            print(f"Section: {section_id}")
            print(f"  File: {file_path}")
            print(f"  Word count: {word_count:,} words")
            print(f"  Status: {status}")
            print()

        # Calculate statistics
        total_sections = len(self.results)
        pass_count = sum(1 for _, _, _, status in self.results if "PASS" in status)
        warn_count = sum(1 for _, _, _, status in self.results if "WARN" in status)
        fail_count = sum(1 for _, _, _, status in self.results if "FAIL" in status)
        avg_word_count = sum(wc for _, _, wc, _ in self.results) / total_sections if total_sections > 0 else 0

        # Print summary
        print("=" * 80)
        print("Summary:")
        print(f"  Total sections: {total_sections}")
        print(f"  PASS (800-1500 words): {pass_count}")
        print(f"  WARN (outside target): {warn_count}")
        print(f"  FAIL (outside acceptable): {fail_count}")
        print(f"  Average word count: {avg_word_count:,.0f} words")
        print()

        if fail_count == 0 and warn_count == 0:
            print("  Overall: ✅ ALL SECTIONS PASS")
        elif fail_count == 0:
            print(f"  Overall: ⚠️ {warn_count} SECTION(S) NEED ADJUSTMENT")
        else:
            print(f"  Overall: ❌ {fail_count} SECTION(S) REQUIRE REVISION")

    def get_exit_code(self) -> int:
        """Return appropriate exit code based on validation results."""
        has_failures = any("FAIL" in status for _, _, _, status in self.results)
        has_warnings = any("WARN" in status for _, _, _, status in self.results)

        if has_failures:
            return 2  # Failures found
        elif has_warnings:
            return 1  # Warnings found
        else:
            return 0  # All sections pass


def main():
    """Main entry point."""
    if len(sys.argv) != 2:
        print("Usage: python validate-word-count.py <mdx_file_or_directory>", file=sys.stderr)
        sys.exit(3)

    path = Path(sys.argv[1])
    validator = WordCountValidator()

    if path.is_file():
        if path.suffix in ['.md', '.mdx']:
            validator.validate_file(path)
        else:
            print(f"Error: {path} is not an MDX/MD file", file=sys.stderr)
            sys.exit(3)
    elif path.is_dir():
        mdx_files = list(path.glob('**/*.mdx')) + list(path.glob('**/*.md'))
        if not mdx_files:
            print(f"Error: No MDX/MD files found in {path}", file=sys.stderr)
            sys.exit(3)
        for mdx_file in sorted(mdx_files):
            validator.validate_file(mdx_file)
    else:
        print(f"Error: {path} does not exist", file=sys.stderr)
        sys.exit(3)

    validator.print_report()
    sys.exit(validator.get_exit_code())


if __name__ == '__main__':
    main()
