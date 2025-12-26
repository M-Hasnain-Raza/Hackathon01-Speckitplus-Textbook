# Contributing to ROS 2 Nervous System Documentation

First off, thank you for considering contributing to this project! It's people like you that make educational resources better for everyone.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How Can I Contribute?](#how-can-i-contribute)
- [Development Setup](#development-setup)
- [Contribution Workflow](#contribution-workflow)
- [Style Guidelines](#style-guidelines)
- [Commit Message Format](#commit-message-format)
- [Pull Request Process](#pull-request-process)
- [Review Process](#review-process)

## Code of Conduct

This project adheres to a Code of Conduct that all contributors are expected to follow. By participating, you are expected to uphold this code.

### Our Standards

**Positive behaviors**:
- Using welcoming and inclusive language
- Being respectful of differing viewpoints and experiences
- Gracefully accepting constructive criticism
- Focusing on what is best for the community
- Showing empathy towards other community members

**Unacceptable behaviors**:
- The use of sexualized language or imagery
- Trolling, insulting/derogatory comments, and personal or political attacks
- Public or private harassment
- Publishing others' private information without explicit permission
- Other conduct which could reasonably be considered inappropriate

## How Can I Contribute?

### Reporting Bugs

**Before submitting a bug report**:
- Check the [existing issues](https://github.com/yourusername/hackathon01-Textbook/issues) to avoid duplicates
- Verify the issue with the latest version
- Collect relevant information (ROS 2 version, OS, error messages)

**How to submit a bug report**:
1. Use the issue template
2. Provide a clear, descriptive title
3. Describe the expected behavior vs actual behavior
4. Include steps to reproduce
5. Attach screenshots, code snippets, or logs if applicable

### Suggesting Enhancements

We welcome suggestions for:
- New sections or topics
- Improved explanations
- Additional code examples
- Better diagrams or visualizations
- Accessibility improvements

**How to submit an enhancement**:
1. Check if the enhancement has already been suggested
2. Provide a clear use case
3. Explain why this would be valuable to learners
4. Suggest implementation approach if you have ideas

### Contributing Content

We accept contributions for:

**Documentation improvements**:
- Fixing typos, grammar, or formatting
- Clarifying confusing explanations
- Adding missing information
- Updating outdated links or citations

**Code examples**:
- New practical examples
- Improvements to existing examples
- Bug fixes in example code
- Better inline comments

**Diagrams and visualizations**:
- New Mermaid diagrams
- Improvements to existing diagrams
- Alternative visual representations

## Development Setup

### Prerequisites

- Python 3.8+
- ROS 2 (Humble Hawksbill or later)
- Git
- Text editor (VS Code, Vim, etc.)
- (Optional) Docusaurus for local preview

### Local Setup

1. **Fork the repository**:
   ```bash
   # Click "Fork" on GitHub, then clone your fork
   git clone https://github.com/YOUR_USERNAME/hackathon01-Textbook.git
   cd hackathon01-Textbook
   ```

2. **Create a feature branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Set up ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. **(Optional) Install Docusaurus** for local preview:
   ```bash
   npm install
   npm start
   ```

## Contribution Workflow

### 1. Find or Create an Issue

- Check [existing issues](https://github.com/yourusername/hackathon01-Textbook/issues)
- Create a new issue if your contribution doesn't have one
- Comment on the issue to claim it

### 2. Create a Feature Branch

```bash
git checkout -b feature/short-description
# or
git checkout -b fix/bug-description
```

Branch naming conventions:
- `feature/add-navigation-example` - New features
- `fix/typo-section-03` - Bug fixes
- `docs/improve-urdf-explanation` - Documentation improvements
- `refactor/update-code-style` - Code refactoring

### 3. Make Your Changes

**For documentation changes**:
- Follow the [style guidelines](#style-guidelines)
- Maintain consistent terminology (see `specs/001-ros2-nervous-system/spec.md`)
- Test all code examples
- Run validation checks (see below)

**For code examples**:
- Include proper imports
- Add inline comments explaining ROS 2 concepts
- Use consistent naming conventions
- Test that code runs without errors

### 4. Validate Your Changes

**Documentation validation**:
```bash
# Word count check (should be 600-700 words per section)
wc -w docs/module-01-ros2-nervous-system/XX-your-section.md

# Check markdown formatting
# (Use your editor's markdown linter)
```

**Code validation**:
```bash
# Test Python code examples
python3 your_example.py

# Check Python style (optional)
flake8 your_example.py
```

**Terminology consistency**:
- ✅ "ROS 2" (with space)
- ✅ "rclpy" (lowercase)
- ✅ "Publisher/Subscriber" (full terms)
- ❌ "ROS2" (no space)
- ❌ "Rclpy" (capitalized)

### 5. Commit Your Changes

Follow the [commit message format](#commit-message-format):

```bash
git add .
git commit -m "feat(section-03): add node lifecycle code example

- Add complete example demonstrating node initialization
- Include error handling with try-except-finally
- Add inline comments explaining rclpy API calls

Closes #42"
```

### 6. Push and Create Pull Request

```bash
git push origin feature/your-feature-name
```

Then create a pull request on GitHub.

## Style Guidelines

### Markdown Style

**Headings**:
- Use ATX-style headings (`#`, `##`, `###`)
- Maximum depth: h3 (avoid h4, h5, h6 for scannability)
- One h1 per document (the title)

**Code blocks**:
```python title="filename.py"
# Use triple backticks with language tag
# Add title attribute for file names
import rclpy
from rclpy.node import Node
```

**Lists**:
- Use `-` for unordered lists (consistent with existing files)
- Use `1.` for ordered lists
- Add blank line before and after lists

**Emphasis**:
- `**bold**` for important terms on first use
- `*italic*` for gentle emphasis
- `` `code` `` for inline code, commands, file names

**Links**:
- Use descriptive link text (not "click here")
- Include retrieval dates for citations: `(retrieved YYYY-MM-DD)`
- Format: `[Link Text](URL) (retrieved 2025-12-26)`

### Code Style

**Python**:
- Follow PEP 8 style guide
- Use 4 spaces for indentation
- Maximum line length: 88 characters (Black formatter)
- Include docstrings for classes and non-trivial functions
- Add inline comments for ROS 2-specific code

**Example structure**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):
    """Brief description of what this node does."""

    def __init__(self):
        super().__init__('example_node')
        # Create publisher with clear comment
        self.publisher = self.create_publisher(String, '/topic', 10)
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Diagram Style

**Mermaid diagrams**:
- Use consistent color scheme:
  - Blue (#e1f5ff): Input/initialization
  - Yellow (#fff4e1): Processing/computation
  - Red (#ffe1e1): Output/shutdown
  - Gray (#f0f0f0): Hardware/external
- Include figure captions: `**Figure N**: Description`
- Add textual explanation after diagram

## Commit Message Format

We follow the [Conventional Commits](https://www.conventionalcommits.org/) specification:

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Type

- `feat`: New feature or content
- `fix`: Bug fix or correction
- `docs`: Documentation-only changes
- `style`: Formatting, missing semicolons, etc. (no code change)
- `refactor`: Code restructuring (no feature change)
- `test`: Adding or updating tests
- `chore`: Build process, dependencies, etc.

### Scope

- `section-01` through `section-11` - Specific section
- `validation` - Validation reports
- `diagrams` - Mermaid diagrams
- `examples` - Code examples
- `spec` - Specification files

### Subject

- Use imperative mood ("add" not "added")
- Don't capitalize first letter
- No period at the end
- Maximum 50 characters

### Body (optional)

- Explain **what** and **why** (not how)
- Wrap at 72 characters
- Use bullet points for multiple changes

### Footer (optional)

- Reference issues: `Closes #42` or `Fixes #123`
- Breaking changes: `BREAKING CHANGE: description`

### Examples

**Good commit messages**:
```
feat(section-04): add multi-threaded publisher example

- Demonstrate concurrent message publishing
- Include comments on thread safety
- Show proper executor usage

Closes #45
```

```
fix(section-08): correct rclpy lifecycle diagram

The previous diagram showed incorrect order of destroy_node()
and shutdown() calls. This fix ensures proper cleanup sequence.

Fixes #67
```

```
docs(readme): update installation instructions

Add missing step for sourcing ROS 2 environment before
running examples. Clarify Python version requirement.
```

**Poor commit messages**:
```
fixed stuff                          # Too vague
Updated section 3                    # What changed?
Added example.                       # Missing scope, details
FEAT: Add example                    # Don't capitalize type
```

## Pull Request Process

### Before Submitting

**Checklist**:
- [ ] Code examples run without errors
- [ ] Markdown formatting is correct
- [ ] Terminology standards followed
- [ ] Citations include retrieval dates
- [ ] Diagrams have figure captions
- [ ] Comprehension questions have answers
- [ ] Commit messages follow format
- [ ] Branch is up to date with `main`

### PR Description Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Documentation update
- [ ] Breaking change (fix or feature that would cause existing functionality to change)

## Related Issue
Closes #(issue number)

## Changes Made
- Change 1
- Change 2
- Change 3

## Testing
Describe how you tested your changes:
- [ ] Ran code examples
- [ ] Checked markdown rendering
- [ ] Verified links work
- [ ] Tested on ROS 2 Humble

## Screenshots (if applicable)
Add screenshots of diagrams, rendered markdown, etc.

## Checklist
- [ ] My code follows the style guidelines
- [ ] I have performed a self-review
- [ ] I have commented my code where needed
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective
- [ ] New and existing tests pass locally
```

### Review Process

1. **Automated checks** (if configured):
   - Markdown linting
   - Link validation
   - Spelling check

2. **Maintainer review**:
   - Content accuracy
   - Style consistency
   - Pedagogical quality
   - Technical correctness

3. **Feedback incorporation**:
   - Address reviewer comments
   - Push additional commits to same branch
   - Re-request review when ready

4. **Merge**:
   - Maintainer merges when approved
   - Branch is automatically deleted

## Recognition

Contributors will be:
- Listed in [CONTRIBUTORS.md](./CONTRIBUTORS.md)
- Credited in release notes
- Acknowledged in documentation updates

## Questions?

- **General questions**: [GitHub Discussions](https://github.com/yourusername/hackathon01-Textbook/discussions)
- **Bugs or issues**: [GitHub Issues](https://github.com/yourusername/hackathon01-Textbook/issues)
- **Security concerns**: Email security@example.com

---

Thank you for contributing to make robotics education more accessible! 🤖

*Last Updated: 2025-12-26*
