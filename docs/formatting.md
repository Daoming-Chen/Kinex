# Code formatting

Kinex uses clang-format following the Google C++ style. A `.clang-format` file is included at the repo root.

Files: C/C++ files are formatted using `IndentWidth: 2`, `ColumnLimit: 100` and includes are sorted.

Quick usage:

1. Format everything:

```bash
./scripts/format.sh
```

2. Format staged files automatically on commit:

- Set git hooks path once (optional):
```bash
git config core.hooksPath .githooks
```
- Ensure hooks are executable:
```bash
chmod +x .githooks/pre-commit
```

CI: A GitHub Actions workflow runs clang-format on PRs and fails if formatting changes are required.
