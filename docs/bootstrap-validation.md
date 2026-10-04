# Bootstrap validation record

## Creation-time checks

This archive was produced in an environment without GNAT, Alire or native OpenCV.
Therefore it makes **no claim** that the Ada project or native shim has built or
that AUnit has run.

Creation-time standalone checks should include:

- `python3 scripts/check_repository.py`;
- Python configuration tests;
- C11 public C-header/layout syntax profile;
- standalone C++17 validation helper;
- shell syntax checks;
- `git diff --check` once placed in Git.

Native qualification remains Task 001.

## Qualification targets

Expected PR jobs: repository-checks, Linux, macOS, linux-sanitizers.
Windows is main-push-only. The manual pinned matrix targets OpenCV 4.1.0, 4.10.0
and 5.0.0. Do not replace this section with success claims until the exact commands
have actually run.
