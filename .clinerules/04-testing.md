# Testing

- Warnings remain errors.
- Use real Core handles; no fake opaque pointers.
- Validate compiler-derived C/Ada record layouts.
- Linux ASan/UBSan instruments the real Calib3D shim.
- Run Alire build/test/example commands serially; shared generated directories
  are not safe for concurrent mutation.
- Do not convert the presence of tests into a claim that they passed.
