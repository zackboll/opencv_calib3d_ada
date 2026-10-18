# Optional Linux debugger qualification against tests/bin/run_tests.
# Source breakpoints intentionally pin the focused regression fixtures.
set language c
set pagination off
set confirm off
set breakpoint pending on
set $active = 0
set $calls = 0
break opencv_calib3d_project_points
commands
silent
if $active
set $calls = $calls + 1
end
continue
end
break calib3d_tests__reference_batch_mapping
commands
silent
set $active = 1
set $calls = 0
continue
end
break calib3d_tests.adb:8874
commands
silent
printf "Four hypotheses with three positive references each: %d native calls (expected 4)\n", $calls
if $calls != 4
quit 1
end
continue
end
break calib3d_tests.adb:8879
commands
silent
printf "Four batches plus singleton: %d native calls (expected 5)\n", $calls
if $calls != 5
quit 1
end
set $active = 0
continue
end
break calib3d_tests__reference_batch_unavailable
commands
silent
set $active = 1
set $calls = 0
continue
end
break calib3d_tests.adb:8897
commands
silent
printf "Two mixed arithmetic batches: %d native calls (expected 2)\n", $calls
if $calls != 2
quit 1
end
set $active = 0
continue
end
break calib3d_tests__reference_batch_atomic
commands
silent
set $active = 1
set $calls = 0
continue
end
break calib3d_tests.adb:8930
commands
silent
printf "Two healthy hypotheses plus failing batch; empty/behind skip: %d native calls (expected 3)\n", $calls
if $calls != 3
quit 1
end
set $active = 0
continue
end
run