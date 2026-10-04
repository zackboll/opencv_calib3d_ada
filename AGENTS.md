# Agent entry point

Read the files in `.clinerules/` before changing this repository.

This is a handwritten thick Ada binding. Production bindings are not generated.
`opencv_core` remains the sole owner of application `Mat` wrappers. Calib3D may
borrow native Mat headers only through `OpenCV.Core.Module_Interop` callbacks.
Never expose C++/STL ABI types or public raw pointers.

The public namespace is `OpenCV.Calib3D` on every supported OpenCV version even
though the native backend is `calib3d` on OpenCV 4.x and `geometry` on OpenCV 5.0.
The existing Ada `OpenCV.Geometry` namespace belongs to another crate.

Start with `docs/tasks/001-validate-bootstrap.md`. The ZIP has static validation
only; no native Ada/OpenCV build result is claimed. Do not merge, tag, release,
or force-push unless the user explicitly asks.
