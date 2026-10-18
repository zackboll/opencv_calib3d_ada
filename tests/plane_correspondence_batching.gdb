# Run with /home/zboll/git/opencv/calib3d/tests/bin/run_tests.
# Symbol breakpoints avoid source line drift. Focus on the three clean points
# in PC_Brown_Tilted; the next registered fixture is PC_Mixed.
set pagination off
set confirm off
set breakpoint pending on
python
import gdb
active = False
counts = {}
verified = False
class Count(gdb.Breakpoint):
    def stop(self):
        if active:
            counts[self.location] = counts.get(self.location, 0) + 1
        return False
class Start(gdb.Breakpoint):
    def stop(self):
        global active
        active = True
        return False
class Verify(gdb.Breakpoint):
    def stop(self):
        global active, verified
        active = False
        for symbol in symbols:
            count = counts.get(symbol, 0)
            gdb.write('%s: %d calls (expected 1)\n' % (symbol, count))
            if count != 1:
                gdb.execute('quit 1')
        verified = True
        return False
def finish(event):
    if not verified or getattr(event, 'exit_code', 1) != 0:
        gdb.execute('quit 1')
gdb.events.exited.connect(finish)
symbols = ['opencv__calib3d__unproject_pixels_onto_world_plane',
           'opencv__calib3d__evaluate_ap3p_references',
           'opencv_calib3d_project_points']
for symbol in symbols:
    Count(symbol)
Start('calib3d_tests__pc_brown_tilted')
Verify('calib3d_tests__pc_mixed')
end
run