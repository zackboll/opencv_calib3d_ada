with Ada.Finalization;
with OpenCV.Core.Float64_Vec2;
with OpenCV.Core.Float64_Vec3;

package OpenCV.Calib3D is
   --  OpenCV 4.x uses native calib3d; OpenCV 5.0 moved this functionality to
   --  the native geometry module. The Ada namespace remains version-neutral.

   subtype Image_Point is OpenCV.Core.Float64_Vec2.Vector;
   subtype Object_Point is OpenCV.Core.Float64_Vec3.Vector;
   subtype Rotation_Vector is OpenCV.Core.Float64_Vec3.Vector;
   subtype Translation_Vector is OpenCV.Core.Float64_Vec3.Vector;

   type Image_Point_Array is array (Positive range <>) of Image_Point;
   type Object_Point_Array is array (Positive range <>) of Object_Point;
   type Inlier_Index_Array is array (Positive range <>) of Positive;

   --  [x',y',1]^T ~ H * [x,y,1]^T. H has arbitrary nonzero common scale;
   --  h22=1 is NOT an invariant. A single H models planes/projective image
   --  registration, not general 3-D terrain pose (use PnP for 3-D points).
   type Homography_Matrix is
     array (Natural range 0 .. 2, Natural range 0 .. 2) of OpenCV.Float64_Value;
   type Homography_Point_Result (Finite : Boolean := False) is record
      case Finite is
         when True => Point : Image_Point;
         when False => null;
      end case;
   end record;
   --  w=h20*x+h21*y+h22; x'=(h00*x+h01*y+h02)/w, similarly y'.
   --  Nonfinite inputs raise OpenCV_Error. Zero w or nonfinite arithmetic
   --  returns Finite=False; no epsilon rejection or coordinate clamping.
   function Map_With_Homography
     (Matrix : Homography_Matrix; Point : Image_Point)
      return Homography_Point_Result;

   type Homography_RANSAC_Options is record
      Maximum_Iterations            : Positive := 2_000;
      Reprojection_Threshold_Pixels : OpenCV.Float64_Value := 3.0;
      Confidence                    : OpenCV.Float64_Value := 0.995;
   end record;
   type Homography_Estimate is limited private;
   --  Equal counts >=5 deliberately exclude OpenCV's n=4 direct solve, which
   --  bypasses robust consensus. Native estimation converts points to Float32.
   --  Public inliers are independently classified with FINAL H and ORIGINAL
   --  Float64 points: finite forward mapping and hypot error <= threshold in
   --  destination-image pixels. Native masks are not the public contract.
   --  Found requires >=4 final inliers; otherwise Inliers is empty (1 .. 0).
   --  Options: iterations <=INT32_MAX, finite positive Float32-representable
   --  threshold, finite confidence strictly between zero and one.
   function Estimate_Homography_RANSAC
     (Source_Points, Destination_Points : Image_Point_Array;
      Options : Homography_RANSAC_Options := (others => <>))
      return Homography_Estimate;
   function Found (Estimate : Homography_Estimate) return Boolean;
   --  Raises OpenCV_Error when Found=False.
   function Homography (Estimate : Homography_Estimate) return Homography_Matrix;
   function Inlier_Count (Estimate : Homography_Estimate) return Natural;
   function Inlier (Estimate : Homography_Estimate; Index : Positive) return Positive;
   --  One-based correspondence positions, valid/unique/strictly ascending.
   function Inliers (Estimate : Homography_Estimate) return Inlier_Index_Array;

   --  Dimensionless normalized pinhole coordinates, NOT pixels: (x,y)
   --  corresponds to the camera-frame projective direction (x,y,1).
   type Normalized_Image_Point is new OpenCV.Core.Float64_Vec2.Vector;
   type Normalized_Image_Point_Array is
     array (Positive range <>) of Normalized_Image_Point;
   type Camera_Direction is new OpenCV.Core.Float64_Vec3.Vector;
   type World_Direction is new OpenCV.Core.Float64_Vec3.Vector;
   type Camera_Direction_Array is array (Positive range <>) of Camera_Direction;
   type World_Ray is record
      Origin    : Object_Point;
      Direction : World_Direction;
   end record;
   type World_Ray_Array is array (Positive range <>) of World_Ray;
   type Rotation_Matrix is array (Natural range 0 .. 2, Natural range 0 .. 2)
     of OpenCV.Float64_Value;
   type Reprojection_Error_Array is
     array (Positive range <>) of OpenCV.Float64_Value;

   type Reprojection_Summary is record
      Count                : Natural := 0;
      RMS_Error_Pixels     : OpenCV.Float64_Value := 0.0;
      Maximum_Error_Pixels : OpenCV.Float64_Value := 0.0;
   end record;

   type Camera_Intrinsics is record
      Focal_X  : OpenCV.Float64_Value;
      Focal_Y  : OpenCV.Float64_Value;
      Center_X : OpenCV.Float64_Value;
      Center_Y : OpenCV.Float64_Value;
   end record;

   type Distortion_Coefficients is record
      K1 : OpenCV.Float64_Value := 0.0;
      K2 : OpenCV.Float64_Value := 0.0;
      P1 : OpenCV.Float64_Value := 0.0;
      P2 : OpenCV.Float64_Value := 0.0;
      K3 : OpenCV.Float64_Value := 0.0;
   end record;

   No_Distortion : constant Distortion_Coefficients := (others => 0.0);

   --  OpenCV pose convention:
   --       X_camera = R * X_world + t
   --  Rotation is a Rodrigues rotation vector. Translation is t, NOT the
   --  camera position. Camera_Center returns C_world = -R^T * t.
   type World_To_Camera_Pose is record
      Rotation    : Rotation_Vector := [others => 0.0];
      Translation : Translation_Vector := [others => 0.0];
   end record;

   type RANSAC_Options is record
      Maximum_Iterations        : Positive := 100;
      Reprojection_Error_Pixels : OpenCV.Float64_Value := 8.0;
      Confidence                : OpenCV.Float64_Value := 0.99;
   end record;

   type Pose_Estimate is limited private;

   function Found (Estimate : Pose_Estimate) return Boolean;
   function Pose (Estimate : Pose_Estimate) return World_To_Camera_Pose;
   function Inlier_Count (Estimate : Pose_Estimate) return Natural;
   function Inlier (Estimate : Pose_Estimate; Index : Positive) return Positive;
   function Inliers (Estimate : Pose_Estimate) return Inlier_Index_Array;

   function Camera_Center (Pose : World_To_Camera_Pose) return Object_Point;

   --  R = native Rodrigues(Pose.Rotation), X_camera = R * X_world + t.
   --  All six pose components must be finite, even for direction operations.
   function Rotation_Matrix_Of (Pose : World_To_Camera_Pose) return Rotation_Matrix;
   function World_To_Camera_Point
     (Pose : World_To_Camera_Pose; Point : Object_Point) return Object_Point;
   function Camera_To_World_Point
     (Pose : World_To_Camera_Pose; Point : Object_Point) return Object_Point;
   --  Translation has no effect; magnitude is preserved (rounding aside).
   --  Arbitrary finite direction inputs are NOT silently normalized.
   function World_To_Camera_Direction
     (Pose : World_To_Camera_Pose; Direction : World_Direction) return Camera_Direction;
   function Camera_To_World_Direction
     (Pose : World_To_Camera_Pose; Direction : Camera_Direction) return World_Direction;

   --  Distorted pixels -> normalized standard-model coordinates. No R/P.
   --  Fixed native COUNT|EPS policy: 20 iterations, 1e-12 pixel epsilon.
   --  One-based output preserves length/order; empty returns 1 .. 0 AFTER
   --  validating parameters. No native undistortion for valid empty input.
   function Undistort_To_Normalized
     (Points : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients := No_Distortion)
      return Normalized_Image_Point_Array;
   --  Unit finite directions (Float64 norm tolerance 1e-12), camera Z > 0.
   function Camera_Bearing_Rays
     (Points : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients := No_Distortion)
      return Camera_Direction_Array;
   --  Origin = Camera_Center(Pose); direction = normalized R^T * bearing.
   --  X_world(s) = Origin + s * Direction, s > 0 is forward along the ray.
   --  Origin/s use caller-defined Cartesian units, not necessarily meters.
   --  No terrain, Earth model, or geographic frame interpretation is implied.
   function World_Bearing_Rays
     (Points : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients; Pose : World_To_Camera_Pose)
      return World_Ray_Array;

   function Project_Points
     (Points     : Object_Point_Array;
      Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients;
      Pose       : World_To_Camera_Pose) return Image_Point_Array;

   function Solve_PnP_RANSAC
     (Object_Points : Object_Point_Array;
      Image_Points  : Image_Point_Array;
      Intrinsics    : Camera_Intrinsics;
      Distortion    : Distortion_Coefficients := No_Distortion;
      Options       : RANSAC_Options := (others => <>)) return Pose_Estimate;

   --  Euclidean pixel errors, one-based in correspondence order. Equal counts
   --  are required; empty/empty returns 1 .. 0. These diagnostics are not pose
   --  uncertainty and do not establish globally correct localization.
   function Reprojection_Errors
     (Object_Points : Object_Point_Array;
      Image_Points  : Image_Point_Array;
      Intrinsics    : Camera_Intrinsics;
      Distortion    : Distortion_Coefficients;
      Pose          : World_To_Camera_Pose) return Reprojection_Error_Array;

   --  Finite, nonnegative errors only. Empty input returns all-zero summary.
   function Summarize_Reprojection
     (Errors : Reprojection_Error_Array) return Reprojection_Summary;

   --  solvePnP with SOLVEPNP_ITERATIVE and useExtrinsicGuess=true, NOT
   --  solvePnPRefineLM. Pose is the initial world -> camera estimate.
   --  Equal counts >=4 are a conservative binding contract: native OpenCV
   --  also permits three points with an extrinsic guess. No planarity policy.
   --  False leaves Pose exactly unchanged; errors never publish partial Pose.
   procedure Refine_Pose_Iterative
     (Object_Points : Object_Point_Array;
      Image_Points  : Image_Point_Array;
      Intrinsics    : Camera_Intrinsics;
      Distortion    : Distortion_Coefficients := No_Distortion;
      Pose          : in out World_To_Camera_Pose;
      Refined       : out Boolean);

private
   type Inlier_Buffer is access Inlier_Index_Array;
   type Homography_Estimate is new Ada.Finalization.Limited_Controlled with record
      Has_Model : Boolean := False;
      Value     : Homography_Matrix := [others => [others => 0.0]];
      Data      : Inlier_Buffer := null;
   end record;
   overriding procedure Finalize (Self : in out Homography_Estimate);
   type Pose_Estimate is new Ada.Finalization.Limited_Controlled with record
      Has_Pose : Boolean := False;
      Value    : World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Data     : Inlier_Buffer := null;
   end record;
   overriding procedure Finalize (Self : in out Pose_Estimate);
end OpenCV.Calib3D;
