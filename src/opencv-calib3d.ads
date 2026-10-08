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

   --  p2^T * F * p1 = 0, p=(x,y,1). Arbitrary nonzero common scale;
   --  neither F(2,2)=1 nor a sign convention is promised. F alone does not
   --  recover metric scale, camera position, or 3-D terrain location.
   type Fundamental_Matrix is
     array (Natural range 0 .. 2, Natural range 0 .. 2) of OpenCV.Float64_Value;
   type Epipolar_Error_Result (Defined : Boolean := False) is record
      case Defined is
         when True => Maximum_Error_Pixels : OpenCV.Float64_Value;
         when False => null;
      end case;
   end record;
   --  max(abs(p1^T F^T p2)/hypot((F^T p2).xy),
   --      abs(p2^T F p1)/hypot((F p1).xy)), NOT Sampson distance.
   --  Nonfinite inputs raise OpenCV_Error. Zero line normals or nonfinite
   --  intermediates/distances return Defined=False; no epsilon or clamping.
   function Maximum_Epipolar_Error
     (Matrix : Fundamental_Matrix; First_Point, Second_Point : Image_Point)
      return Epipolar_Error_Result;
   type Fundamental_RANSAC_Options is record
      Epipolar_Threshold_Pixels : OpenCV.Float64_Value := 3.0;
      Confidence               : OpenCV.Float64_Value := 0.99;
   end record;
   type Fundamental_Estimate is limited private;
   --  Equal counts >=15 fitting INT32: 8..14 would silently run LMeDS.
   --  Common legacy native RANSAC ceiling is fixed at 1000 iterations.
   --  Native estimation converts coordinates to Float32 internally.
   --  Public inliers use FINAL F and ORIGINAL Float64 observations, defined
   --  Maximum_Epipolar_Error <= threshold. Found requires >=7 final inliers.
   --  Threshold: finite positive, float(double(threshold*threshold)) finite
   --  positive. Confidence: DBL_EPSILON <= c <= 1-DBL_EPSILON (inclusive),
   --  not merely (0,1): avoid silent native substitution with 0.99.
   function Estimate_Fundamental_RANSAC
     (First_Points, Second_Points : Image_Point_Array;
      Options : Fundamental_RANSAC_Options := (others => <>))
      return Fundamental_Estimate;
   function Found (Estimate : Fundamental_Estimate) return Boolean;
   --  Raises OpenCV_Error when Found=False; no-model inliers are 1 .. 0.
   function Fundamental (Estimate : Fundamental_Estimate) return Fundamental_Matrix;
   function Inlier_Count (Estimate : Fundamental_Estimate) return Natural;
   function Inlier (Estimate : Fundamental_Estimate; Index : Positive) return Positive;
   --  One-based correspondence positions, valid/unique/strictly ascending.
   function Inliers (Estimate : Fundamental_Estimate) return Inlier_Index_Array;

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

   --  Calibrated normalized coordinates ONLY. Do not pass distorted pixels:
   --  use Undistort_To_Normalized separately for each camera first.
   --  x2^T * E * x1 = 0, x=(normalized_x,normalized_y,1).
   --  Arbitrary nonzero scale; no E(2,2)=1 or coefficient sign convention.
   type Essential_Matrix is
     array (Natural range 0 .. 2, Natural range 0 .. 2) of OpenCV.Float64_Value;
   type Normalized_Sampson_Error_Result (Defined : Boolean := False) is record
      case Defined is
         when True => Error : OpenCV.Float64_Value;
         when False => null;
      end case;
   end record;
   --  abs(x2^T E x1) / norm((E*x1).xy, (E^T*x2).xy), dimensionless.
   --  This is NOT Maximum_Epipolar_Error. Nonfinite inputs raise OpenCV_Error;
   --  zero denominator/nonrepresentable arithmetic returns Defined=False.
   --  No epsilon, clamping, NaN publication, or escaping Constraint_Error.
   function Normalized_Sampson_Error
     (Matrix : Essential_Matrix; First_Point, Second_Point : Normalized_Image_Point)
      return Normalized_Sampson_Error_Result;
   type Essential_RANSAC_Options is record
      Normalized_Epipolar_Threshold : OpenCV.Float64_Value := 1.0E-3;
      Confidence : OpenCV.Float64_Value := 0.999;
   end record;
   --  X_second = R * X_first + lambda * t_hat, UNKNOWN lambda > 0.
   --  recoverPose does NOT recover metric baseline magnitude or navigation
   --  position. Translation_Direction is unit translation-TERM direction,
   --  NOT the second camera position or camera-center motion direction.
   type Relative_Camera_Pose is record
      Rotation_First_To_Second : Rotation_Matrix;
      Translation_Direction : Camera_Direction;
   end record;
   --  normalize(-R^T*t_hat): second-camera-center direction in first frame.
   type Triangulation_Status is
     (Usable, At_Infinity, Unrepresentable_Point, Non_Positive_Depth,
      Undefined_Reprojection);
   type Triangulated_Point
     (Status : Triangulation_Status := Unrepresentable_Point) is record
      case Status is
         when Usable =>
            Position_In_First_Camera : Object_Point;
            First_Depth, Second_Depth : OpenCV.Float64_Value;
            First_Normalized_Error, Second_Normalized_Error : OpenCV.Float64_Value;
         when others => null;
      end case;
   end record;
   type Triangulated_Point_Array is
     array (Positive range <>) of Triangulated_Point;
   --  P1=[I|0], P2=[R|normalize(t)]. First-camera coordinates in unit-baseline
   --  units, NOT meters: multiply by the physical baseline for metric scale.
   --  Equal finite counts fitting INT32; one pair permitted; empty returns 1..0
   --  after pose validation. SO(3): absolute tolerance 1E-9 on R^T R and det=1.
   --  No rotation repair. Nonzero finite translation normalized preserving sign.
   --  W=0 is infinity; small nonzero W is divided without an epsilon cutoff.
   --  Both depths must be positive. Errors are dimensionless, NOT pixels;
   --  no residual rejection threshold. Usable is not an uncertainty certificate:
   --  tiny parallax can produce enormous poorly conditioned finite points.
   --  One result per pair in iteration order, output lower bound always one.
   function Triangulate_Normalized
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose) return Triangulated_Point_Array;

   function Second_Camera_Center_Direction_In_First
     (Pose : Relative_Camera_Pose) return Camera_Direction;
   type Essential_Estimate is limited private;
   --  Equal counts >=6 fitting INT32. OpenCV can solve five points, but five
   --  bypass subset consensus; this RANSAC API deliberately requires six.
   --  Fixed native maximum 1000; adaptive termination may stop earlier.
   --  Observations/five-point algebra are Float64; native error and squared
   --  threshold comparison are Float32. Public inliers instead classify final
   --  E against ORIGINAL Float64 observations using defined Sampson <= threshold.
   --  Threshold finite positive; double square and Float32 conversion finite
   --  positive. Confidence finite and strictly (0,1), without epsilon limits.
   --  Found requires >=5 final Essential inliers. Pose recovery uses exactly
   --  those inliers, explicit finite Double'Last distance cutoff (not hidden 50),
   --  and requires >=5 cheirality survivors. Infinite/nonrepresentable internal
   --  triangulations can still fail. Insufficient pose support retains E.
   function Estimate_Essential_RANSAC
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Options : Essential_RANSAC_Options := (others => <>)) return Essential_Estimate;
   function Found (Estimate : Essential_Estimate) return Boolean;
   --  Raises OpenCV_Error if Found=False.
   function Essential (Estimate : Essential_Estimate) return Essential_Matrix;
   function Inlier_Count (Estimate : Essential_Estimate) return Natural;
   function Inlier (Estimate : Essential_Estimate; Index : Positive) return Positive;
   --  One-based correspondence positions, valid/unique/strictly ascending.
   function Inliers (Estimate : Essential_Estimate) return Inlier_Index_Array;
   function Pose_Recovered (Estimate : Essential_Estimate) return Boolean;
   --  Raises OpenCV_Error if Pose_Recovered=False.
   function Recovered_Pose (Estimate : Essential_Estimate) return Relative_Camera_Pose;
   function Pose_Inlier_Count (Estimate : Essential_Estimate) return Natural;
   function Pose_Inlier (Estimate : Essential_Estimate; Index : Positive) return Positive;
   --  Cheirality-qualified subset of Essential inliers, likewise ascending.
   function Pose_Inliers (Estimate : Essential_Estimate) return Inlier_Index_Array;
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
   type Essential_Estimate is new Ada.Finalization.Limited_Controlled with record
      Has_Model, Has_Pose : Boolean := False;
      Value : Essential_Matrix := [others => [others => 0.0]];
      Pose_Value : Relative_Camera_Pose :=
        (Rotation_First_To_Second => [others => [others => 0.0]],
         Translation_Direction => [others => 0.0]);
      Data, Pose_Data : Inlier_Buffer := null;
   end record;
   overriding procedure Finalize (Self : in out Essential_Estimate);
   type Fundamental_Estimate is new Ada.Finalization.Limited_Controlled with record
      Has_Model : Boolean := False;
      Value     : Fundamental_Matrix := [others => [others => 0.0]];
      Data      : Inlier_Buffer := null;
   end record;
   overriding procedure Finalize (Self : in out Fundamental_Estimate);
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
