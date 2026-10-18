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
   --  Guarded native cv::correctMatches policy. EVERY field is caller-selected;
   --  there is deliberately no default. With s1>=s2>=s3 the measured singular
   --  values of F divided by its maximum |coefficient|:
   --    s2/s1 >= Minimum_Second_Singular_Ratio   (0 < value <= 1)
   --    s3/s1 <= Maximum_Third_Singular_Ratio    (0 <= value < 1, < minimum)
   --  Input line quality (dimensionless, 0..1, scale invariant) is
   --    min(|(F p1).xy|/(||F||_F ||p1||), |(F^T p2).xy|/(||F||_F ||p2||)),
   --  p=(x,y,1), and must be >= Minimum_Epipolar_Line_Ratio (0..1).
   --  Corrected Maximum_Epipolar_Error must be <= Maximum_Corrected_Error_Pixels.
   --  Displacement D must satisfy D <= Bound*(1+Relative)+Absolute, Relative in
   --  0..1, Absolute >= 0. These are numerical screens; no combination is a
   --  guarantee of global optimality.
   type Epipolar_Correction_Options is record
      Minimum_Second_Singular_Ratio       : OpenCV.Float64_Value;
      Maximum_Third_Singular_Ratio        : OpenCV.Float64_Value;
      Minimum_Epipolar_Line_Ratio         : OpenCV.Float64_Value;
      Maximum_Corrected_Error_Pixels      : OpenCV.Float64_Value;
      Absolute_Displacement_Excess_Pixels : OpenCV.Float64_Value;
      Relative_Displacement_Excess        : OpenCV.Float64_Value;
   end record;

   --  Measured geometric diagnostics, NOT an optimality certificate.
   --  Errors are Maximum_Epipolar_Error values against the supplied F.
   --  Total_Displacement_Pixels is the four-dimensional Euclidean correction
   --  distance; One_Sided_Bound_Pixels = min(d1,d2) of the ORIGINAL pair.
   type Epipolar_Correction_Diagnostic is record
      Original_Error_Pixels     : OpenCV.Float64_Value := 0.0;
      Corrected_Error_Pixels    : OpenCV.Float64_Value := 0.0;
      Total_Displacement_Pixels : OpenCV.Float64_Value := 0.0;
      One_Sided_Bound_Pixels    : OpenCV.Float64_Value := 0.0;
   end record;
   type Epipolar_Correction_Diagnostic_Array is
     array (Positive range <>) of Epipolar_Correction_Diagnostic;

   type Corrected_Epipolar_Matches (Count : Natural) is record
      First_Points  : Image_Point_Array (1 .. Count);
      Second_Points : Image_Point_Array (1 .. Count);
      Diagnostics   : Epipolar_Correction_Diagnostic_Array (1 .. Count);
   end record;

   --  Native OpenCV cv::correctMatches (Hartley-Sturm) behind a guarded,
   --  all-or-error contract. Does not estimate F, run RANSAC, undistort, or
   --  select correspondences: F and the points must share one pixel domain.
   --  Sources are paired by position (lower bounds may differ); every result
   --  array is one-based. Invalid input/options, a policy rejection, nonfinite
   --  or inconsistent native output, or any failed check raises OpenCV_Error and
   --  publishes nothing. Success means the explicitly checked conditions held;
   --  it does NOT certify the global optimum, 3-D accuracy, correspondence
   --  identity, pose, metric scale or navigation quality.
   function Correct_Epipolar_Matches
     (Matrix        : Fundamental_Matrix;
      First_Points  : Image_Point_Array;
      Second_Points : Image_Point_Array;
      Options       : Epipolar_Correction_Options)
      return Corrected_Epipolar_Matches;

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
   --  N . (X - Anchor) = 0 in the caller's Cartesian world frame.
   --  Normal and ray directions need not be unit vectors, but must be finite
   --  and nonzero. Range and residual use the caller's world length units.
   type World_Plane is record
      Anchor : Object_Point;
      Normal : World_Direction;
   end record;
   type World_Plane_Intersection_Options is record
      Minimum_Absolute_Incidence_Cosine : OpenCV.Float64_Value;
      Minimum_Forward_Range : OpenCV.Float64_Value;
      Maximum_Forward_Range : OpenCV.Float64_Value;
      Maximum_Absolute_Plane_Residual : OpenCV.Float64_Value;
   end record;
   type World_Plane_Intersection_Disposition is
     (Forward_Intersection_Accepted, Parallel_Ray, Coplanar_Ray,
      Grazing_Incidence_Rejected, Intersection_At_Origin,
      Intersection_Behind_Ray, Below_Minimum_Forward_Range,
      Above_Maximum_Forward_Range, Plane_Residual_Rejected,
      Numerically_Unavailable);
   type World_Plane_Intersection is private;
   type World_Plane_Intersection_Array is
     array (Positive range <>) of World_Plane_Intersection;
   --  Validate ALL inputs before intersection, including empty requests.
   --  One-based, owned results preserve input order, regardless of bounds.
   --  Precedence: arithmetic availability at each stage; exact parallel or
   --  coplanar; signed distance zero/negative; incidence; min/max range;
   --  represented residual; acceptance. No hidden epsilons or defaults.
   function Intersect_World_Rays_With_Plane
     (Rays : World_Ray_Array; Plane : World_Plane;
      Options : World_Plane_Intersection_Options)
      return World_Plane_Intersection_Array;
   function Disposition (Result : World_Plane_Intersection)
      return World_Plane_Intersection_Disposition;
   function Incidence_Evaluated (Result : World_Plane_Intersection) return Boolean;
   function Absolute_Incidence_Cosine (Result : World_Plane_Intersection)
      return OpenCV.Float64_Value;
   function Has_Signed_Ray_Distance (Result : World_Plane_Intersection) return Boolean;
   function Signed_Ray_Distance (Result : World_Plane_Intersection)
      return OpenCV.Float64_Value;
   function Plane_Residual_Evaluated (Result : World_Plane_Intersection) return Boolean;
   function Absolute_Plane_Residual (Result : World_Plane_Intersection)
      return OpenCV.Float64_Value;
   function Accepted (Result : World_Plane_Intersection) return Boolean;
   --  Every unavailable numeric accessor raises OpenCV_Error. Position is
   --  available ONLY after full acceptance, never merely after construction.
   function Accepted_World_Position (Result : World_Plane_Intersection)
      return Object_Point;
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

   type Stereo_Parallax is record
      Forward_Ray_Angle_Radians : OpenCV.Float64_Value;
      Acute_Line_Angle_Radians : OpenCV.Float64_Value;
   end record;
   type Stereo_Parallax_Array is array (Positive range <>) of Stereo_Parallax;
   --  Pure Ada bearing diagnostics: compare b1 with normalize(R^T*b2).
   --  Forward angle is 0..pi; acute line angle is 0..pi/2, including
   --  antiparallel lines as weak geometry. Translation never shifts bearings.
   --  Equal finite observations; same pose validation as triangulation.
   --  One-based iteration order; empty returns 1..0 after pose validation.
   function Measure_Stereo_Parallax
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose) return Stereo_Parallax_Array;

   type Triangulation_Quality_Options is record
      Minimum_Acute_Parallax_Radians : OpenCV.Float64_Value := 0.0;
      Maximum_Normalized_Reprojection_Error : OpenCV.Float64_Value :=
        OpenCV.Float64_Value'Last;
   end record;
   type Triangulation_Quality is record
      Original_Status : Triangulation_Status;
      Parallax : Stereo_Parallax;
      Passes_Parallax_Limit : Boolean;
      Passes_Reprojection_Limit : Boolean;
      Accepted : Boolean;
   end record;
   type Triangulation_Quality_Array is
     array (Positive range <>) of Triangulation_Quality;
   --  Caller policy only, not covariance/confidence or a depth-error bound.
   --  Inclusive thresholds, finite minimum in 0..pi/2 and maximum >=0.
   --  Zero minimum disables parallax filtering. Defaults accept all Usable
   --  results. Equal counts, paired by iteration order, one-based output.
   --  Allegedly Usable entries must have finite position, positive finite
   --  depths and nonnegative finite errors; otherwise raise OpenCV_Error.
   --  Never modifies/recreates Points or changes triangulation status.
   function Assess_Triangulation
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose;
      Points : Triangulated_Point_Array;
      Options : Triangulation_Quality_Options := (others => <>))
      return Triangulation_Quality_Array;

   --  Every threshold is caller-selected; no empirical acceptance defaults.
   type Metric_Triangulation_Options is record
      Minimum_Acute_Parallax_Radians : OpenCV.Float64_Value;
      Maximum_Normalized_Reprojection_Error : OpenCV.Float64_Value;
      Maximum_First_Pixel_Reprojection_Error : OpenCV.Float64_Value;
      Maximum_Second_Pixel_Reprojection_Error : OpenCV.Float64_Value;
   end record;
   type Metric_Triangulation_Disposition is
     (Metric_Accepted, Metric_Policy_Rejected, No_Physical_Baseline,
      Source_At_Infinity, Source_Unrepresentable_Point,
      Source_Non_Positive_Depth, Source_Undefined_Reprojection,
      Metric_Conversion_Unavailable);
   type Metric_Source_Diagnostics (Available : Boolean := False) is record
      case Available is
         when True =>
            Point : Triangulated_Point;
            Quality : Triangulation_Quality;
         when False => null;
      end case;
   end record;
   type Metric_Point_Geometry (Available : Boolean := False) is record
      case Available is
         when True =>
            Position_In_First_Camera, World_Position : Object_Point;
            First_Depth, Second_Depth : OpenCV.Float64_Value;
         when False => null;
      end case;
   end record;
   type Metric_Pixel_Diagnostics (Available : Boolean := False) is record
      case Available is
         when True =>
            First_Error_Pixels, Second_Error_Pixels : OpenCV.Float64_Value;
            Passes_First_Limit, Passes_Second_Limit : Boolean;
         when False => null;
      end case;
   end record;
   type Metric_Triangulated_Point is record
      Disposition : Metric_Triangulation_Disposition := No_Physical_Baseline;
      Source : Metric_Source_Diagnostics;
      Geometry : Metric_Point_Geometry;
      Pixels : Metric_Pixel_Diagnostics;
   end record;
   type Metric_Triangulated_Point_Array is
     array (Positive range <>) of Metric_Triangulated_Point;

   type Point_Refinement_Outcome is
     (Skipped_Unusable, Skipped_Low_Parallax, No_Improving_Step,
      Improved, Numerically_Unavailable);
   type Point_Refinement_Result is record
      Outcome : Point_Refinement_Outcome;
      Point : Triangulated_Point;
      Accepted_Steps : Natural := 0;
      Residuals_Evaluated : Boolean := False;
      Initial_Normalized_RMS : OpenCV.Float64_Value := 0.0;
      Final_Normalized_RMS : OpenCV.Float64_Value := 0.0;
   end record;
   type Point_Refinement_Result_Array is
     array (Positive range <>) of Point_Refinement_Result;
   type Triangulation_Refinement_Options is record
      Maximum_Iterations : Positive := 20;
      Minimum_Acute_Parallax_Radians : OpenCV.Float64_Value := 0.0;
   end record;

   --  Required, finite nonnegative pixel allowances; equality passes.
   type Metric_Refinement_Acceptance_Options is record
      Maximum_First_Pixel_Worsening : OpenCV.Float64_Value;
      Maximum_Second_Pixel_Worsening : OpenCV.Float64_Value;
   end record;
   type Metric_Refinement_Disposition is
     (Initial_No_Baseline, Initial_Source_Unusable,
      Initial_Geometry_Unavailable, Initial_Pixels_Unavailable,
      Refinement_Low_Parallax, Refinement_No_Improving_Step,
      Refinement_Numerically_Unavailable, Refined_Conversion_Unavailable,
      Refined_Pixels_Unavailable, Refined_Policy_Rejected,
      Refined_Metric_Accepted);
   type Optional_Point_Refinement (Available : Boolean := False) is record
      case Available is
         when True => Value : Point_Refinement_Result;
         when False => null;
      end case;
   end record;
   type Metric_Point_Refinement_Result is private;
   type Metric_Point_Refinement_Result_Array is
     array (Positive range <>) of Metric_Point_Refinement_Result;
   function Disposition (Result : Metric_Point_Refinement_Result)
     return Metric_Refinement_Disposition;
   function Original_Reconstruction (Result : Metric_Point_Refinement_Result)
     return Metric_Triangulated_Point;
   --  Absent if initial geometry/pixel diagnostics were not refinable.
   function Normalized_Refinement (Result : Metric_Point_Refinement_Result)
     return Optional_Point_Refinement;
   --  Candidate diagnostics are NOT an accepted world position. Available only
   --  following Improved, and independently guarded at each conversion stage.
   function Refined_Diagnostics (Result : Metric_Point_Refinement_Result)
     return Metric_Triangulated_Point;
   function Accepted (Result : Metric_Point_Refinement_Result) return Boolean;
   --  Raises OpenCV_Error unless the complete final policy accepted this point.
   function Accepted_World_Position (Result : Metric_Point_Refinement_Result)
     return Object_Point;
   --  Pure Ada, fixed-pose point-only normalized reprojection refinement.
   --  Iterations 1..100; finite minimum angle 0..pi/2 (zero disables screening).
   --  Equal counts paired by iteration order; one-based output, including empty.
   --  Only Improved publishes new coordinates/diagnostics. Other outcomes retain
   --  the exact input point. RMS fields are meaningful only when evaluated.
   --  Strictly decreasing accepted residual norm, positive depths in both cameras.
   --  First-camera unit-baseline coordinates, not metric accuracy/covariance.
   function Refine_Triangulated_Points
     (First_Points, Second_Points : Normalized_Image_Point_Array;
      Pose : Relative_Camera_Pose;
      Initial : Triangulated_Point_Array;
      Options : Triangulation_Refinement_Options := (others => <>))
      return Point_Refinement_Result_Array;

   --  normalize(-R^T*t_hat): second-camera-center direction in first frame.
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

   --  Native cv::composeRT. First maps frame A to B (X_B = R1 X_A + t1), Second
   --  maps B to C (X_C = R2 X_B + t2); the result maps A directly to C. "Apply
   --  First, then Second." The caller supplies consecutive compatible frames and
   --  consistent Cartesian units. The result Rodrigues vector is native output and
   --  is not canonicalized: compare Rotation_Matrix_Of, not vector components.
   --  Raises OpenCV_Error for nonfinite or unrepresentable inputs/outputs.
   function Compose_Poses
     (First : World_To_Camera_Pose; Second : World_To_Camera_Pose)
      return World_To_Camera_Pose;

   --  Invert a rigid source-to-destination transform stored in
   --  World_To_Camera_Pose. The record is reused even when the frames are
   --  reversed: if X_destination = R * X_source + t, the result maps
   --  destination coordinates back to source coordinates:
   --      R_inverse = R^T
   --      t_inverse = -R^T * t
   --  Rotation is native Rodrigues output and is not canonicalized; compare
   --  Rotation_Matrix_Of, not vector components. Negating the Rodrigues
   --  vector is an equivalent rotation only when that vector is a validated
   --  representation of the same physical rotation. Raises OpenCV_Error for
   --  nonfinite input or an inverse that cannot be represented safely.
   function Invert_Pose (Pose : World_To_Camera_Pose) return World_To_Camera_Pose;

   --  Relative rigid transform from First camera coordinates to Second camera
   --  coordinates. Both inputs are world-to-camera poses in the SAME caller
   --  Cartesian world frame and the SAME length units:
   --      X_first  = R1 * X_world + t1
   --      X_second = R2 * X_world + t2
   --      X_second = R_relative * X_first + t_relative
   --  Equivalent composition is Invert_Pose (First), then Second. The result
   --  translation is expressed in the SECOND camera frame. It is not the
   --  second camera's position and is not the world-frame displacement.
   --  This operation does not estimate a pose and does not decide whether the
   --  inputs are accurate or share a real-world reference. Raises OpenCV_Error
   --  for nonfinite input or an unrepresentable result.
   function Camera_To_Camera_Pose
     (First, Second : World_To_Camera_Pose) return World_To_Camera_Pose;

   --  Owned diagnostics for two absolute poses. Baseline_Direction is defined
   --  only when Baseline_Distance is not exactly zero. A zero baseline is a
   --  legitimate pure rotation about a common camera center; no epsilon treats
   --  a nonzero length as zero. All vectors are caller world-frame Cartesian
   --  units except the relative translation, which is in the second camera
   --  frame, and Second_Center_In_First, which is in the first camera frame.
   type Camera_Motion_Result (Baseline_Direction_Defined : Boolean := False) is record
      Relative_Pose             : World_To_Camera_Pose;
      First_Center_World        : Object_Point;
      Second_Center_World       : Object_Point;
      Second_Center_In_First    : Object_Point;
      Displacement_World        : Object_Point;
      Baseline_Distance         : OpenCV.Float64_Value;
      case Baseline_Direction_Defined is
         when True =>
            Baseline_Direction_World : Object_Point;
         when False =>
            null;
      end case;
   end record;

   --  Distorted pixels -> separate calibration -> unit-baseline native
   --  triangulation -> baseline scaling -> caller-world-frame coordinates.
   --  Both poses MUST share one Cartesian world frame and length units.
   --  Exact zero baseline publishes no source/geometry/pixel diagnostics.
   --  Other results preserve source status. Only Available fields may be read.
   --  Finite nonnegative inclusive thresholds; minimum angle <= pi/2.
   --  Equal finite counts fitting INT32; positional pairing, one-based output.
   --  Ada metric arithmetic unavailability is per-point; native failures raise
   --  OpenCV_Error without partial publication. No confidence/accuracy bound.
   function Triangulate_World_Points
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Options : Metric_Triangulation_Options)
      return Metric_Triangulated_Point_Array;

   --  One initial batch triangulation, fixed-pose unit-baseline refinement,
   --  shared metric conversion and fresh two-camera distorted pixel checks.
   --  Preserves every original result, including initial policy rejections.
   --  An initially rejected but geometrically usable point may become accepted.
   --  Final acceptance requires Improved, all four metric quality thresholds,
   --  and both pixel-worsening allowances. No hidden epsilon or accuracy claim.
   --  Pose, baseline, intrinsics and distortion are never optimized.
   function Refine_Metric_World_Points
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Metric_Options : Metric_Triangulation_Options;
      Refinement_Options : Triangulation_Refinement_Options;
      Acceptance : Metric_Refinement_Acceptance_Options)
      return Metric_Point_Refinement_Result_Array;

   --  Measure the rigid motion from First to Second. Centers are
   --  C = -R^T * t. Displacement_World is C2 - C1. Baseline_Distance is the
   --  scaled Euclidean norm of that displacement, in the caller's world
   --  length units. Relative scale exists only because both absolute poses
   --  use that same Cartesian frame. No Earth, geographic, or navigation
   --  uncertainty is implied. Raises OpenCV_Error instead of returning NaN,
   --  infinity, a saturated coordinate, or a fabricated zero displacement.
   function Measure_Camera_Motion
     (First, Second : World_To_Camera_Pose) return Camera_Motion_Result;

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

   --  Reuses World_Bearing_Rays, including validation and native-free empty
   --  behavior. Native errors propagate atomically, not as ray dispositions.
   function Unproject_Pixels_Onto_World_Plane
     (Pixels : Image_Point_Array; Intrinsics : Camera_Intrinsics;
      Distortion : Distortion_Coefficients; Pose : World_To_Camera_Pose;
      Plane : World_Plane; Options : World_Plane_Intersection_Options)
      return World_Plane_Intersection_Array;

   type Planar_Motion_Hypothesis is record
      Rotation_First_To_Second : Rotation_Matrix;
      Translation_Over_Plane_Distance : Translation_Vector;
      Plane_Normal_In_First : Camera_Direction;
      Pure_Rotation : Boolean;
   end record;
   type Planar_Motion_Hypothesis_Array is
     array (Positive range <>) of Planar_Motion_Hypothesis;
   --  Undistorted pinhole homography; same K in both views. Identity K for
   --  normalized coordinates. K^-1 H K ~ R+(t/d)n^T, first to second camera.
   --  All native hypotheses, no physical selection or visibility filtering.
   --  t/d preserves magnitude, NOT metric translation. Simultaneous negation
   --  of t/d and n preserves geometry. Native near-rotation tolerance: 0.001.
   function Decompose_Calibrated_Homography
     (Matrix : Homography_Matrix; Intrinsics : Camera_Intrinsics)
      return Planar_Motion_Hypothesis_Array;

   type Planar_Visibility_Disposition is
     (Passes_Visibility, Rejected_By_Visibility,
      Not_Applicable_Pure_Rotation, No_Selected_References);
   type Planar_Hypothesis_Visibility is record
      Disposition : Planar_Visibility_Disposition;
   end record;
   type Planar_Hypothesis_Visibility_Array is
     array (Positive range <>) of Planar_Hypothesis_Visibility;
   --  Source-defined strict signs: n.p1 > 0 and (R*n).p2 > 0 for ALL
   --  selected references. Not full cheirality or unique motion selection.
   --  Public Float64 observations are checked then rounded to native Float32;
   --  signs near zero may differ. Overflow/nonzero-to-zero underflow raises
   --  OpenCV_Error. All supplied observations must be finite, even unselected.
   --  Result bounds 1 .. Hypotheses'Length, preserving candidate order.
   function Assess_Planar_Visibility
     (Hypotheses : Planar_Motion_Hypothesis_Array;
      First_Points, Second_Points : Normalized_Image_Point_Array)
      return Planar_Hypothesis_Visibility_Array;
   --  One-based correspondence POSITIONS, unique and strictly ascending.
   --  Omitted selection means all; explicitly empty means none. Pure/near
   --  rotation is not applicable, including when no references are selected.
   function Assess_Planar_Visibility
     (Hypotheses : Planar_Motion_Hypothesis_Array;
      First_Points, Second_Points : Normalized_Image_Point_Array;
      Selected_Indices : Inlier_Index_Array)
      return Planar_Hypothesis_Visibility_Array;
   --  One-based hypothesis positions, not correspondence positions.
   function Visibility_Passing_Hypothesis_Indices
     (Results : Planar_Hypothesis_Visibility_Array) return Inlier_Index_Array;

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


    --  Exactly three pixel correspondences, fixed SOLVEPNP_AP3P. Every native
    --  hypothesis is retained; nothing selects a unique physical pose. Native
    --  order is version-specific. A real zero-solution result is 1 .. 0.
    type AP3P_Pose_Hypothesis is record
       Pose : World_To_Camera_Pose;
    end record;
    type AP3P_Pose_Hypothesis_Array is
      array (Positive range <>) of AP3P_Pose_Hypothesis;

    function Solve_AP3P_Hypotheses
      (Object_Points : Object_Point_Array;
       Image_Points  : Image_Point_Array;
       Intrinsics    : Camera_Intrinsics;
       Distortion    : Distortion_Coefficients := No_Distortion)
       return AP3P_Pose_Hypothesis_Array;

   --  Independent-reference diagnostics for existing AP3P hypotheses.
   --  Native Project_Points performs perspective projection and the five
   --  Brown/OpenCV distortion coefficients. Ada classifies camera depth and
   --  aggregates results. No candidate is selected, sorted, refined, or replaced.
   --
   --  X_camera = R * X_world + t, with R from Rotation_Matrix_Of. A reference
   --  is eligible for projection only when that camera Z is finite and Z > 0.
   --  Zero and negative Z are Non_Positive_Depth and are not projected.
   --  Numerically_Unavailable covers specified Ada-side Float64 representability
   --  failures (camera-frame products/sums, or a residual after a successful
   --  native projection). It is not a substitute for a native Project_Points
   --  failure: those raise OpenCV_Error and do not return a partial matrix.
   --  Stored zeros on a nonprojected disposition are undefined placeholders,
   --  not a perfect reprojection.
   --
   --  Output is 1 .. Hypotheses'Length by 1 .. Reference_World'Length, paired
   --  by position regardless of input bounds. Empty pairs do not call
   --  projectPoints. Malformed inputs raise OpenCV_Error before evaluation.
   type AP3P_Reference_Disposition is
     (Projected_Positive_Depth, Non_Positive_Depth, Numerically_Unavailable);

   type AP3P_Reference_Result is record
      Disposition : AP3P_Reference_Disposition := Numerically_Unavailable;
      Camera_Depth : OpenCV.Float64_Value := 0.0;
      Error_Pixels : OpenCV.Float64_Value := 0.0;
   end record;

   type AP3P_Reference_Result_Matrix is
     array (Positive range <>, Positive range <>) of AP3P_Reference_Result;

   function Evaluate_AP3P_References
     (Hypotheses       : AP3P_Pose_Hypothesis_Array;
      Reference_World  : Object_Point_Array;
      Reference_Pixels : Image_Point_Array;
      Intrinsics       : Camera_Intrinsics;
      Distortion       : Distortion_Coefficients := No_Distortion)
      return AP3P_Reference_Result_Matrix;

   --  One one-based summary per hypothesis row, original order preserved.
   --  RMS and maximum use Summarize_Reprojection on projected errors only.
   --  Entire_Set_Evaluable is True only when every supplied reference for that
   --  row was projected. An empty row is not a zero-error success.
   type AP3P_Hypothesis_Reference_Summary is record
      Total_References : Natural := 0;
      Projected_References : Natural := 0;
      Non_Positive_Depth_References : Natural := 0;
      Numerically_Unavailable_References : Natural := 0;
      Entire_Set_Evaluable : Boolean := False;
      Reprojection : Reprojection_Summary;
   end record;

   type AP3P_Hypothesis_Reference_Summary_Array is
     array (Positive range <>) of AP3P_Hypothesis_Reference_Summary;

   function Summarize_AP3P_References
     (Results : AP3P_Reference_Result_Matrix)
      return AP3P_Hypothesis_Reference_Summary_Array;

   --  Caller-controlled qualification of already evaluated AP3P hypotheses.
   --  Every field is required; there is no library default. Thresholds are
   --  inclusive. Separation is strict: second_best - best must be greater
   --  than Minimum_RMS_Separation_Pixels. All three values must be finite
   --  and nonnegative. A unique result is unique only for this reference set
   --  and this policy. It is not proof of globally correct localization.
   type AP3P_Selection_Options is record
      Maximum_RMS_Error_Pixels      : OpenCV.Float64_Value;
      Maximum_Point_Error_Pixels    : OpenCV.Float64_Value;
      Minimum_RMS_Separation_Pixels : OpenCV.Float64_Value;
   end record;

   type AP3P_Selection_Disposition is
     (No_Hypotheses,
      No_Independent_References,
      No_Eligible_Hypotheses,
      Ambiguous_Qualified_Hypotheses,
      Unique_Selected_Hypothesis);

   --  One diagnostic per original hypothesis, original order preserved.
   --  Eligible is True only when the whole reference set was projected at
   --  positive depth and both RMS and maximum error are within the caller's
   --  inclusive thresholds. RMS_Error_Pixels and Maximum_Error_Pixels are the
   --  whole-set scores when Eligible, otherwise 0.0 placeholders. A stored
   --  zero is not a perfect score. Rank is 0 when not eligible; eligible
   --  candidates are ranked 1 .. N by increasing RMS, original order breaking
   --  exact RMS ties. Rank is diagnostic and is not itself a selection.
   type AP3P_Candidate_Qualification is record
      Eligible             : Boolean := False;
      Rank                 : Natural := 0;
      RMS_Error_Pixels     : OpenCV.Float64_Value := 0.0;
      Maximum_Error_Pixels : OpenCV.Float64_Value := 0.0;
   end record;

   type AP3P_Candidate_Qualification_Array is
     array (Positive range <>) of AP3P_Candidate_Qualification;

   --  Index-only qualification of a caller-supplied reference matrix.
   --  Candidate_Index is a position in that matrix's original hypothesis order.
   --  This result does not own a pose and does not prove that any external
   --  hypothesis array is the array that produced the matrix. Selected_Pose
   --  only checks length and position, then returns that external value.
   type AP3P_Selection_Result (Count : Natural) is record
      Disposition       : AP3P_Selection_Disposition := No_Hypotheses;
      Candidate_Index   : Natural := 0;
      Separation_Pixels : OpenCV.Float64_Value := 0.0;
      Candidates        : AP3P_Candidate_Qualification_Array (1 .. Count);
   end record;

   function Selected (Result : AP3P_Selection_Result) return Boolean;

   --  Raises OpenCV_Error unless Selected is True.
   function Selected_Hypothesis_Index (Result : AP3P_Selection_Result) return Positive;

   --  Raises OpenCV_Error unless Selected is True and Index is the selected
   --  position in the caller-supplied Hypotheses. This does not verify that
   --  Hypotheses is the array that was evaluated. It does not compare
   --  Rodrigues components.
   function Selected_Pose
     (Result : AP3P_Selection_Result;
      Hypotheses : AP3P_Pose_Hypothesis_Array;
      Index : Positive) return World_To_Camera_Pose;

   --  Qualify existing hypotheses from an already computed reference matrix.
   --  Does not solve AP3P, project points, sort the caller's hypotheses, or
   --  refine a pose. A native Project_Points failure cannot occur here: it
   --  must already have propagated from Evaluate_AP3P_References.
   --  Malformed summaries and invalid options raise OpenCV_Error and publish
   --  nothing. Empty hypothesis or reference dimensions are classified, not
   --  raised, after validation.
   function Select_AP3P_Hypothesis
     (Results : AP3P_Reference_Result_Matrix;
      Options : AP3P_Selection_Options) return AP3P_Selection_Result;

   --  End-to-end result. The owned pose is the hypothesis selected from the
   --  single Solve_AP3P_Hypotheses call inside Select_AP3P_Pose. Callers
   --  cannot construct this type. Candidate diagnostics use the same policy
   --  as AP3P_Selection_Result, but Selected_Pose here needs no external array.
   type AP3P_Pose_Selection (Count : Natural) is private;

   function Disposition (Result : AP3P_Pose_Selection) return AP3P_Selection_Disposition;
   function Candidate_Index (Result : AP3P_Pose_Selection) return Natural;
   function Separation_Pixels (Result : AP3P_Pose_Selection) return OpenCV.Float64_Value;
   function Candidates (Result : AP3P_Pose_Selection) return AP3P_Candidate_Qualification_Array;
   function Selected (Result : AP3P_Pose_Selection) return Boolean;

   --  Raises OpenCV_Error unless this result uniquely selected and owns the
   --  pose from that same solve.
   function Selected_Pose (Result : AP3P_Pose_Selection) return World_To_Camera_Pose;

   --  End-to-end: exactly three seed correspondences, at least one additional
   --  reference, one native AP3P solve, reference evaluation, then the caller's
   --  policy. The returned object owns the selected pose from that solve.
   --  Seed and reference arrays are paired by position. A native zero-solution
   --  result returns No_Hypotheses. OpenCV_Error from projection or from the
   --  solver propagates; no partial selection is returned.
   --  Reference world points must not repeat a seed world point or each
   --  other. Equality is exact; no near-duplicate tolerance is applied.
   function Select_AP3P_Pose
     (Seed_World       : Object_Point_Array;
      Seed_Pixels      : Image_Point_Array;
      Reference_World  : Object_Point_Array;
      Reference_Pixels : Image_Point_Array;
      Intrinsics       : Camera_Intrinsics;
      Distortion       : Distortion_Coefficients := No_Distortion;
      Options          : AP3P_Selection_Options) return AP3P_Pose_Selection;

   --  Caller-controlled acceptance of one native iterative refinement.
   --  Every field is required; there is no library default. All three
   --  values must be finite and nonnegative. RMS and maximum are inclusive.
   --  Worsening is the refined held-out RMS minus the original held-out RMS;
   --  a nonpositive difference always satisfies a nonnegative allowance.
   --  Acceptance uses only the held-out set, never the fitting observations.
   type AP3P_Refinement_Acceptance_Options is record
      Maximum_Holdout_RMS_Pixels           : OpenCV.Float64_Value;
      Maximum_Holdout_Point_Error_Pixels   : OpenCV.Float64_Value;
      Maximum_Holdout_RMS_Worsening_Pixels : OpenCV.Float64_Value;
   end record;

   --  Why refinement was not published as the camera pose. Selection_Only
   --  means Task 016 did not uniquely select, so refinement was not called.
   --  The other values require a unique selection. Native_Refinement_False
   --  is an expected native false return, not an ABI error.
   type AP3P_Refinement_Disposition is
     (Selection_Only,
      Initial_Holdout_Unusable,
      Native_Refinement_False,
      Refined_Holdout_Unusable,
      Rejected_By_Policy,
      Accepted);

   --  End-to-end result. The initial pose is the owned pose from one
   --  Select_AP3P_Pose call. The refined pose is published only when every
   --  acceptance check succeeds. Callers cannot construct this type.
   --  Held-out summaries exist only when that pose was fully evaluated at
   --  positive depth. A stored zero is not a perfect score.
   type AP3P_Refined_Pose (Count : Natural) is private;

   function Selection_Disposition
     (Result : AP3P_Refined_Pose) return AP3P_Selection_Disposition;
   function Refinement_Disposition
     (Result : AP3P_Refined_Pose) return AP3P_Refinement_Disposition;
   function Refinement_Attempted (Result : AP3P_Refined_Pose) return Boolean;
   function Native_Refinement_Succeeded (Result : AP3P_Refined_Pose) return Boolean;
   function Accepted (Result : AP3P_Refined_Pose) return Boolean;

   --  Raises OpenCV_Error unless selection uniquely owned a pose.
   function Initial_Pose (Result : AP3P_Refined_Pose) return World_To_Camera_Pose;

   --  Raises OpenCV_Error unless refinement was accepted. An unaccepted
   --  native result is never exposed as the camera pose.
   function Accepted_Pose (Result : AP3P_Refined_Pose) return World_To_Camera_Pose;

   function Initial_Holdout_Valid (Result : AP3P_Refined_Pose) return Boolean;
   function Refined_Holdout_Valid (Result : AP3P_Refined_Pose) return Boolean;

   --  Raise OpenCV_Error unless the corresponding holdout is valid.
   function Initial_Holdout (Result : AP3P_Refined_Pose) return Reprojection_Summary;
   function Refined_Holdout (Result : AP3P_Refined_Pose) return Reprojection_Summary;

   --  Exactly three seed pairs, zero or more selection references, and one
   --  or more held-out references. An empty selection set preserves the
   --  Task 016 No_Independent_References disposition and does not refine.
   --  The three sets must be disjoint under
   --  exact world-point equality, including duplicates inside a set.
   --  Selection references may be used for refinement; held-out pairs are
   --  never supplied to AP3P, candidate selection, or iterative refinement.
   --  Disjoint inputs are not statistical independence when measurements
   --  share systematic error. Arrays are paired by position regardless of
   --  lower bounds. Native refinement is existing SOLVEPNP_ITERATIVE with
   --  the selected pose as the guess. It is not a new optimizer.
   --  X_camera = R * X_world + t. Translation remains t in the caller's
   --  Cartesian frame, not a camera position and not an Earth or DTED frame.
   --  Pixel residuals are not position uncertainty, covariance, or metric
   --  accuracy. OpenCV_Error propagates and publishes no partial result.
   function Select_Refine_AP3P_Pose
     (Seed_World        : Object_Point_Array;
      Seed_Pixels       : Image_Point_Array;
      Selection_World   : Object_Point_Array;
      Selection_Pixels  : Image_Point_Array;
      Holdout_World     : Object_Point_Array;
      Holdout_Pixels    : Image_Point_Array;
      Intrinsics        : Camera_Intrinsics;
      Distortion        : Distortion_Coefficients := No_Distortion;
      Selection         : AP3P_Selection_Options;
      Acceptance        : AP3P_Refinement_Acceptance_Options)
      return AP3P_Refined_Pose;

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

   --  Task 021: robust EPNP consensus followed by ONE inlier-only iterative
   --  refinement. Held-out landmarks never fit the pose. Every policy field is
   --  mandatory; limits are finite, nonnegative, inclusive, with no epsilon.
   type Relocalization_Verification_Options is record
      Minimum_Consensus_Inliers : Positive;
      Maximum_Final_Inlier_RMS_Pixels : OpenCV.Float64_Value;
      Maximum_Holdout_RMS_Pixels : OpenCV.Float64_Value;
      Maximum_Holdout_Point_Error_Pixels : OpenCV.Float64_Value;
      Maximum_Holdout_RMS_Worsening_Pixels : OpenCV.Float64_Value;
   end record;

   type Camera_Relocalization_Disposition is
     (Insufficient_Fitting_Landmarks, No_Native_RANSAC_Pose,
      Insufficient_Consensus_Inliers, Initial_Holdout_Unusable,
      Native_Refinement_False, Refined_Holdout_Unusable,
      Final_Inlier_Verification_Unusable, Rejected_By_Policy, Accepted);

   --  Owned values, independent of the finalized Pose_Estimate. Count is the
   --  native consensus count, NOT the number of post-refinement inliers.
   type Camera_Relocalization_Result (Count : Natural) is private;
   function Relocalize_Camera
     (Fitting_World : Object_Point_Array; Fitting_Pixels : Image_Point_Array;
      Holdout_World : Object_Point_Array; Holdout_Pixels : Image_Point_Array;
      Intrinsics : Camera_Intrinsics; Distortion : Distortion_Coefficients;
      RANSAC : RANSAC_Options; Verification : Relocalization_Verification_Options)
      return Camera_Relocalization_Result;
   --  Requires matched finite positional pairs, unique exact world points in
   --  each set, no exact world overlap, >=1 holdout. Valid fitting counts <6
   --  return Insufficient_Fitting_Landmarks without native entry (even 4/5).
   --  Both sets share the caller's Cartesian world frame and length units.
   function Disposition (Result : Camera_Relocalization_Result)
      return Camera_Relocalization_Disposition;
   function Found (Result : Camera_Relocalization_Result) return Boolean;
   function Initial_Pose (Result : Camera_Relocalization_Result) return World_To_Camera_Pose;
   function Inlier_Count (Result : Camera_Relocalization_Result) return Natural;
   --  Sorted one-based ORIGINAL fitting positions, unchanged by refinement.
   function Inliers (Result : Camera_Relocalization_Result) return Inlier_Index_Array;
   function Refinement_Attempted (Result : Camera_Relocalization_Result) return Boolean;
   function Native_Refinement_Succeeded (Result : Camera_Relocalization_Result) return Boolean;
   function Initial_Holdout_Valid (Result : Camera_Relocalization_Result) return Boolean;
   function Refined_Holdout_Valid (Result : Camera_Relocalization_Result) return Boolean;
   function Final_Inliers_Valid (Result : Camera_Relocalization_Result) return Boolean;
   --  Missing summaries/errors/poses raise OpenCV_Error, never placeholder scores.
   function Initial_Holdout (Result : Camera_Relocalization_Result) return Reprojection_Summary;
   function Refined_Holdout (Result : Camera_Relocalization_Result) return Reprojection_Summary;
   function Final_Inlier_Summary (Result : Camera_Relocalization_Result) return Reprojection_Summary;
   function Final_Inlier_Errors (Result : Camera_Relocalization_Result) return Reprojection_Error_Array;
   function Accepted (Result : Camera_Relocalization_Result) return Boolean;
   function Accepted_Pose (Result : Camera_Relocalization_Result) return World_To_Camera_Pose;
   --  C_world=-R^T*t, NOT -t. Available only after full acceptance.
   function Camera_Center (Result : Camera_Relocalization_Result) return Object_Point;

   --  Optional Task 020 adapter. Acceptance is a reprojection policy, not proof
   --  of physical landmark accuracy. Only Accepted_World_Position is consumed.
   type Accepted_Landmark_Set (Count : Natural) is private;
   function Accepted_Landmark_Observations
     (Landmarks : Metric_Point_Refinement_Result_Array; Pixels : Image_Point_Array)
      return Accepted_Landmark_Set;
   function World_Points (Landmarks : Accepted_Landmark_Set) return Object_Point_Array;
   function Image_Points (Landmarks : Accepted_Landmark_Set) return Image_Point_Array;
   function Original_Positions (Landmarks : Accepted_Landmark_Set) return Inlier_Index_Array;

   --  Task 022: known poses, never estimated or changed. Translation below is
   --  the first-to-second transform term in the SECOND camera frame, not a
   --  world camera-center displacement. E=[normalize(t_relative)]_x*R_relative,
   --  x2^T E x1=0. Exact zero physical baseline has no Essential coefficients.
   type Known_Pose_Essential_Result (Defined : Boolean := False) is record
      Relative_Pose : World_To_Camera_Pose;
      Baseline_Distance : OpenCV.Float64_Value := 0.0;
      case Defined is
         when True =>
            Rotation_First_To_Second : Rotation_Matrix;
            Translation_Direction : Camera_Direction;
            Matrix : Essential_Matrix;
         when False => null;
      end case;
   end record;
   function Essential_From_Camera_Poses
     (First, Second : World_To_Camera_Pose) return Known_Pose_Essential_Result;

   --  Mandatory finite nonnegative inclusive limits, dimensionless NOT pixels.
   type Epipolar_Verification_Options is record
      Minimum_Correspondences : Positive;
      Maximum_RMS_Normalized_Sampson : OpenCV.Float64_Value;
      Maximum_Point_Normalized_Sampson : OpenCV.Float64_Value;
   end record;
   type Epipolar_Verification_Disposition is
     (Insufficient_Correspondences, No_Physical_Baseline,
      Undefined_Correspondence_Errors, Rejected_By_Policy, Accepted);
   type Epipolar_Correspondence_Result (Defined : Boolean := False) is record
      case Defined is
         when True =>
            Error : OpenCV.Float64_Value;
            Passes_Point_Limit : Boolean;
         when False => null;
      end case;
   end record;
   type Epipolar_Correspondence_Result_Array is
     array (Positive range <>) of Epipolar_Correspondence_Result;
   type Epipolar_Verification_Summary is record
      RMS_Normalized_Sampson : OpenCV.Float64_Value;
      Maximum_Normalized_Sampson : OpenCV.Float64_Value;
   end record;
   --  Count is the ORIGINAL pair count. All diagnostics are owned, one-based,
   --  ordered by position regardless of input lower bounds. No partial-set RMS.
   type Known_Pose_Epipolar_Result (Count : Natural) is private;
   function Verify_Known_Pose_Epipolar
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose : World_To_Camera_Pose;
      Options : Epipolar_Verification_Options) return Known_Pose_Epipolar_Result;
   --  Both relocalizations MUST be Accepted; uses only Accepted_Pose, no fit.
   --  Cross-image matches are separately supplied; their independence from pose
   --  fitting/calibration is the caller's responsibility, not an API guarantee.
   function Verify_Known_Pose_Epipolar
     (First_Pixels, Second_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion : Distortion_Coefficients;
      First, Second : Camera_Relocalization_Result;
      Options : Epipolar_Verification_Options) return Known_Pose_Epipolar_Result;
   function Disposition (Result : Known_Pose_Epipolar_Result)
      return Epipolar_Verification_Disposition;
   function Accepted (Result : Known_Pose_Epipolar_Result) return Boolean;
   function Geometry_Evaluated (Result : Known_Pose_Epipolar_Result) return Boolean;
   --  Raises OpenCV_Error when geometry was not evaluated (insufficient count).
   function Geometry (Result : Known_Pose_Epipolar_Result) return Known_Pose_Essential_Result;
   function Correspondence_Errors (Result : Known_Pose_Epipolar_Result)
      return Epipolar_Correspondence_Result_Array;
   function Entire_Set_Evaluable (Result : Known_Pose_Epipolar_Result) return Boolean;
   --  Raises OpenCV_Error unless EVERY supplied error was defined.
   function Summary (Result : Known_Pose_Epipolar_Result) return Epipolar_Verification_Summary;
   --  All input fields are validated even for empty/insufficient requests;
   --  valid insufficient requests return without native entry. Construction or
   --  native failures raise OpenCV_Error atomically. No baseline epsilon.
   --  Magnitude-only epipolar errors cannot verify absolute position, physical
   --  baseline magnitude, E sign, positive depth, reconstruction or match truth.

   --  Task 023: first/second observations fit a metric candidate; third pixels
   --  verify it only, never select it or participate in reconstruction.
   type Three_View_Source_Mode is
     (Initial_Only, Prefer_Refined_With_Original_Fallback,
      Require_Accepted_Refinement);
   type Three_View_Landmark_Options is record
      Maximum_Normalized_Sampson_Error : OpenCV.Float64_Value;
      Metric : Metric_Triangulation_Options;
      Refinement : Triangulation_Refinement_Options;
      Refinement_Acceptance : Metric_Refinement_Acceptance_Options;
      Source_Mode : Three_View_Source_Mode;
      Maximum_Third_Pixel_Error : OpenCV.Float64_Value;
   end record;
   type Three_View_Landmark_Disposition is
     (No_Stereo_Baseline, Epipolar_Undefined, Epipolar_Rejected,
      Stereo_Source_Unusable, Stereo_Quality_Rejected,
      Refinement_Required_Unavailable, Third_Non_Positive_Depth,
      Third_Numerically_Unavailable, Third_Reprojection_Rejected,
      Three_View_Accepted);
   type Three_View_Epipolar_Disposition is
     (Epipolar_Not_Evaluated, Epipolar_Metric_Undefined,
      Epipolar_Limit_Rejected, Epipolar_Passed);
   type Three_View_Candidate_Source is
     (No_Selected_Candidate, Initial_Metric_Candidate, Refined_Metric_Candidate);
   type Three_View_Landmark_Result is private;
   type Three_View_Landmark_Result_Array is
     array (Positive range <>) of Three_View_Landmark_Result;
   function Qualify_Three_View_Landmarks
     (First_Pixels, Second_Pixels, Third_Pixels : Image_Point_Array;
      First_Intrinsics, Second_Intrinsics, Third_Intrinsics : Camera_Intrinsics;
      First_Distortion, Second_Distortion, Third_Distortion : Distortion_Coefficients;
      First_Pose, Second_Pose, Third_Pose : World_To_Camera_Pose;
      Options : Three_View_Landmark_Options) return Three_View_Landmark_Result_Array;
   function Original_Position (Result : Three_View_Landmark_Result) return Positive;
   function Disposition (Result : Three_View_Landmark_Result)
      return Three_View_Landmark_Disposition;
   function Epipolar_Disposition (Result : Three_View_Landmark_Result)
      return Three_View_Epipolar_Disposition;
   function Epipolar_Error (Result : Three_View_Landmark_Result)
      return Epipolar_Correspondence_Result;
   function Has_Initial_Reconstruction (Result : Three_View_Landmark_Result) return Boolean;
   function Original_Reconstruction (Result : Three_View_Landmark_Result)
      return Metric_Triangulated_Point;
   function Refinement_Requested (Result : Three_View_Landmark_Result) return Boolean;
   function Has_Refinement_Diagnostics (Result : Three_View_Landmark_Result) return Boolean;
   function Refinement_Diagnostics (Result : Three_View_Landmark_Result)
      return Metric_Point_Refinement_Result;
   function Candidate_Source (Result : Three_View_Landmark_Result)
      return Three_View_Candidate_Source;
   --  Fitting-stage accepted candidate, NOT final three-view acceptance.
   --  Raises OpenCV_Error when Candidate_Source=No_Selected_Candidate.
   function Selected_World_Position (Result : Three_View_Landmark_Result) return Object_Point;
   function Third_View_Evaluated (Result : Three_View_Landmark_Result) return Boolean;
   --  Disposition after evaluation; finite depth unless numerically unavailable;
   --  pixel error only after positive-depth projection.
   --  All unavailable guarded accessors raise OpenCV_Error, never return zeros.
   function Third_View_Disposition (Result : Three_View_Landmark_Result)
      return AP3P_Reference_Disposition;
   function Third_Camera_Depth (Result : Three_View_Landmark_Result) return OpenCV.Float64_Value;
   function Third_Pixel_Error (Result : Three_View_Landmark_Result) return OpenCV.Float64_Value;
   function Accepted (Result : Three_View_Landmark_Result) return Boolean;
   function Accepted_World_Position (Result : Three_View_Landmark_Result) return Object_Point;
   --  Pixels may be separately supplied fourth-image observations. No fourth
   --  pose estimation/verification occurs. Original positions survive slicing.
   function Three_View_Landmark_Observations
     (Landmarks : Three_View_Landmark_Result_Array; Pixels : Image_Point_Array)
      return Accepted_Landmark_Set;

private
   type World_Plane_Intersection is record
      State : World_Plane_Intersection_Disposition := Numerically_Unavailable;
      Has_Incidence, Has_Distance, Has_Residual : Boolean := False;
      Incidence, Distance, Residual : OpenCV.Float64_Value := 0.0;
      Position : Object_Point := [others => 0.0];
   end record;
   type Three_View_Landmark_Result is record
      Position : Positive := 1;
      State : Three_View_Landmark_Disposition := Epipolar_Undefined;
      Screening : Three_View_Epipolar_Disposition := Epipolar_Not_Evaluated;
      Sampson : Epipolar_Correspondence_Result;
      Has_Initial, Wants_Refinement, Has_Refinement : Boolean := False;
      Initial : Metric_Triangulated_Point;
      Refined : Metric_Point_Refinement_Result;
      Selected : Three_View_Candidate_Source := No_Selected_Candidate;
      Candidate : Object_Point := [others => 0.0];
      Has_Third : Boolean := False;
      Third : AP3P_Reference_Result;
   end record;
   type Known_Pose_Epipolar_Result (Count : Natural) is record
      State : Epipolar_Verification_Disposition := Insufficient_Correspondences;
      Has_Geometry, Has_Summary : Boolean := False;
      Geometry_Value : Known_Pose_Essential_Result;
      Errors : Epipolar_Correspondence_Result_Array (1 .. Count);
      Summary_Value : Epipolar_Verification_Summary := (0.0, 0.0);
   end record;
   type Accepted_Landmark_Set (Count : Natural) is record
      World : Object_Point_Array (1 .. Count);
      Pixels : Image_Point_Array (1 .. Count);
      Positions : Inlier_Index_Array (1 .. Count) := [others => 1];
   end record;
   type Camera_Relocalization_Result (Count : Natural) is record
      State : Camera_Relocalization_Disposition := Insufficient_Fitting_Landmarks;
      Has_Initial, Attempted, Native_Succeeded : Boolean := False;
      Initial_Valid, Refined_Valid, Inliers_Valid : Boolean := False;
      Initial, Final_Pose : World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Center : Object_Point := [others => 0.0];
      Initial_Summary, Refined_Summary, Inlier_Summary : Reprojection_Summary;
      Positions : Inlier_Index_Array (1 .. Count) := [others => 1];
      Errors : Reprojection_Error_Array (1 .. Count) := [others => 0.0];
   end record;
   type Metric_Point_Refinement_Result is record
      State : Metric_Refinement_Disposition := Initial_No_Baseline;
      Original, Candidate : Metric_Triangulated_Point;
      Optimization : Optional_Point_Refinement;
   end record;
   --  Has_Pose is true only when Select_AP3P_Pose copied the selected
   --  hypothesis from its own solver result. Disposition alone is not enough.
   type AP3P_Pose_Selection (Count : Natural) is record
      Disposition       : AP3P_Selection_Disposition := No_Hypotheses;
      Candidate_Index   : Natural := 0;
      Separation_Pixels : OpenCV.Float64_Value := 0.0;
      Candidates        : AP3P_Candidate_Qualification_Array (1 .. Count);
      Has_Pose          : Boolean := False;
      Pose              : World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
   end record;

   --  Has_Initial is true only when selection uniquely owned a pose.
   --  Has_Accepted is true only after native success and held-out acceptance.
   --  Holdout flags are independent: an unusable refined evaluation does not
   --  erase a valid initial summary, and neither flag publishes a pose.
   type AP3P_Refined_Pose (Count : Natural) is record
      Selection_State    : AP3P_Selection_Disposition := No_Hypotheses;
      Refinement_State   : AP3P_Refinement_Disposition := Selection_Only;
      Attempted          : Boolean := False;
      Native_Succeeded   : Boolean := False;
      Has_Initial        : Boolean := False;
      Has_Accepted       : Boolean := False;
      Initial_Valid      : Boolean := False;
      Refined_Valid      : Boolean := False;
      Initial            : World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Accepted_Value     : World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Initial_Summary    : Reprojection_Summary := (others => <>);
      Refined_Summary    : Reprojection_Summary := (others => <>);
      Candidates         : AP3P_Candidate_Qualification_Array (1 .. Count);
   end record;

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
