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
   type Pose_Estimate is new Ada.Finalization.Limited_Controlled with record
      Has_Pose : Boolean := False;
      Value    : World_To_Camera_Pose :=
        (Rotation => [others => 0.0], Translation => [others => 0.0]);
      Data     : Inlier_Buffer := null;
   end record;
   overriding procedure Finalize (Self : in out Pose_Estimate);
end OpenCV.Calib3D;
