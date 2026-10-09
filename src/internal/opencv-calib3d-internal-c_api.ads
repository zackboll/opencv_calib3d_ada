with Interfaces;
with Interfaces.C;
with Interfaces.C.Strings;
with OpenCV.Core.Module_Interop;
with System;

package OpenCV.Calib3D.Internal.C_API is
   subtype Status is Interfaces.Integer_32;
   Success : constant Status := 0;

   type C_Essential is record
      E00, E01, E02, E10, E11, E12, E20, E21, E22 : Interfaces.C.double;
   end record with Convention => C;
   type C_Relative_Pose is record
      R00, R01, R02, R10, R11, R12, R20, R21, R22, TX, TY, TZ : Interfaces.C.double;
   end record with Convention => C;
   type C_Triangulated_Point is record
      Point_Status : Interfaces.Integer_32;
      X, Y, Z, Depth_First, Depth_Second, Error_First, Error_Second : Interfaces.C.double;
   end record with Convention => C;
   function Triangulate_Normalized
     (First, Second : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Pose : access constant C_Relative_Pose; Result : access System.Address) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_triangulate_normalized";
   function Triangulation_Result_Count
     (Handle : System.Address; Count : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_triangulation_result_count";
   function Triangulation_Result_Point
     (Handle : System.Address; Index : Interfaces.Integer_32;
      Point : access C_Triangulated_Point) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_triangulation_result_point";
   procedure Triangulation_Result_Destroy (Handle : System.Address)
     with Import, Convention => C, External_Name => "opencv_calib3d_triangulation_result_destroy";
   type C_Essential_Options is record
      Normalized_Epipolar_Threshold, Confidence : Interfaces.C.double;
   end record with Convention => C;
   function Validate_Essential_Options (Options : access constant C_Essential_Options) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_validate_essential_options";
   function Normalized_Sampson_Error
     (Matrix : access constant C_Essential; X1, Y1, X2, Y2 : Interfaces.C.double;
      Defined : access Interfaces.Unsigned_8; Error : access Interfaces.C.double) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_normalized_sampson_error";
   function Find_Essential_RANSAC
     (First, Second : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Options : access constant C_Essential_Options; Result : access System.Address) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_find_essential_ransac";
   function Essential_Result_Found
     (Handle : System.Address; Value : access Interfaces.Unsigned_8) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_found";
   function Essential_Result_Matrix
     (Handle : System.Address; Value : access C_Essential) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_matrix";
   function Essential_Result_Inlier_Count
     (Handle : System.Address; Count : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_inlier_count";
   function Essential_Result_Inlier
     (Handle : System.Address; Index : Interfaces.Integer_32;
      Correspondence_Index : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_inlier";
   function Essential_Result_Pose_Found
     (Handle : System.Address; Value : access Interfaces.Unsigned_8) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_pose_found";
   function Essential_Result_Pose
     (Handle : System.Address; Value : access C_Relative_Pose) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_pose";
   function Essential_Result_Pose_Inlier_Count
     (Handle : System.Address; Count : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_pose_inlier_count";
   function Essential_Result_Pose_Inlier
     (Handle : System.Address; Index : Interfaces.Integer_32;
      Correspondence_Index : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_pose_inlier";
   procedure Essential_Result_Destroy (Handle : System.Address)
     with Import, Convention => C, External_Name => "opencv_calib3d_essential_result_destroy";

   type C_Homography is record
      H00, H01, H02, H10, H11, H12, H20, H21, H22 : Interfaces.C.double;
   end record with Convention => C;
   type C_Homography_Options is record
      Maximum_Iterations : Interfaces.Integer_32;
      Reprojection_Threshold_Pixels, Confidence : Interfaces.C.double;
   end record with Convention => C;
   function Find_Homography_RANSAC
     (Source, Destination : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Options : access constant C_Homography_Options;
      Result : access System.Address) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_find_homography_ransac";
   function Homography_Result_Found
     (Handle : System.Address; Value : access Interfaces.Unsigned_8) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_homography_result_found";
   function Homography_Result_Matrix
     (Handle : System.Address; Value : access C_Homography) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_homography_result_matrix";
   function Homography_Result_Inlier_Count
     (Handle : System.Address; Count : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_homography_result_inlier_count";
   function Homography_Result_Inlier
     (Handle : System.Address; Index : Interfaces.Integer_32;
      Correspondence_Index : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_homography_result_inlier";
   procedure Homography_Result_Destroy (Handle : System.Address)
     with Import, Convention => C, External_Name => "opencv_calib3d_homography_result_destroy";

   type C_Fundamental is record
      F00, F01, F02, F10, F11, F12, F20, F21, F22 : Interfaces.C.double;
   end record with Convention => C;
   type C_Fundamental_Options is record
      Epipolar_Threshold_Pixels, Confidence : Interfaces.C.double;
   end record with Convention => C;
   function Find_Fundamental_RANSAC
     (First, Second : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Options : access constant C_Fundamental_Options;
      Result : access System.Address) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_find_fundamental_ransac";
   function Fundamental_Result_Found
     (Handle : System.Address; Value : access Interfaces.Unsigned_8) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_fundamental_result_found";
   function Fundamental_Result_Matrix
     (Handle : System.Address; Value : access C_Fundamental) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_fundamental_result_matrix";
   function Fundamental_Result_Inlier_Count
     (Handle : System.Address; Count : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_fundamental_result_inlier_count";
   function Fundamental_Result_Inlier
     (Handle : System.Address; Index : Interfaces.Integer_32;
      Correspondence_Index : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_fundamental_result_inlier";
   procedure Fundamental_Result_Destroy (Handle : System.Address)
     with Import, Convention => C, External_Name => "opencv_calib3d_fundamental_result_destroy";

   function Validate_Fundamental_Options
     (Options : access constant C_Fundamental_Options) return Status
     with Import, Convention => C,
       External_Name => "opencv_calib3d_validate_fundamental_options";

   type C_Camera_Intrinsics is record
      Focal_X, Focal_Y, Center_X, Center_Y : Interfaces.C.double;
   end record with Convention => C;

   type C_Planar_Motion is record
      R00, R01, R02, R10, R11, R12, R20, R21, R22 : Interfaces.C.double;
      TX, TY, TZ, NX, NY, NZ : Interfaces.C.double;
   end record with Convention => C;
   type C_Planar_Motion_Array is array (Positive range 1 .. 4) of C_Planar_Motion
     with Convention => C;
   type C_Planar_Decomposition is record
      Count : Interfaces.Integer_32;
      Candidates : C_Planar_Motion_Array;
   end record with Convention => C;
   function Decompose_Homography
     (Matrix : access constant C_Homography;
      Intrinsics : access constant C_Camera_Intrinsics;
      Result : access C_Planar_Decomposition) return Status
     with Import, Convention => C,
       External_Name => "opencv_calib3d_decompose_homography";

   type C_Distortion5 is record
      K1, K2, P1, P2, K3 : Interfaces.C.double;
   end record with Convention => C;

   type C_Pose is record
      RX, RY, RZ, TX, TY, TZ : Interfaces.C.double;
   end record with Convention => C;

   type C_Point3 is record
      X, Y, Z : Interfaces.C.double;
   end record with Convention => C;

   type C_Rotation_Matrix is record
      M00, M01, M02, M10, M11, M12, M20, M21, M22 : Interfaces.C.double;
   end record with Convention => C;

   function Rotation_Matrix_Of
     (Pose : access constant C_Pose;
      Matrix : access C_Rotation_Matrix) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_rotation_matrix_of";

   function Undistort_Normalized
     (Image_Points : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Intrinsics : access constant C_Camera_Intrinsics;
      Distortion : access constant C_Distortion5;
      Normalized_Points : OpenCV.Core.Module_Interop.Output_Mat_Handle) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_undistort_normalized";

   function Native_Version return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_calib3d_native_version";
   function Native_Backend return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_calib3d_native_backend";
   function Last_Error return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C, External_Name => "opencv_calib3d_last_error";

   function Project_Points
     (Object_Points : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Intrinsics    : access constant C_Camera_Intrinsics;
      Distortion    : access constant C_Distortion5;
      Pose          : access constant C_Pose;
      Image_Points  : OpenCV.Core.Module_Interop.Output_Mat_Handle) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_project_points";

   function Camera_Center
     (Pose   : access constant C_Pose;
      Center : access C_Point3) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_camera_center";

   function Solve_PnP_RANSAC
     (Object_Points              : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Image_Points               : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Intrinsics                 : access constant C_Camera_Intrinsics;
      Distortion                 : access constant C_Distortion5;
      Maximum_Iterations         : Interfaces.Integer_32;
      Reprojection_Error_Pixels  : Interfaces.C.double;
      Confidence                 : Interfaces.C.double;
      Result                     : access System.Address) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_solve_pnp_ransac";

   function Refine_Pose_Iterative
     (Object_Points : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Image_Points  : OpenCV.Core.Module_Interop.Input_Mat_Handle;
      Intrinsics    : access constant C_Camera_Intrinsics;
      Distortion    : access constant C_Distortion5;
      Initial_Pose  : access constant C_Pose;
      Refined       : access Interfaces.Unsigned_8;
      Refined_Pose  : access C_Pose) return Status
     with Import, Convention => C,
       External_Name => "opencv_calib3d_refine_pose_iterative";

   function Result_Found
     (Handle : System.Address; Value : access Interfaces.Unsigned_8) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_pose_result_found";
   function Result_Pose
     (Handle : System.Address; Value : access C_Pose) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_pose_result_pose";
   function Result_Inlier_Count
     (Handle : System.Address; Count : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_pose_result_inlier_count";
   function Result_Inlier
     (Handle : System.Address; Index : Interfaces.Integer_32;
      Correspondence_Index : access Interfaces.Integer_32) return Status
     with Import, Convention => C, External_Name => "opencv_calib3d_pose_result_inlier";
   procedure Result_Destroy (Handle : System.Address)
     with Import, Convention => C, External_Name => "opencv_calib3d_pose_result_destroy";

   procedure Check (Code : Status; Operation : String);
end OpenCV.Calib3D.Internal.C_API;
