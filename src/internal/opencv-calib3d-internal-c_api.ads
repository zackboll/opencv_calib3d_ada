with Interfaces;
with Interfaces.C;
with Interfaces.C.Strings;
with OpenCV.Core.Module_Interop;
with System;

package OpenCV.Calib3D.Internal.C_API is
   subtype Status is Interfaces.Integer_32;
   Success : constant Status := 0;

   type C_Camera_Intrinsics is record
      Focal_X, Focal_Y, Center_X, Center_Y : Interfaces.C.double;
   end record with Convention => C;

   type C_Distortion5 is record
      K1, K2, P1, P2, K3 : Interfaces.C.double;
   end record with Convention => C;

   type C_Pose is record
      RX, RY, RZ, TX, TY, TZ : Interfaces.C.double;
   end record with Convention => C;

   type C_Point3 is record
      X, Y, Z : Interfaces.C.double;
   end record with Convention => C;

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
