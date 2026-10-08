package OpenCV.Calib3D.Internal.Point_Refinement is
   --  Implementation-only numerical kernel; Pose translation is already unit.
   subtype Scalar is OpenCV.Float64_Value;
   type Residual_Vector is array (Natural range 0 .. 3) of Scalar;
   type Jacobian_Matrix is array (Natural range 0 .. 3, Natural range 0 .. 2) of Scalar;
   type Evaluation is record
      Residual : Residual_Vector;
      Jacobian : Jacobian_Matrix;
      Second_Depth, First_Error, Second_Error, Norm : Scalar;
   end record;
   function Second_Position (X : Object_Point; Pose : Relative_Camera_Pose)
      return Object_Point;
   function Evaluate
     (X : Object_Point; A, B : Normalized_Image_Point;
      Pose : Relative_Camera_Pose; Value : out Evaluation) return Boolean;
   function Step
     (Value : Evaluation; Lambda : Scalar; Update : out Object_Point)
      return Boolean;
   function Refine
     (Initial : Triangulated_Point; A, B : Normalized_Image_Point;
      Pose : Relative_Camera_Pose; Iterations : Positive)
      return Point_Refinement_Result;
end OpenCV.Calib3D.Internal.Point_Refinement;