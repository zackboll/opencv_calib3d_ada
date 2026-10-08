with Ada.Numerics.Generic_Elementary_Functions;

package body OpenCV.Calib3D.Internal.Point_Refinement is
   use type Scalar;
   package Math is new Ada.Numerics.Generic_Elementary_Functions (Scalar);

   function Checked (V : Scalar) return Scalar is
   begin
      if V /= V or else V < -Scalar'Last or else V > Scalar'Last then
         raise Constraint_Error with "Unrepresentable refinement arithmetic";
      end if;
      return V;
   end Checked;

   function Hypot (A, B : Scalar) return Scalar is
      S : constant Scalar := Scalar'Max (abs A, abs B);
   begin
      if S = 0.0 then return 0.0; end if;
      return Checked (S * Math.Sqrt ((A / S) ** 2 + (B / S) ** 2));
   end Hypot;

   function Second_Position (X : Object_Point; Pose : Relative_Camera_Pose)
      return Object_Point
   is
      Q : Object_Point := Object_Point (Pose.Translation_Direction);
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Q (I) := Checked (Q (I) + Checked (Pose.Rotation_First_To_Second (I, J) * X (J)));
         end loop;
      end loop;
      return Q;
   end Second_Position;

   function Evaluate
     (X : Object_Point; A, B : Normalized_Image_Point;
      Pose : Relative_Camera_Pose; Value : out Evaluation) return Boolean
   is
      Q : constant Object_Point := Second_Position (X, Pose);
      U, V, W, H : Scalar;
   begin
      Value := (Residual => [others => 0.0], Jacobian => [others => [others => 0.0]], others => 0.0);
      for Component of X loop
         U := Checked (Component);
      end loop;
      if X (2) <= 0.0 or else Q (2) <= 0.0 then return False; end if;
      U := Checked (X (0) / X (2));
      V := Checked (X (1) / X (2));
      W := Checked (Q (0) / Q (2));
      H := Checked (Q (1) / Q (2));
      Value.Residual := [Checked (U - A (0)), Checked (V - A (1)),
                         Checked (W - B (0)), Checked (H - B (1))];
      Value.First_Error := Hypot (Value.Residual (0), Value.Residual (1));
      Value.Second_Error := Hypot (Value.Residual (2), Value.Residual (3));
      Value.Norm := Hypot (Value.First_Error, Value.Second_Error);
      Value.Second_Depth := Q (2);
      --  -u/Z avoids forming Z squared or unnecessarily overflowing X/Z**2.
      Value.Jacobian (0, 0) := Checked (1.0 / X (2));
      Value.Jacobian (0, 1) := 0.0;
      Value.Jacobian (0, 2) := Checked (-U / X (2));
      Value.Jacobian (1, 0) := 0.0;
      Value.Jacobian (1, 1) := Checked (1.0 / X (2));
      Value.Jacobian (1, 2) := Checked (-V / X (2));
      for J in 0 .. 2 loop
         Value.Jacobian (2, J) := Checked (Checked
           (Pose.Rotation_First_To_Second (0, J) - Checked
            (W * Pose.Rotation_First_To_Second (2, J))) / Q (2));
         Value.Jacobian (3, J) := Checked (Checked
           (Pose.Rotation_First_To_Second (1, J) - Checked
            (H * Pose.Rotation_First_To_Second (2, J))) / Q (2));
      end loop;
      return True;
   exception
      when Constraint_Error =>
         Value := (Residual => [others => 0.0], Jacobian => [others => [others => 0.0]], others => 0.0);
         return False;
   end Evaluate;

   function Step
     (Value : Evaluation; Lambda : Scalar; Update : out Object_Point)
      return Boolean
   is
      type Matrix is array (Natural range 0 .. 2, Natural range 0 .. 2) of Scalar;
      L, N : Matrix := [others => [others => 0.0]];
      G, Y, Solution : Object_Point := [others => 0.0];
      J : Jacobian_Matrix;
      S, V : Scalar := 0.0;
   begin
      Update := [others => 0.0];
      for Element of Value.Jacobian loop
         S := Scalar'Max (S, abs Checked (Element));
      end loop;
      if S = 0.0 or else Checked (Lambda) <= 0.0 then return False; end if;
      for Row in 0 .. 3 loop
         for Col in 0 .. 2 loop
            J (Row, Col) := Value.Jacobian (Row, Col) / S;
         end loop;
      end loop;
      for I in 0 .. 2 loop
         for Row in 0 .. 3 loop
            G (I) := Checked (G (I) - Checked (J (Row, I) * Value.Residual (Row)));
         end loop;
         for K in 0 .. 2 loop
            for Row in 0 .. 3 loop
               N (I, K) := Checked (N (I, K) + J (Row, I) * J (Row, K));
            end loop;
         end loop;
         N (I, I) := Checked (N (I, I) + Lambda);
      end loop;
      --  Checked Cholesky; no invalid solve is interpreted as a zero step.
      for I in 0 .. 2 loop
         for K in 0 .. I loop
            V := N (I, K);
            for H in 0 .. K - 1 loop
               V := Checked (V - L (I, H) * L (K, H));
            end loop;
            if I = K then
               if V <= 0.0 then return False; end if;
               L (I, K) := Checked (Math.Sqrt (V));
            else
               L (I, K) := Checked (V / L (K, K));
            end if;
         end loop;
         V := G (I);
         for K in 0 .. I - 1 loop
            V := Checked (V - L (I, K) * Y (K));
         end loop;
         Y (I) := Checked (V / L (I, I));
      end loop;
      for I in reverse 0 .. 2 loop
         V := Y (I);
         for K in I + 1 .. 2 loop
            V := Checked (V - L (K, I) * Solution (K));
         end loop;
         Solution (I) := Checked (V / L (I, I));
         Update (I) := Checked (Solution (I) / S);
      end loop;
      return True;
   exception
      when Constraint_Error => return False;
   end Step;

   function Refine
     (Initial : Triangulated_Point; A, B : Normalized_Image_Point;
      Pose : Relative_Camera_Pose; Iterations : Positive)
      return Point_Refinement_Result
   is
      Result : Point_Refinement_Result := (Outcome => Numerically_Unavailable,
        Point => Initial, others => <>);
      Current, Trial : Evaluation;
      X : Object_Point := Initial.Position_In_First_Camera;
      Candidate, Update : Object_Point;
      Lambda : Scalar := 1.0E-3;
      Accepted, Solved : Boolean;
   begin
      if not Evaluate (X, A, B, Pose, Current) then return Result; end if;
      Result.Residuals_Evaluated := True;
      Result.Initial_Normalized_RMS := Current.Norm / 2.0;
      Result.Final_Normalized_RMS := Result.Initial_Normalized_RMS;
      Result.Outcome := No_Improving_Step;
      for Iteration in 1 .. Iterations loop
         exit when Current.Norm = 0.0;
         Accepted := False;
         Solved := False;
         for Attempt in 1 .. 8 loop
            if Step (Current, Lambda, Update) then
               Solved := True;
               begin
                  for Axis in 0 .. 2 loop
                     Candidate (Axis) := Checked (X (Axis) + Update (Axis));
                  end loop;
                  if Evaluate (Candidate, A, B, Pose, Trial) and then Trial.Norm < Current.Norm then
                     X := Candidate;
                     Current := Trial;
                     Result.Accepted_Steps := Result.Accepted_Steps + 1;
                     Accepted := True;
                  end if;
               exception
                  when Constraint_Error => null; --  Reject trial, retain accepted state.
               end;
            end if;
            exit when Accepted;
            Lambda := Scalar'Min (1.0E12, Lambda * 10.0);
         end loop;
         if not Solved and then Result.Accepted_Steps = 0 then
            Result.Outcome := Numerically_Unavailable;
         end if;
         exit when not Accepted;
         Lambda := Scalar'Max (1.0E-12, Lambda / 10.0);
      end loop;
      if Result.Accepted_Steps > 0 then
         Result.Outcome := Improved;
         Result.Final_Normalized_RMS := Current.Norm / 2.0;
         Result.Point := (Usable, X, X (2), Current.Second_Depth,
                          Current.First_Error, Current.Second_Error);
      end if;
      return Result;
   end Refine;
end OpenCV.Calib3D.Internal.Point_Refinement;