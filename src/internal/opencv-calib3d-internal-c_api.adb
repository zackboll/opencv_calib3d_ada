package body OpenCV.Calib3D.Internal.C_API is
   procedure Check (Code : Status; Operation : String) is
      use type Interfaces.Integer_32;
   begin
      if Code /= Success then
         raise OpenCV.OpenCV_Error with
           Operation & ": " & Interfaces.C.Strings.Value (Last_Error);
      end if;
   end Check;
end OpenCV.Calib3D.Internal.C_API;
