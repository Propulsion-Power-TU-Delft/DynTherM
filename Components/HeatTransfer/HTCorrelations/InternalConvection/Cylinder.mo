within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model Cylinder
  "Internal convection for cylindrical shape"
  extends BaseClassInternal;
  parameter Modelica.Units.SI.Velocity V_int=1 "Air speed inside plenum";

equation
  ht = 5.6783*(2 + 0.314*3.2808*V_int);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Cylinder;