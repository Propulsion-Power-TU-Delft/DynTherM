within DynTherM.Components.MassTransfer.PipeGeometry;
model Circular "Circular cross-section"
  extends BaseClass;
  parameter Length D "Pipe diameter";

equation
  Dh = 4*A_cs/P_cs;
  P_cs = Modelica.Constants.pi*D;
  A_cs = Modelica.Constants.pi*D^2/4;
  A_ht = L*P_cs;

end Circular;
