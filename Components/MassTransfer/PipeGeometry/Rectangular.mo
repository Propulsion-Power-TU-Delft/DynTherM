within DynTherM.Components.MassTransfer.PipeGeometry;
model Rectangular "Rectangular cross-section"
  extends BaseClass;
  parameter Modelica.Units.SI.Length W "Width of rectangular channel";
  parameter Modelica.Units.SI.Length H "Height of rectangular channel";

  Real phi_star "Geometrical correction";

equation
  phi_star = 2/3 + 11/24*H/W*(2 - H/W);
  Dh = 4*A_cs/P_cs;
  P_cs = 2*(H + W);
  A_cs = H*W;
  A_ht = L*P_cs;

end Rectangular;
