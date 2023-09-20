within DynTherM.Components.MassTransfer;
package PipeGeometry
  "Package collecting models of different geometries of pipes cross-section"
  partial model BaseClass
    parameter Modelica.Units.SI.Length L "Length";
    Modelica.Units.SI.Area A_cs "Cross-sectional area";
    Modelica.Units.SI.Area A_ht "Heat transfer area";
    Modelica.Units.SI.Length P_cs "Cross-sectional perimeter";
    Modelica.Units.SI.Length Dh "Hydraulic diameter";

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end BaseClass;

  model Circular "Circular cross-section"
    extends BaseClass;
    parameter Modelica.Units.SI.Length D "Pipe diameter";

  equation
    Dh = 4*A_cs/P_cs;
    P_cs = Modelica.Constants.pi*D;
    A_cs = Modelica.Constants.pi*D^2/4;
    A_ht = L*P_cs;

  end Circular;

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
end PipeGeometry;
