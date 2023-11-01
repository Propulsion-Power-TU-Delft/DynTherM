within DynTherM.Components.Adaptors;
model flowScaler
  "Model used to scale up or down the mass flow rate between two fluid ports"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  input Real scaler=1 "Inlet / outlet mass flow rate" annotation(Dialog(enable = true));

  CustomInterfaces.FluidPort_A inlet(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
  CustomInterfaces.FluidPort_B outlet(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
equation
  inlet.m_flow + scaler*outlet.m_flow = 0 "Mass balance";
  inlet.P = outlet.P "Momentum balance";

  // Energy balance
  inlet.h_outflow = inStream(outlet.h_outflow);
  outlet.h_outflow = inStream(inlet.h_outflow);

  // Independent composition mass balances
  inlet.Xi_outflow = inStream(outlet.Xi_outflow);
  outlet.Xi_outflow = inStream(inlet.Xi_outflow);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-34},{0,34}},     color={0,0,0}),
        Line(points={{0,34},{6,24}},      color={0,0,0}),
        Line(points={{0,34},{-6,24}},     color={0,0,0}),
        Line(points={{-10,-10},{-30,10}}, color={0,0,0}),
        Line(points={{-10,10},{-30,-10}}, color={0,0,0})}),      Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
end flowScaler;
