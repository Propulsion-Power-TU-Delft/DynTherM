within DynTherM.Components.HeatTransfer;
model InternalConvection "Model of internal convection"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter Integer Nx=1 "Number of control volumes in x-direction";
  parameter Modelica.Units.SI.Area A "Heat transfer area";

  replaceable model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal(
      Nx=Nx,
      Ny=1,
      ht_fixed=ones(Nx,1)) annotation (choicesAllMatching=true);

  HTC ht_correlation;

  DynTherM.CustomInterfaces.DistributedHeatPort_A inlet(Nx=Nx, Ny=1) annotation (Placement(transformation(extent={{-38,-18},
            {38,58}}), iconTransformation(extent={{-38,-18},{38,58}})));
  DynTherM.CustomInterfaces.DistributedHeatPort_B outlet(Nx=Nx, Ny=1) annotation (Placement(transformation(extent={{-38,-58},
            {38,18}}), iconTransformation(extent={{-38,-58},{38,18}})));

equation
  inlet.ports.Q_flow + outlet.ports.Q_flow = zeros(Nx,1);

  for i in 1:Nx loop
    inlet.ports[i,1].Q_flow = ht_correlation.ht[i,1]*A/Nx*(inlet.ports[i,1].T - outlet.ports[i,1].T);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{90,8},{-90,8}},     color={0,127,255}),
        Line(points={{78,-14},{90,-8}},    color={0,127,255}),
        Line(points={{90,-8},{-90,-8}},   color={0,127,255}),
        Line(points={{78,-2},{90,-8}},     color={0,127,255}),
        Line(points={{78,2},{90,8}},       color={0,127,255}),
        Line(points={{78,14},{90,8}},      color={0,127,255})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end InternalConvection;
