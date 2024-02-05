within DynTherM.Components.HeatTransfer;
model ExternalConvection "Model of external convection"
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer DynTherM.Components.Environment environment "Environmental properties";

  parameter Integer Nx=1 "Number of control volumes in x-direction";
  parameter Area A "Heat transfer area";

  replaceable model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal(
      Nx=Nx,
      Ny=1,
      T_skin=inlet.ports.T,
      ht_fixed=ones(Nx,1)) annotation (choicesAllMatching=true);

  HTC ht_correlation;

  DynTherM.CustomInterfaces.DistributedHeatPort_A inlet(Nx=Nx, Ny=1)
    annotation (Placement(transformation(extent={{-38,-10},{38,66}}),
        iconTransformation(extent={{-38,-10},{38,66}})));

equation
  for i in 1:Nx loop
    inlet.ports[i,1].Q_flow = ht_correlation.ht[i,1]*A/Nx*(inlet.ports[i,1].T - ht_correlation.T_out[i,1]);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,20},{100,0}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Line(points={{90,-10},{-90,-10}}, color={0,127,255}),
        Line(points={{78,-16},{90,-10}},   color={0,127,255}),
        Line(points={{90,-30},{-90,-30}}, color={0,127,255}),
        Line(points={{78,-4},{90,-10}},    color={0,127,255}),
        Line(points={{78,-36},{90,-30}},   color={0,127,255}),
        Line(points={{78,-24},{90,-30}},   color={0,127,255})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ExternalConvection;
