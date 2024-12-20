within DynTherM.Components.OneDimensional;
model PouchCellThermalCV
  "Control volume modeling a portion of a pouch cell"

  replaceable model Mat = Materials.PolestarCellInPlane constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);

  input Length H "Control volume height" annotation (Dialog(enable=true));
  input Area A "Control volume base surface" annotation (Dialog(enable=true));

  // Initialization
  parameter Temperature Tstart=298.15
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  Mass m "Mass of the control volume";
  Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the control volume";
  Temperature T_vol "Average temperature of the control volume";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));

  Modelica.Blocks.Interfaces.RealInput Q_gen "Internal heat generation" annotation (Placement(
        transformation(extent={{-78,26},{-38,66}}), iconTransformation(
        extent={{-14,-14},{14,14}},
        rotation=-90,
        origin={-40,34})));

equation
  m = Mat.rho*A*H;
  Cm = m*Mat.cm;

  Cm*der(T_vol) = inlet.Q_flow + outlet.Q_flow + Q_gen "Energy balance";
  inlet.Q_flow = (Mat.lambda*A*(inlet.T - T_vol))/(H/2)
    "Heat conduction through the internal half-thickness";
  outlet.Q_flow = (Mat.lambda*A*(outlet.T - T_vol))/(H/2)
    "Heat conduction through the external half-thickness";

initial equation
  if initOpt == Choices.InitOpt.steadyState then
    der(T_vol) = 0;
  elseif initOpt == Choices.InitOpt.fixedState then
    T_vol = Tstart;
  else
    // do nothing
  end if;
  annotation (
    Icon(graphics={   Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Text(
          extent={{-30,32},{34,-32}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Cell")}),
    Documentation(info="<html>
<p>Model adapted from WallConduction.</p>
<p>Heat conduction is modelled along the in-plane direction, whereas the cross-plane conduction is neglected.</p>
</html>",
        revisions="<html>
</html>"));
end PouchCellThermalCV;
