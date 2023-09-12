within DynTherM.Components.HeatTransfer;
model SphereConduction
  "0D Dynamic model of conduction in a hollow sphere"
  replaceable model Mat=DynTherM.Materials.Aluminium constrainedby
    DynTherM.Materials.Properties          "Material choice" annotation (choicesAllMatching=true);
  parameter Real coeff=1 "Fraction of sphere with active heat transfer";
  parameter Modelica.Units.SI.Length R_ext "Sphere external radius";
  parameter Modelica.Units.SI.Length R_int "Sphere internal radius";
  parameter Modelica.Units.SI.Temperature Tstart=300
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt "Initialization option"
    annotation (Dialog(tab="Initialization"));
  constant Real pi=Modelica.Constants.pi;
  final parameter Modelica.Units.SI.Mass m=coeff*Mat.rho*4/3*pi*(R_ext^3 -
      R_int^3) "Mass of the sphere";
  final parameter Modelica.Units.SI.HeatCapacity Cm=m*Mat.cm
    "Heat capacity of the sphere";
  Modelica.Units.SI.Temperature T_vol(start=Tstart)
    "Average temperature of the sphere";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));
equation
  assert(R_ext > R_int, "External radius must be greater than internal radius");
  Cm*der(T_vol) = inlet.Q_flow + outlet.Q_flow "Energy balance";
  inlet.Q_flow = coeff*(4*pi*Mat.lambda*R_ext*R_int)*(inlet.T - T_vol)/
    ((R_ext + R_int)/2 - R_int) "Heat conduction through the internal half-thickness";
  outlet.Q_flow = coeff*(4*pi*Mat.lambda*R_ext*R_int)*(outlet.T - T_vol)/
    (R_ext - (R_ext + R_int)/2) "Heat conduction through the external half-thickness";
initial equation
  if initOpt == DynTherM.Choices.InitOpt.steadyState then
    der(T_vol) = 0;
  elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
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
          extent={{-44,36},{46,-36}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="SPHERE")}),
    Documentation);
end SphereConduction;
