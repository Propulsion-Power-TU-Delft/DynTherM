within DynTherM.Components.HeatTransfer;
model TubeConduction "Dynamic model of conduction in a hollow cylinder"
  replaceable model Mat=DynTherM.Materials.Aluminium constrainedby
    DynTherM.Materials.Properties "Material choice" annotation (choicesAllMatching=true);

  // Geometry
  parameter Integer N_cv=1 "Number of control volumes";
  parameter Real coeff "Fraction of tube with active heat transfer";
  parameter Length L "Tube length";
  parameter Length R_ext "Tube external radius";
  parameter Length R_int "Tube internal radius";

  parameter Length L_window=0 "Window length - aircraft fuselage application" annotation (Dialog(tab="Passive surface"));
  parameter Length H_window=0 "Window height - aircraft fuselage application" annotation (Dialog(tab="Passive surface"));
  parameter Integer Nw_side=0 "Number of windows per fuselage side - aircraft fuselage application" annotation (Dialog(tab="Passive surface"));

  // Initialization
  parameter Temperature Tstart[N_cv,1]=288.15*ones(N_cv,1)
    "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt "Initialization option" annotation (Dialog(tab="Initialization"));

  constant Real pi=Modelica.Constants.pi;
  final parameter Mass m=coeff*Mat.rho*L*pi*(R_ext^2 - R_int^2) "Mass of the tube";
  final parameter Modelica.Units.SI.HeatCapacity Cm=m*Mat.cm
    "Heat capacity of the tube";

  Temperature T_vol[N_cv,1](start=Tstart) "Average temperature of the tube";
  Length A_window_int
    "Equivalent internal window area - aircraft fuselage application";
  Length A_window_ext
    "Equivalent external window area - aircraft fuselage application";

  DynTherM.CustomInterfaces.DistributedHeatPort_A inlet(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(extent={{-40,-12},{40,68}}), iconTransformation(extent={{-40,-12},{40,68}})));
  DynTherM.CustomInterfaces.DistributedHeatPort_B outlet(Nx=N_cv, Ny=1)
    annotation (Placement(transformation(extent={{-40,-68},{40,12}}), iconTransformation(extent={{-40,-68},{40,12}})));

equation
  assert(R_ext > R_int, "External radius must be greater than internal radius");

  A_window_int = H_window/R_int*L_window*Nw_side;
  A_window_ext = H_window/R_ext*L_window*Nw_side;

  for i in 1:N_cv loop
    // Energy balance
    Cm/N_cv*der(T_vol[i,1]) = inlet.ports[i,1].Q_flow + outlet.ports[i,1].Q_flow;

    // Heat conduction through the internal half-thickness
    inlet.ports[i,1].Q_flow = (Mat.lambda/N_cv*(coeff*2*pi*L - A_window_int)*
      (inlet.ports[i,1].T - T_vol[i,1]))/Modelica.Math.log((R_int + R_ext)/(2*R_int));

    // Heat conduction through the external half-thickness
    outlet.ports[i,1].Q_flow = (Mat.lambda/N_cv*(coeff*2*pi*L - A_window_ext)*
      (outlet.ports[i,1].T - T_vol[i,1]))/Modelica.Math.log((2*R_ext)/(R_int + R_ext));
  end for;

initial equation
  for i in 1:N_cv loop
    if initOpt == DynTherM.Choices.InitOpt.steadyState then
      der(T_vol[i,1]) = 0;
    elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
      T_vol[i,1] = Tstart[i,1];
    else
      // do nothing
    end if;
  end for;

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
          textString="TUBE")}),
    Documentation(info="<html>
<p>Model of a cylindrical tube of solid material.</p>
<p>The heat capacity (which is lumped at the center of the tube thickness) is accounted for, as well as the thermal resistance due to the finite heat conduction coefficient. Longitudinal heat conduction is neglected.</p>
<p>The model can be used to reproduce the heat transfer through many tubes in parallel. In that case, the heat flow rate is split equally among the different tubes, assuming there is no heat transfer and temperature difference between them.</p>
<p>The tube element can be used to model the fuselage of an aircraft. In that case, the heat transfer through the cabin windows is neglected and treated separately. </p>
</html>",
        revisions="<html>
</html>"));
end TubeConduction;
