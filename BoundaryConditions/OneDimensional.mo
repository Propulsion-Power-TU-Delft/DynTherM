within DynTherM.BoundaryConditions;
package OneDimensional

  model thermal1D
    "Model to impose 1D distribution of heat flow rate and temperature"

    parameter Integer Nx=1 "Number of ports in x-direction";
    parameter Temperature T[Nx]=273.15*ones(Nx) "Temperature" annotation (Dialog(tab="Boundary conditions", enable=use_di_T));
    parameter HeatFlowRate Q[Nx]=1e3*ones(Nx) "Heat flow rate" annotation (Dialog(tab="Boundary conditions", enable=use_di_Q));
    parameter Boolean use_di_Q = true "True if heat flow rate is given as parameter" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));
    parameter Boolean use_di_T = false "True if temperature is given as parameter" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));
    parameter Boolean use_in_Q = false "True if heat flow rate is given as input" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));
    parameter Boolean use_in_T = false "True if temperature is given as input" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));

    CustomInterfaces.OneDimensional.HeatPort1D_A thermal(Nx=Nx)
      annotation (Placement(transformation(extent={{74,-16},{106,16}}),
          iconTransformation(extent={{74,-16},{106,16}})));
    Modelica.Blocks.Interfaces.RealInput in_Q if use_in_Q annotation (
       Placement(transformation(
         origin={98,14},
         extent={{-10,-10},{10,10}},
         rotation=270), iconTransformation(
         extent={{-4,-4},{4,4}},
         rotation=270,
         origin={80,8})));
    Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
         transformation(
         origin={82,14},
         extent={{10,-10},{-10,10}},
         rotation=90), iconTransformation(
         extent={{4,-4},{-4,4}},
         rotation=90,
         origin={100,8})));

  protected
    Modelica.Blocks.Interfaces.RealInput in_Q_internal;
    Modelica.Blocks.Interfaces.RealInput in_T_internal;

  equation
    //Boundary equations
    if use_di_T then
      thermal.ports.T = T;
    elseif use_in_T then
      thermal.ports.T = in_T_internal*ones(Nx);
    end if;

    if use_di_Q then
      thermal.ports.Q_flow + Q = zeros(Nx);
    elseif use_in_Q then
      thermal.ports.Q_flow + in_Q_internal/(Nx)*ones(Nx) = zeros(Nx);
    end if;

    // Connect protected connectors to public conditional connectors
    connect(in_Q, in_Q_internal);
    connect(in_T, in_T_internal);

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
              {120,20}})),                  Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
            extent={{86,8},{94,0}},
            lineColor={238,46,47},
            textString="thermal distributed")}),
      experiment(StopTime=60, __Dymola_Algorithm="Dassl"));
  end thermal1D;

  model thermal_flux1D
    "Model to impose 1D distribution of heat flux and temperature"

    parameter Integer Nx(min=1) "Number of ports in x-direction";
    input Modelica.Units.SI.Temperature T[Nx]=273.15*ones(Nx) "Temperature" annotation (Dialog(tab="Boundary conditions", enable=use_T));
    input Modelica.Units.SI.HeatFlux phi[Nx]=1e3*ones(Nx) "Heat flux" annotation (Dialog(tab="Boundary conditions", enable=use_phi));
    parameter Boolean use_phi = true "True if heat flux is given" annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_T = false "True if temperature is given" annotation (Dialog(tab="Boundary conditions"));

    CustomInterfaces.OneDimensional.HeatFluxPort1D_A thermal_flux(Nx=Nx)
      annotation (Placement(transformation(extent={{74,-16},{106,16}}),
          iconTransformation(extent={{74,-16},{106,16}})));

  equation
    //Boundary equations
    if use_T then
      thermal_flux.ports.T = T;
    end if;
    if use_phi then
      thermal_flux.ports.phi + phi = zeros(Nx);
    end if;

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
              {120,20}})),                  Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
            extent={{86,8},{94,0}},
            lineColor={238,46,47},
            textString="thermal distributed")}));
  end thermal_flux1D;

  model flow_source_1D "1D flow rate source"
    outer DynTherM.Components.Environment environment "Environmental properties";
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    parameter Integer N(min=1) "Number of ports";
    parameter Boolean use_massFlow=true "Use text-based defined mass flow rate" annotation(choices(checkBox=true));
    parameter Boolean use_T=true "Use text-based defined temperature" annotation(choices(checkBox=true));
    parameter Boolean use_X=true "Use text-based defined composition" annotation(choices(checkBox=true));
    input Medium.MassFlowRate massFlow_di[N]=ones(N) "Mass flow" annotation(Dialog(enable=use_di_massFlow));
    input Medium.Temperature T_di[N]=288.15*ones(N) "Temperature" annotation(Dialog(enable=use_di_T));
    input Medium.MassFraction X_di[Medium.nX]=Medium.reference_X "Mass fractions" annotation(Dialog(enable=use_di_X));
    parameter Boolean allowFlowReversal=environment.allowFlowReversal
      "= true to allow flow reversal, false restricts to design direction";

    Medium.MassFlowRate massFlow[N](start=ones(N)) "Mass flow rate";
    Medium.AbsolutePressure P[N](start=101325*ones(N)) "Pressure";
    Medium.Temperature T[N](start=288.15*ones(N)) "Temperature";
    Medium.MassFraction X[Medium.nX](start=Medium.reference_X) "Mass fractions";
    Medium.ThermodynamicState state[N] "Thermodynamic state";

    CustomInterfaces.OneDimensional.FluidPort1D_B outlet(redeclare package
        Medium =
          Medium, Nx=N) annotation (Placement(transformation(extent={{98,-2},{120,20}}),
          iconTransformation(
          extent={{-40,-35},{40,35}},
          rotation=-90,
          origin={97,7.10543e-15})));

  equation

    if use_massFlow then
      massFlow = massFlow_di;
    end if;

    if use_T then
      T = T_di;
    end if;

    if use_X then
      X = X_di;
    end if;

    outlet.ports.m_flow + massFlow = zeros(N);
    outlet.ports.P = P;

    for i in 1:N loop
      state[i] = Medium.setState_pTX(P[i], T[i], X);
      outlet.ports[i].h_outflow = Medium.specificEnthalpy(state[i]);
      outlet.ports[i].Xi_outflow = X;
    end for;

    annotation (Documentation(info="<html>
<p>Flow source model featuring a distributed flow port.</p>
</html>", revisions="<html>
</html>"), Icon(graphics={
          Rectangle(
            extent={{-100,60},{90,-60}},
            lineColor={128,128,128},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-40,-40},{40,0},{-40,40},{0,0},{-40,-40}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid)}));
  end flow_source_1D;

  model pressure_sink_1D "1D pressure sink"
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    outer DynTherM.Components.Environment environment "Environmental properties";

    parameter Integer N(min=1) "Number of ports";
    parameter Boolean allowFlowReversal=environment.allowFlowReversal
      "= true to allow flow reversal, false restricts to design direction";
    parameter Boolean use_ambient=true "Use ambient conditions for the plenum";
    parameter Modelica.Units.SI.Pressure P_di[N]=101325*ones(N) "Fixed value of pressure" annotation (Dialog(enable=not use_ambient));
    parameter Modelica.Units.SI.Temperature T_di[N]=288.15*ones(N) "Fixed value of temperature" annotation (Dialog(enable=not use_ambient));
    parameter Medium.MassFraction X_di[Medium.nX]=Medium.reference_X "Fixed value of mass fractions" annotation (Dialog(enable=not use_ambient));

    Medium.ThermodynamicState state[N] "Thermodynamic state of the sink";

    DynTherM.CustomInterfaces.OneDimensional.FluidPort1D_A inlet(redeclare
        package Medium = Medium, Nx=N) annotation (Placement(transformation(extent
            ={{-120,-20},{-80,20}}, rotation=0), iconTransformation(
          extent={{-40,-42},{40,42}},
          rotation=-90,
          origin={-100,7.10543e-15})));

  equation
    if use_ambient then
      inlet.ports.P = environment.P_amb*ones(N);
      for i in 1:N loop
        state[i] = Medium.setState_pTX(environment.P_amb, environment.T_amb, environment.X_amb);
        inlet.ports[i].h_outflow = Medium.specificEnthalpy(state[i]);
        inlet.ports[i].Xi_outflow = X_di;
      end for;
    else
      inlet.ports.P = P_di;
      for i in 1:N loop
        state[i] = Medium.setState_pTX(P_di[i], T_di[i], X_di);
        inlet.ports[i].h_outflow = Medium.specificEnthalpy(state[i]);
        inlet.ports[i].Xi_outflow = X_di;
      end for;
    end if;

    annotation (Documentation(info="<html>
<p>Pressure sink model featuring a distributed flow port.</p>
</html>", revisions="<html>
</html>"), Icon(graphics={     Ellipse(
            extent={{-90,-90},{90,90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),             Text(
            extent={{-62,62},{66,-66}},
            lineColor={0,0,0},
            fillColor={159,159,223},
            fillPattern=FillPattern.None,
            textString="P SINK")}));
  end pressure_sink_1D;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),      Text(
          extent={{-98,98},{94,-94}},
          lineColor={0,0,0},
          textString="1D")}));
end OneDimensional;
