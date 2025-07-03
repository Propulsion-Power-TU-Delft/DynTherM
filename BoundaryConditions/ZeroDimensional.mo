within DynTherM.BoundaryConditions;
package ZeroDimensional

  model mechanical
    "Model to impose torque, mechanical power and rotational speed"
    extends DynTherM.Base.min1;
    input Modelica.Units.SI.Torque M=0 "Torque" annotation (Dialog(tab="Boundary conditions", enable=use_M));
    input Modelica.Units.SI.AngularVelocity omega=3000 "Rotational speed" annotation (Dialog(tab="Boundary conditions", enable=use_omega));
    input Modelica.Units.SI.Power W=1e3 "Power" annotation (Dialog(tab="Boundary conditions", enable=use_W));
    parameter Boolean use_M = false "True if torque is given"  annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_omega = false "True if rotational speed is given"  annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_W = false "True if mechanical power is given"  annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_in_M = false "Use connector input for the torque" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    parameter Boolean use_in_omega = false "Use connector input for the rotational speed" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    parameter Boolean use_in_W = false "Use connector input for the mechanical power" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    Modelica.Blocks.Interfaces.RealInput in_M if use_in_M annotation (Placement(
          transformation(
          origin={-60,64},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,20})));
    Modelica.Blocks.Interfaces.RealInput in_omega if use_in_omega annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,-20})));
    Modelica.Blocks.Interfaces.RealInput in_W if use_in_W annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,-20})));
  protected
    Modelica.Blocks.Interfaces.RealInput in_M0 annotation (Placement(
          transformation(
          origin={-60,64},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,60})));
    Modelica.Blocks.Interfaces.RealInput in_omega0 annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-50,0})));
    Modelica.Blocks.Interfaces.RealInput in_W0 annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-50,0})));

  equation
    //Boundary equations
    if use_M then
      mechanical.M = M;
    end if;
    if use_omega then
      mechanical.omega = omega;
    end if;
    if use_W then
      mechanical.M*mechanical.omega =  W;
    end if;

    in_M0 = mechanical.M;
    in_omega0 = mechanical.omega;
    in_W0 = mechanical.M*mechanical.omega;

    // Connect protected connectors to public conditional connectors
    connect(in_M, in_M0);
    connect(in_omega, in_omega0);
    connect(in_W, in_W0);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
              {120,20}}),        graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
            extent={{66,8},{82,-8}},
            lineColor={0,0,0},
            textString="AM")}));
  end mechanical;

  model thermal "Model to impose heat flow rate and temperature"
    extends DynTherM.Base.tin1;
    input Modelica.Units.SI.Temperature T=273.15 "Temperature" annotation (Dialog(tab="Boundary conditions", enable=use_T));
    input Modelica.Units.SI.HeatFlowRate Q=1e3 "Heat flow rate" annotation (Dialog(tab="Boundary conditions", enable=use_Q));
    parameter Boolean use_Q = true "True if heat flow rate is given"  annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_T = false "True if temperature is given" annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_in_T = false "Use connector input for the temperature" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    parameter Boolean use_in_Q = false "Use connector input for the heat flow rate" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
          transformation(
          origin={-60,64},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,20})));
    Modelica.Blocks.Interfaces.RealInput in_Q if use_in_Q annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,-20})));
  protected
    Modelica.Blocks.Interfaces.RealInput in_T0 annotation (Placement(
          transformation(
          origin={-60,64},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,60})));
    Modelica.Blocks.Interfaces.RealInput in_Q0 annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-50,0})));
  equation
    //Boundary equations
    if use_T then
      thermal.T = T;
    end if;
    if use_Q then
      thermal.Q_flow + Q = 0;
    end if;

    in_T0 = thermal.T;
    in_Q0 + thermal.Q_flow = 0;

    // Connect protected connectors to public conditional connectors
    connect(in_T, in_T0);
    connect(in_Q, in_Q0);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
              {120,20}})),                  Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
            extent={{66,8},{82,-8}},
            lineColor={0,0,0},
            textString="AT")}));
  end thermal;

  model thermal_flux "Model to impose heat flux and temperature"
    extends DynTherM.Base.tfin1;
    input Modelica.Units.SI.Temperature T=273.15 "Temperature" annotation (Dialog(tab="Boundary conditions", enable=use_T));
    input Modelica.Units.SI.HeatFlux phi=1e3 "Heat flux" annotation (Dialog(tab="Boundary conditions", enable=use_phi));
    parameter Boolean use_phi = true "True if heat flux is given"  annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_T = false "True if temperature is given" annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_in_T = false "Use connector input for the temperature" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    parameter Boolean use_in_phi = false "Use connector input for the heat flux" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
          transformation(
          origin={-60,64},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,20})));
    Modelica.Blocks.Interfaces.RealInput in_phi if use_in_phi annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,-20})));
  protected
    Modelica.Blocks.Interfaces.RealInput in_T0 annotation (Placement(
          transformation(
          origin={-60,64},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,60})));
    Modelica.Blocks.Interfaces.RealInput in_phi0 annotation (Placement(
          transformation(
          origin={0,90},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-50,0})));
  equation
    //Boundary equations
    if use_T then
      thermal_flux.T = T;
    end if;
    if use_phi then
      thermal_flux.phi + phi = 0;
    end if;

    in_T0 = thermal_flux.T;
    in_phi0 + thermal_flux.phi = 0;

    // Connect protected connectors to public conditional connectors
    connect(in_T, in_T0);
    connect(in_phi, in_phi0);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
              {120,20}})),                  Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
            extent={{66,8},{82,-8}},
            lineColor={0,0,0},
            textString="AT")}));
  end thermal_flux;

  model radiation "Model to impose irradiance and incidence angle"
    extends DynTherM.Base.rin1;
    input Modelica.Units.SI.Irradiance E "Incident irradiance" annotation (Dialog(tab="Boundary conditions", enable=use_E));
    input Modelica.Units.SI.Angle theta "Incidence angle" annotation (Dialog(tab="Boundary conditions", enable=use_theta));
    parameter Boolean use_E = false "True if irradiance is given" annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_in_E = false "Use connector input for the irradiance" annotation (Dialog(group="External inputs"), choices(checkBox=true));
    parameter Boolean use_theta = false "True if theta is given" annotation (Dialog(tab="Boundary conditions"));
    parameter Boolean use_in_theta = false "Use connector input for the theta" annotation (Dialog(group="External inputs"), choices(checkBox=true));

    Modelica.Blocks.Interfaces.RealInput in_E if use_in_E annotation (Placement(
          transformation(
          origin={60,48},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,20})));
    Modelica.Blocks.Interfaces.RealInput in_theta if use_in_theta annotation (Placement(
          transformation(
          origin={80,48},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={54,-20})));
  protected
    Modelica.Blocks.Interfaces.RealInput in_E0 annotation (Placement(
          transformation(
          origin={60,32},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,60})));
    Modelica.Blocks.Interfaces.RealInput in_theta0 annotation (Placement(
          transformation(
          origin={80,32},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,60})));

  equation
    radiative.E_td = 0;
    radiative.E_tr = 0;

    //Boundary equations
    // Need to update if-equations to allow for different combo of inputs/parameter (input E, parameter theta or viceversa)
    if use_E then
      radiative.E_tb = E;
    elseif (use_E and use_theta and (Modelica.Math.cos(theta) > 0)) then
      radiative.theta = theta;
      radiative.E_tb = E*Modelica.Math.cos(theta);
    end if;

    if use_theta then
      radiative.theta = theta;
    end if;

    in_E0 = radiative.E_tb;
    in_theta0 = radiative.theta;

    // Connect protected connectors to public conditional connectors
    connect(in_E, in_E0);
    connect(in_theta, in_theta0);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
              {120,20}})),                  Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
            extent={{64,8},{80,-8}},
            lineColor={0,0,0},
            textString="AT"), Rectangle(
            extent={{88,12},{114,-12}},
            lineColor={244,125,35},
            fillColor={244,125,35},
            fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>
<p>Irradiance&nbsp;is&nbsp;imposed&nbsp;as&nbsp;beam&nbsp;component, whereas&nbsp;the&nbsp;diffused&nbsp;and&nbsp;ground-reflected&nbsp;components&nbsp;are&nbsp;set&nbsp;to&nbsp;zero&nbsp;by&nbsp;default</p>
</html>"));
  end radiation;

  model flow_source "Flow rate source"
    outer DynTherM.Components.Environment environment "Environmental properties";
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    parameter AbsolutePressure P_nom=101325 "Nominal pressure";
    parameter Temperature T_nom=300 "Nominal temperature" annotation(Dialog(enable=not use_in_T and not use_di_T));
    parameter MassFraction X_nom[Medium.nX]=Medium.reference_X "Nominal mass fractions" annotation(Dialog(enable=not use_in_Xw and not use_di_Xw));
    parameter MassFlowRate massFlow_nom=1 "Nominal mass flowrate" annotation(Dialog(enable=not use_in_massFlow and not use_di_massFlow));
    parameter DynTherM.CustomUnits.HydraulicConductance G=0
      "HydraulicConductance";
    parameter Boolean allowFlowReversal=environment.allowFlowReversal
      "= true to allow flow reversal, false restricts to design direction";

    // External Inputs
    parameter Boolean use_in_massFlow = false "Use connector input for the nominal flow rate" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));
    parameter Boolean use_in_T = false "Use connector input for the temperature" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));

    // Direct Inputs
    parameter Boolean use_di_massFlow=false "Use text-based defined mass flow rate" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    parameter Boolean use_di_T=false "Use text-based defined temperature" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    parameter Boolean use_di_X=false "Use text-based defined composition" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    input MassFlowRate massFlow_di=massFlow_nom "Mass flow" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_massFlow));
    input Temperature T_di=T_nom "Temperature" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_T));
    input MassFraction X_di[Medium.nX]=X_nom "Water mass fraction" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_X));

    parameter Pressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
    parameter Temperature T_start=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

    MassFlowRate massFlow(start=massFlow_nom) "Mass flow rate";
    AbsolutePressure P(start=P_nom) "Pressure";
    Temperature T(start=T_nom) "Temperature";
    MassFraction X[Medium.nX](start=X_nom) "Mass fractions";
    Medium.ThermodynamicState state "Thermodynamic state";

    DynTherM.CustomInterfaces.ZeroDimensional.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{
              80,-20},{120,20}}, rotation=0), iconTransformation(extent={{90,-10},
              {110,10}})));
    Modelica.Blocks.Interfaces.RealInput in_massFlow if use_in_massFlow annotation (Placement(
          transformation(
          origin={-60,70},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-58,70})));
    Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
          transformation(
          origin={0,70},
          extent={{10,-10},{-10,10}},
          rotation=90), iconTransformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={0,70})));
  protected
    Modelica.Blocks.Interfaces.RealInput in_massFlow_internal;
    Modelica.Blocks.Interfaces.RealInput in_T_internal;

  equation
    state = Medium.setState_pTX(P, T, X);

    if G > 0 then
      outlet.m_flow = -massFlow + (outlet.P - P_nom)*G;
    else
      outlet.m_flow = -massFlow;
    end if;

    massFlow = in_massFlow_internal;
    if not use_in_massFlow and not use_di_massFlow then
      in_massFlow_internal = massFlow_nom "Flow rate set by parameter";
    elseif use_di_massFlow and not use_in_massFlow then
      in_massFlow_internal = massFlow_di "Flow rate set by direct inputs";
    end if;

    T = in_T_internal;
    if not use_in_T and not use_di_T then
      in_T_internal = T_nom "Temperature set by parameter";
    elseif use_di_T and not use_in_T then
      in_T_internal = T_di "Temperature set by direct inputs";
    end if;

    if use_di_X then
      X = X_di "Composition set by direct inputs";
    else
      X = X_nom "Composition set by parameter";
    end if;

    outlet.P = P;
    outlet.h_outflow = Medium.specificEnthalpy(state);
    outlet.Xi_outflow = X;

    // Connect protected connectors to public conditional connectors
    connect(in_massFlow, in_massFlow_internal);
    connect(in_T, in_T_internal);

    annotation (Documentation(info="<html>
<p>The actual gas used in the component is determined by the replaceable <span style=\"font-family: Courier New;\">Medium</span> package.</p>
<p>The source mass flow rate, temperature and mass fraction can be either specified as parameter, input or wired from input blocks.</p>
<p>Model adapted from <span style=\"font-family: Courier New;\">ThermoPower</span> library by Francesco Casella.</p>
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
  end flow_source;

  model flow_source_ext "Flow rate source using ExternalMedia fluid"
    outer DynTherM.Components.Environment environment "Environmental properties";
    replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
      ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);

    parameter Medium.AbsolutePressure P_nom=101325 "Nominal pressure";
    parameter Medium.Temperature T_nom=300 "Nominal temperature" annotation(Dialog(enable=not use_in_T and not use_di_T));
    parameter Medium.MassFraction X_nom[Medium.nX]=Medium.reference_X "Nominal mass fractions" annotation(Dialog(enable=not use_in_Xw and not use_di_Xw));
    parameter Medium.MassFlowRate massFlow_nom=1 "Nominal mass flowrate" annotation(Dialog(enable=not use_in_massFlow and not use_di_massFlow));
    parameter DynTherM.CustomUnits.HydraulicConductance G=0
      "HydraulicConductance";
    parameter Boolean allowFlowReversal=environment.allowFlowReversal
      "= true to allow flow reversal, false restricts to design direction";

    // External Inputs
    parameter Boolean use_in_massFlow = false "Use connector input for the nominal flow rate" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));
    parameter Boolean use_in_T = false "Use connector input for the temperature" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));

    // Direct Inputs
    parameter Boolean use_di_massFlow=false "Use text-based defined mass flow rate" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    parameter Boolean use_di_T=false "Use text-based defined temperature" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    parameter Boolean use_di_X=false "Use text-based defined composition" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    input Medium.MassFlowRate massFlow_di=massFlow_nom "Mass flow" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_massFlow));
    input Medium.Temperature T_di=T_nom "Temperature" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_T));
    input Medium.MassFraction X_di[Medium.nX]=X_nom "Water mass fraction" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_X));

    Medium.MassFlowRate massFlow(start=massFlow_nom) "Mass flow rate";
    Medium.AbsolutePressure P(start=P_nom) "Pressure";
    Medium.Temperature T(start=T_nom) "Temperature";
    Medium.MassFraction X[Medium.nX](start=X_nom) "Mass fractions";
    Medium.ThermodynamicState state "Thermodynamic state";

    DynTherM.CustomInterfaces.ZeroDimensional.ExtFluidPort_B outlet(redeclare
        package Medium = Medium, m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
             else 0)) annotation (Placement(transformation(extent={{80,-20},{
              120,20}}, rotation=0), iconTransformation(extent={{90,-10},{110,
              10}})));
    Modelica.Blocks.Interfaces.RealInput in_massFlow if use_in_massFlow annotation (Placement(
          transformation(
          origin={-60,50},
          extent={{-10,-10},{10,10}},
          rotation=270), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-80,70})));
    Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
          transformation(
          origin={-20,70},
          extent={{10,-10},{-10,10}},
          rotation=90), iconTransformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-20,70})));

  protected
    Modelica.Blocks.Interfaces.RealInput in_massFlow_internal;
    Modelica.Blocks.Interfaces.RealInput in_T_internal;

  equation
    state = Medium.setState_pTX(P, T, X);

    if G > 0 then
      outlet.m_flow = -massFlow + (outlet.P - P_nom)*G;
    else
      outlet.m_flow = -massFlow;
    end if;

    massFlow = in_massFlow_internal;
    if not use_in_massFlow and not use_di_massFlow then
      in_massFlow_internal = massFlow_nom "Flow rate set by parameter";
    elseif use_di_massFlow and not use_in_massFlow then
      in_massFlow_internal = massFlow_di "Flow rate set by direct inputs";
    end if;

    T = in_T_internal;
    if not use_in_T and not use_di_T then
      in_T_internal = T_nom "Temperature set by parameter";
    elseif use_di_T and not use_in_T then
      in_T_internal = T_di "Temperature set by direct inputs";
    end if;

    if use_di_X then
      X = X_di "Composition set by direct inputs";
    else
      X = X_nom "Composition set by parameter";
    end if;

    outlet.P = P;
    outlet.h_outflow = Medium.specificEnthalpy(state);
    outlet.Xi_outflow = X;

    // Connect protected connectors to public conditional connectors
    connect(in_massFlow, in_massFlow_internal);
    connect(in_T, in_T_internal);

    annotation (Documentation(info="<html>
<p>The actual gas used in the component is determined by the replaceable <span style=\"font-family: Courier New;\">Medium</span> package.</p>
<p>The source mass flow rate, temperature and mass fraction can be either specified as parameter, input or wired from input blocks.</p>
<p>Model adapted from <span style=\"font-family: Courier New;\">ThermoPower</span> library by Francesco Casella.</p>
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
  end flow_source_ext;

  model pressure_source "Flow rate source"
    outer DynTherM.Components.Environment environment "Environmental properties";
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    parameter AbsolutePressure P_nom=101325 "Nominal pressure";
    parameter Temperature T_nom=300 "Nominal temperature" annotation(Dialog(enable=not use_in_T and not use_di_T));
    parameter MassFraction X_nom[Medium.nX]=Medium.reference_X "Nominal mass fractions" annotation(Dialog(enable=not use_in_Xw and not use_di_Xw));
    parameter Boolean allowFlowReversal=environment.allowFlowReversal
      "= true to allow flow reversal, false restricts to design direction";

    // External Inputs
    parameter Boolean use_in_P = false "Use connector input for the pressure" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));
    parameter Boolean use_in_T = false "Use connector input for the temperature" annotation(Dialog(tab="Inputs", group="External inputs"), choices(checkBox=true));

    // Direct Inputs
    parameter Boolean use_di_P=false "Use text-based defined pressure" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    parameter Boolean use_di_T=false "Use text-based defined temperature" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    parameter Boolean use_di_X=false "Use text-based defined composition" annotation(Dialog(tab="Inputs", group="Direct inputs"), choices(checkBox=true));
    input AbsolutePressure P_di=P_nom "Pressure" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_P));
    input Temperature T_di=T_nom "Temperature" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_T));
    input MassFraction X_di[Medium.nX]=X_nom "Water mass fraction" annotation(Dialog(tab="Inputs", group="Direct inputs", enable=use_di_X));

    parameter Pressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
    parameter Temperature T_start=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

    AbsolutePressure P(start=P_nom) "Pressure";
    Temperature T(start=T_nom) "Temperature";
    MassFraction X[Medium.nX](start=X_nom) "Mass fractions";
    Medium.ThermodynamicState state "Thermodynamic state";

    DynTherM.CustomInterfaces.ZeroDimensional.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{
              80,-20},{120,20}}, rotation=0), iconTransformation(extent={{90,-10},
              {110,10}})));
    Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
          transformation(
          origin={0,70},
          extent={{10,-10},{-10,10}},
          rotation=90), iconTransformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={0,70})));
    Modelica.Blocks.Interfaces.RealInput in_P if use_in_P annotation (Placement(
          transformation(
          origin={60,70},
          extent={{10,-10},{-10,10}},
          rotation=90), iconTransformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={60,70})));
  protected
    Modelica.Blocks.Interfaces.RealInput in_P_internal;
    Modelica.Blocks.Interfaces.RealInput in_T_internal;

  equation
    state = Medium.setState_pTX(P, T, X);

    T = in_T_internal;
    if not use_in_T and not use_di_T then
      in_T_internal = T_nom "Temperature set by parameter";
    elseif use_di_T and not use_in_T then
      in_T_internal = T_di "Temperature set by direct inputs";
    end if;

    P = in_P_internal;
    if not use_in_P and not use_di_P then
      in_P_internal = P_nom "Pressure set by parameter";
    elseif use_di_P and not use_in_P then
      in_P_internal = P_di "Pressure set by direct inputs";
    end if;

    if use_di_X then
      X = X_di "Composition set by direct inputs";
    else
      X = X_nom "Composition set by parameter";
    end if;

    outlet.P = P;
    outlet.h_outflow = Medium.specificEnthalpy(state);
    outlet.Xi_outflow = X;

    // Connect protected connectors to public conditional connectors
    connect(in_P, in_P_internal);
    connect(in_T, in_T_internal);

    annotation (Documentation(info="<html>
<p>The actual gas used in the component is determined by the replaceable <span style=\"font-family: Courier New;\">Medium</span> package.</p>
<p>The source mass flow rate, temperature and mass fraction can be either specified as parameter, input or wired from input blocks.</p>
<p>Model adapted from <span style=\"font-family: Courier New;\">ThermoPower</span> library by Francesco Casella.</p>
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
  end pressure_source;

  model pressure_sink "Pressure sink"
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    outer DynTherM.Components.Environment environment "Environmental properties";

    parameter DynTherM.CustomUnits.HydraulicResistance R=0 "Hydraulic Resistance";
    parameter Boolean allowFlowReversal=environment.allowFlowReversal
      "= true to allow flow reversal, false restricts to design direction";
    parameter Boolean use_ambient=true "Use ambient conditions for the plenum";
    input Modelica.Units.SI.Pressure P_di=101325 "Fixed value of pressure" annotation (Dialog(enable=not use_ambient));
    input Modelica.Units.SI.Temperature T_di=288.15 "Fixed value of temperature" annotation (Dialog(enable=not use_ambient));
    input Medium.MassFraction X_di[Medium.nX]=Medium.reference_X "Fixed value of mass fractions" annotation (Dialog(enable=not use_ambient));

    parameter Pressure P_start=101325 "Pressure start value" annotation (Dialog(tab="Initialization"));
    parameter Temperature T_start=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter MassFraction X_start[Medium.nX]=Medium.reference_X "Start gas composition" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));

    Medium.ThermodynamicState state_sink "Thermodynamic state of the sink";

    DynTherM.CustomInterfaces.ZeroDimensional.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{
              -120,-20},{-80,20}}, rotation=0), iconTransformation(extent={{-110,
              -10},{-90,10}})));

  equation
    if use_ambient then

      if R > 0 then
        inlet.P = environment.P_amb + inlet.m_flow*R;
      else
        inlet.P = environment.P_amb;
      end if;

      state_sink = Medium.setState_pTX(environment.P_amb, environment.T_amb, environment.X_amb);

    else

      inlet.P = P_di;
      state_sink = Medium.setState_pTX(P_di, T_di, X_di);
    end if;

    inlet.Xi_outflow = X_di;
    inlet.h_outflow = Medium.specificEnthalpy(state_sink);

    annotation (Documentation(info="<html>
<p>The actual gas used in the component is determined by the replaceable Medium package.</p>
<p>The sink pressure, temperature and mass fraction can be either specified as parameter or input.</p>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
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
  end pressure_sink;
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
          textString="0D")}));
end ZeroDimensional;
