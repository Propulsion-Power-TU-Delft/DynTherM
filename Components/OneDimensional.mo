within DynTherM.Components;
package OneDimensional "Package collecting the components modeling coupled heat and mass transfer and featuring a one dimensional spatial discretization"

  model CircularCV
    "Control volume modeling a portion of a circular channel"
    outer Components.Environment environment "Environmental properties";
    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    // Geometry
    parameter Modelica.Units.SI.Length L "Length of the control volume" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length R_ext "External radius of the control volume" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length R_int "Internal radius of the control volume" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length Roughness=0.015*10^(-3) "Pipe roughness" annotation (Dialog(tab="Geometry"));

    // Initialization
    parameter Modelica.Units.SI.Temperature T_start_solid=288.15
      "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid=288.15
      "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Pressure P_start=101325
      "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=environment.initOpt
      "Initialization option" annotation (Dialog(tab="Initialization"));

    DynTherM.CustomInterfaces.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if environment.allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
              {-94,6}},       rotation=0), iconTransformation(extent={{-90,-10},{-70,
              10}})));
    DynTherM.CustomInterfaces.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if environment.allowFlowReversal then +Modelica.Constants.inf else 0, start=
            -m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
              {106,6}},       rotation=0), iconTransformation(extent={{70,-10},{90,
              10}})));
    MassTransfer.CircularPipe fluid(
      redeclare package Medium = Medium,
      DP_opt=DynTherM.Choices.PDropOpt.correlation,
      m_flow_start=m_flow_start,
      P_start=P_start,
      T_start=T_start_fluid,
      X_start=X_start,
      state_start=state_start,
      L=L,
      D=R_int*2,
      Roughness=Roughness)
      annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
    HeatTransfer.TubeConduction solid(
      redeclare model Mat = Mat,
      coeff=1,
      L=L,
      R_ext=R_ext,
      R_int=R_int,
      Tstart=T_start_solid,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{-28,70},{28,30}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a solid_surface
      annotation (Placement(transformation(extent={{-10,70},{10,90}}),
          iconTransformation(extent={{-10,70},{10,90}})));
  equation
    connect(inlet, fluid.inlet)
      annotation (Line(points={{-100,0},{-40,0}}, color={0,0,0}));
    connect(fluid.outlet, outlet)
      annotation (Line(points={{40,0},{100,0}}, color={0,0,0}));
    connect(solid_surface, solid.outlet) annotation (Line(points={{0,80},{0,
            68.4},{3.55271e-15,68.4},{3.55271e-15,56.8}}, color={191,0,0}));
    connect(solid.inlet, fluid.thermalPort) annotation (Line(points={{
            3.55271e-15,43.2},{3.55271e-15,29.2},{0,29.2},{0,15.2}}, color={191,
            0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-80,40},{80,20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
                        Rectangle(
            extent={{-80,-20},{80,-40}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Rectangle(extent={{-80,20},{80,-20}},   lineColor={0,0,0}),
          Rectangle(extent={{-100,60},{100,-60}}, lineColor={0,0,0},
            pattern=LinePattern.Dash)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end CircularCV;

  model RectangularCV
    "Control volume modeling a portion of a rectangular channel"
    outer Components.Environment environment "Environmental properties";
    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    // Geometry
    parameter Modelica.Units.SI.Length L "Length of the control volume" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length W "Width of the control volume" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length H "Height of the control volume" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_north "Thickness of north wall" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_east "Thickness of east wall" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_south "Thickness of south wall" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_west "Thickness of west wall" annotation (Dialog(tab="Geometry"));

    // Initialization
    parameter Modelica.Units.SI.Temperature T_start_solid=288.15
      "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid=288.15
      "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Pressure P_start=101325
      "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=environment.initOpt
      "Initialization option" annotation (Dialog(tab="Initialization"));

    DynTherM.CustomInterfaces.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if environment.allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
              {-94,6}},       rotation=0), iconTransformation(extent={{-90,-10},{-70,
              10}})));
    DynTherM.CustomInterfaces.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if environment.allowFlowReversal then +Modelica.Constants.inf else 0, start=
            -m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
              {106,6}},       rotation=0), iconTransformation(extent={{70,-10},{90,
              10}})));
    MassTransfer.RectangularPipe fluid(
      redeclare package Medium = Medium,
      DP_opt=DynTherM.Choices.PDropOpt.correlation,
      m_flow_start=m_flow_start,
      P_start=P_start,
      T_start=T_start_fluid,
      X_start=X_start,
      state_start=state_start,
      L=L,
      W=W,
      H=H)
      annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
    CustomInterfaces.HeatFluxPort_A solid_surface_north
      annotation (Placement(transformation(extent={{-84,92},{-76,100}}),
          iconTransformation(extent={{-70,60},{-50,80}})));
    CustomInterfaces.HeatFluxPort_A solid_surface_south
      annotation (Placement(transformation(extent={{26,92},{34,100}}),
          iconTransformation(extent={{10,60},{30,80}})));
    CustomInterfaces.HeatFluxPort_A solid_surface_west
      annotation (Placement(transformation(extent={{76,92},{84,100}}),
          iconTransformation(extent={{48,60},{68,80}})));
    CustomInterfaces.HeatFluxPort_A solid_surface_east
      annotation (Placement(transformation(extent={{-34,92},{-26,100}}),
          iconTransformation(extent={{-30,60},{-10,80}})));
    HeatTransfer.WallConduction solid_north(
      redeclare model Mat = Mat,
      t=t_north,
      A=W*L,
      Tstart=T_start_solid,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{-100,66},{-60,36}})));
    HeatTransfer.WallConduction solid_east(
      redeclare model Mat = Mat,
      t=t_east,
      A=(H + 2*t_east)*L,
      Tstart=T_start_solid,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{-50,66},{-10,36}})));
    HeatTransfer.WallConduction solid_south(
      redeclare model Mat = Mat,
      t=t_south,
      A=W*L,
      Tstart=T_start_solid,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{10,66},{50,36}})));
    HeatTransfer.WallConduction solid_west(
      redeclare model Mat = Mat,
      t=t_west,
      A=(H + 2*t_west)*L,
      Tstart=T_start_solid,
      initOpt=environment.initOpt)
      annotation (Placement(transformation(extent={{60,66},{100,36}})));
    Adaptors.heatFluxToHeatFlow conversion_north(A=W*L)
      annotation (Placement(transformation(extent={{-94,90},{-66,62}})));
    Adaptors.heatFluxToHeatFlow conversion_east(A=(H + 2*t_east)*L)
      annotation (Placement(transformation(extent={{-44,90},{-16,62}})));
    Adaptors.heatFluxToHeatFlow conversion_south(A=W*L)
      annotation (Placement(transformation(extent={{16,90},{44,62}})));
    Adaptors.heatFluxToHeatFlow conversion_west(A=(H + 2*t_west)*L)
      annotation (Placement(transformation(extent={{66,90},{94,62}})));
  equation
    connect(inlet, fluid.inlet)
      annotation (Line(points={{-100,0},{-40,0}}, color={0,0,0}));
    connect(fluid.outlet, outlet)
      annotation (Line(points={{40,0},{100,0}}, color={0,0,0}));
    connect(solid_north.inlet, fluid.thermalPort) annotation (Line(points={{-80,45.9},
            {-80,30},{0,30},{0,15.2}}, color={191,0,0}));
    connect(solid_east.inlet, fluid.thermalPort) annotation (Line(points={{-30,45.9},
            {-30,30},{0,30},{0,15.2}}, color={191,0,0}));
    connect(solid_west.inlet, fluid.thermalPort) annotation (Line(points={{80,45.9},
            {80,30},{0,30},{0,15.2}}, color={191,0,0}));
    connect(solid_south.inlet, fluid.thermalPort) annotation (Line(points={{30,45.9},
            {30,30},{0,30},{0,15.2}}, color={191,0,0}));
    connect(solid_surface_north, conversion_north.outlet)
      annotation (Line(points={{-80,96},{-80,84.4}}, color={255,127,0}));
    connect(solid_surface_east, conversion_east.outlet)
      annotation (Line(points={{-30,96},{-30,84.4}}, color={255,127,0}));
    connect(solid_surface_south, conversion_south.outlet)
      annotation (Line(points={{30,96},{30,84.4}}, color={255,127,0}));
    connect(solid_surface_west, conversion_west.outlet)
      annotation (Line(points={{80,96},{80,84.4}}, color={255,127,0}));
    connect(conversion_west.inlet, solid_west.outlet)
      annotation (Line(points={{80,67.6},{80,56.1}}, color={191,0,0}));
    connect(conversion_south.inlet, solid_south.outlet)
      annotation (Line(points={{30,67.6},{30,56.1}}, color={191,0,0}));
    connect(conversion_east.inlet, solid_east.outlet)
      annotation (Line(points={{-30,67.6},{-30,56.1}}, color={191,0,0}));
    connect(conversion_north.inlet, solid_north.outlet)
      annotation (Line(points={{-80,67.6},{-80,56.1}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-80,40},{80,20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
                        Rectangle(
            extent={{-80,-20},{80,-40}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Rectangle(extent={{-80,20},{80,-20}},   lineColor={0,0,0}),
          Rectangle(extent={{-100,60},{100,-60}}, lineColor={0,0,0},
            pattern=LinePattern.Dash)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end RectangularCV;

  model CircularChannel1D "Circular channel implementing one-dimensional spatial discretization"

    outer Components.Environment environment "Environmental properties";
    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    model CV = CircularCV "Control volume";

    // Geometry
    parameter Modelica.Units.SI.Length L "Channel length" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length R_ext "Channel external radius" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length R_int "Channel internal radius" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length Roughness=0.015*10^(-3) "Channel roughness" annotation (Dialog(tab="Geometry"));

    // Initialization
    parameter Modelica.Units.SI.Temperature T_start_solid=288.15
      "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid=288.15
      "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Pressure P_start=101325
      "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=environment.initOpt
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N(min=1) "Number of longitudinal sections in which the tube is discretized";

    CV fluid_cv[N](
      redeclare model Mat = Mat,
      redeclare package Medium = Medium,
      each L=L/N,
      each R_ext=R_ext,
      each R_int=R_int,
      each Roughness=Roughness,
      each T_start_solid=T_start_solid,
      each T_start_fluid=T_start_fluid,
      each P_start=P_start,
      each X_start=X_start,
      each state_start=state_start,
      each m_flow_start=m_flow_start,
      each initOpt=initOpt);

    DynTherM.CustomInterfaces.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if environment.allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
              {-94,6}},       rotation=0), iconTransformation(extent={{-110,-10},{
              -90,10}})));
    DynTherM.CustomInterfaces.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if environment.allowFlowReversal then +Modelica.Constants.inf else 0, start=
            -m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
              {106,6}},       rotation=0), iconTransformation(extent={{90,-10},{110,
              10}})));

    CustomInterfaces.DistributedHeatPort_B solid_surface(N=N) annotation (
        Placement(transformation(extent={{-40,14},{40,80}}), iconTransformation(
            extent={{-40,14},{40,80}})));

  equation
    // thermal connections
    for i in 1:N loop
      connect(solid_surface.ports[i], fluid_cv[i].solid_surface);
    end for;

    // internal flow connections
    for i in 1:(N-1) loop
      connect(fluid_cv[i].outlet, fluid_cv[i+1].inlet);
    end for;

    // boundary flow connections
    connect(inlet, fluid_cv[1].inlet);
    connect(outlet, fluid_cv[N].outlet);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-100,40},{100,20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
                        Rectangle(
            extent={{-100,-20},{100,-40}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Rectangle(extent={{-100,20},{100,-20}}, lineColor={0,0,0}),
          Line(
            points={{-60,20},{-60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-20,20},{-20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,20},{20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{60,20},{60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash)}),         Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end CircularChannel1D;

  model RectangularChannel1D "Rectangular channel implementing one-dimensional spatial discretization"

    outer Components.Environment environment "Environmental properties";
    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    model CV = RectangularCV "Control volume";

    // Geometry
    parameter Modelica.Units.SI.Length L "Channel length" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length W "Width of the channel" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length H "Height of the channel" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_north "Thickness of north wall" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_east "Thickness of east wall" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_south "Thickness of south wall" annotation (Dialog(tab="Geometry"));
    parameter Modelica.Units.SI.Length t_west "Thickness of west wall" annotation (Dialog(tab="Geometry"));

    // Initialization
    parameter Modelica.Units.SI.Temperature T_start_solid=288.15
      "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Temperature T_start_fluid=288.15
      "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.Pressure P_start=101325
      "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFraction X_start[Medium.nX]=Medium.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
    parameter Modelica.Units.SI.MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=environment.initOpt
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N(min=1) "Number of longitudinal sections in which the tube is discretized";

    CV cv[N](
      redeclare model Mat = Mat,
      redeclare package Medium = Medium,
      each L=L/N,
      each W=W,
      each H=H,
      each t_north=t_north,
      each t_east=t_east,
      each t_south=t_south,
      each t_west=t_west,
      each T_start_solid=T_start_solid,
      each T_start_fluid=T_start_fluid,
      each P_start=P_start,
      each X_start=X_start,
      each state_start=state_start,
      each m_flow_start=m_flow_start,
      each initOpt=initOpt);

    DynTherM.CustomInterfaces.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if environment.allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
              {-94,6}},       rotation=0), iconTransformation(extent={{-110,-10},{
              -90,10}})));
    DynTherM.CustomInterfaces.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if environment.allowFlowReversal then +Modelica.Constants.inf else 0, start=
            -m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
              {106,6}},       rotation=0), iconTransformation(extent={{90,-10},{110,
              10}})));

    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_north(N=N) annotation (
        Placement(transformation(extent={{-90,16},{-60,76}}), iconTransformation(
            extent={{-90,16},{-60,76}})));
    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_east(N=N) annotation (
        Placement(transformation(extent={{-40,16},{-10,76}}),  iconTransformation(
            extent={{-40,16},{-10,76}})));
    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_south(N=N) annotation (
        Placement(transformation(extent={{10,16},{40,76}}),    iconTransformation(
            extent={{10,16},{40,76}})));
    CustomInterfaces.DistributedHeatFluxPort_B solid_surface_west(N=N) annotation (
        Placement(transformation(extent={{60,16},{90,76}}),    iconTransformation(
            extent={{60,16},{90,76}})));
  equation
    // thermal connections
    for i in 1:N loop
      // north
      connect(solid_surface_north.ports[i], cv[i].solid_surface_north);

      // east
      connect(solid_surface_east.ports[i], cv[i].solid_surface_east);

      // south
      connect(solid_surface_south.ports[i], cv[i].solid_surface_south);

      // west
      connect(solid_surface_west.ports[i], cv[i].solid_surface_west);

    end for;

    // internal flow connections
    for i in 1:(N-1) loop
      connect(cv[i].outlet, cv[i+1].inlet);
    end for;

    // boundary flow connections
    connect(inlet, cv[1].inlet);
    connect(outlet, cv[N].outlet);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-100,40},{100,20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
                        Rectangle(
            extent={{-100,-20},{100,-40}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Rectangle(extent={{-100,20},{100,-20}}, lineColor={0,0,0}),
          Line(
            points={{-60,20},{-60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-20,20},{-20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,20},{20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{60,20},{60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash)}),         Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end RectangularChannel1D;
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
