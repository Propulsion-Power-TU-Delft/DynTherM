within DynTherM.Components.OneDimensional;
model ColdPlateCircularChannelCV
  "Control volume for modelling of heat transfer through a portion of circular channel in a cold plate"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  // Options
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction";
  parameter Choices.PDropOpt DP_opt
    "Select the type of pressure drop to impose";
  parameter Choices.InitOpt initOpt
    "Initialization option" annotation (Dialog(tab="Initialization"));


  // Geometry
  parameter Integer N=1 "Number of control volumes in parallel";
  parameter Length L "Length of the control volume, in the flow direction" annotation (Dialog(tab="Geometry"));
  parameter Length t "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
  parameter Length d "Center to center distance between the parallel pipes" annotation (Dialog(tab="Geometry"));
  parameter Length R_int "Internal radius of the pipe control volume" annotation (Dialog(tab="Geometry"));
  parameter Length Roughness=0.015*10^(-3) "Pipe roughness" annotation (Dialog(tab="Geometry"));

// Initialization
  parameter Temperature T_start_solid=288.15
    "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature T_start_fluid=288.15
    "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure P_start=101325
    "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter MassFraction X_start[Medium.nX]=Medium.reference_X
    "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.ThermodynamicState state_start=
      Medium.setState_pTX(P_start, T_start_fluid, X_start)
    "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
  parameter MassFlowRate m_flow_start=1
    "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
 parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
  parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
  parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
  parameter ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
  parameter PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));


  MassTransfer.CircularPipe circularPipe(
    redeclare package Medium = Medium,
    allowFlowReversal=allowFlowReversal,
    DP_opt=DP_opt,
    m_flow_start=m_flow_start,
    P_start=P_start,
    T_start=T_start_fluid,
    X_start=X_start,
    u_start=u_start,
    rho_start=rho_start,
    dP_start=dP_start,
    state_start=state_start,
    Re_start=Re_start,
    Pr_start=Pr_start,
    N=N,
    L=L,
    D=R_int*2,
    Roughness=Roughness)
    annotation (Placement(transformation(extent={{-40,-44},{40,36}})));

  CustomInterfaces.FluidPort_A          inlet(
    redeclare package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-52,-10},
            {-40,2}},       rotation=0), iconTransformation(extent={{-40,-8},{-26,
            6}})));
  CustomInterfaces.FluidPort_B          outlet(
    redeclare package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=-
          m_flow_start),
    P(start=P_start),
    h_outflow(start=Medium.specificEnthalpy(state_start)),
    Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{40,-10},
            {52,2}},        rotation=0), iconTransformation(extent={{24,-8},{40,
            8}})));
  HeatTransfer.ConductionPlanoConcave2D PCWest( R=R_int, dz=L, y=d/2,
    Tstart=T_start_solid, redeclare model Mat = Mat,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-88,-8},{-44,48}})));
  HeatTransfer.ConductionPlanoConcave2D PCEast(R=R_int,
    y=d/2,     redeclare model Mat = Mat,                     dz=L,
    Tstart=T_start_solid,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{88,-8},{44,48}})));
  HeatTransfer.ConductionPlanoConcave2D PCSouth(R=R_int,
    y=t/2,                                               dz=L,
    Tstart=T_start_solid, redeclare model Mat = Mat,
    initOpt=initOpt)                            annotation (Placement(
        transformation(
        extent={{22,-28},{-22,28}},
        rotation=-90,
        origin={0,-42})));
  HeatTransfer.ConductionPlanoConcave2D PCNorth(R=R_int,
    y=t/2,                                               dz=L,
    Tstart=T_start_solid, redeclare model Mat = Mat,
    initOpt=initOpt)                            annotation (Placement(
        transformation(
        extent={{-22,-28},{22,28}},
        rotation=-90,
        origin={0,64})));
  HeatTransfer.WallConduction2D PlaneNW(
    w=t/2 - R_int/sqrt(2),
    l=d/2 - R_int/sqrt(2),              dz=L,
    Tstart=T_start_solid, redeclare model Mat = Mat,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-82,50},{-52,80}})));
  HeatTransfer.WallConduction2D PlaneSW(w=t/2-R_int/sqrt(2),
    l=d/2 - R_int/sqrt(2),                                   dz=L,
    Tstart=T_start_solid, redeclare model Mat = Mat,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{-82,-56},{-52,-26}})));
  HeatTransfer.WallConduction2D PlaneSE(
    w=t/2 - R_int/sqrt(2),
    l=d/2 - R_int/sqrt(2),              dz=L,
    Tstart=T_start_solid, redeclare model Mat = Mat,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{52,-58},{82,-28}})));
  HeatTransfer.WallConduction2D PlaneNE(
    w=t/2 - R_int/sqrt(2),
    l=d/2 - R_int/sqrt(2),              dz=L,
    Tstart=T_start_solid, redeclare model Mat = Mat,
    initOpt=initOpt)
    annotation (Placement(transformation(extent={{50,50},{80,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a NorthWestHor annotation (
      Placement(transformation(extent={{-106,56},{-86,76}}), iconTransformation(
          extent={{-72,40},{-60,52}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a West annotation (
      Placement(transformation(extent={{-106,10},{-86,30}}), iconTransformation(
          extent={{-72,-6},{-60,6}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a SouthWestHor annotation (
      Placement(transformation(extent={{-106,-50},{-86,-30}}),
        iconTransformation(extent={{-72,-50},{-60,-38}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a SouthBottom annotation (
      Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(
          extent={{-6,-72},{6,-60}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b SouthEastHor annotation (
      Placement(transformation(extent={{86,-52},{106,-32}}), iconTransformation(
          extent={{60,-50},{72,-38}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a East annotation (
      Placement(transformation(extent={{86,10},{106,30}}), iconTransformation(
          extent={{60,-6},{72,6}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b NorthEastHor annotation (
      Placement(transformation(extent={{86,56},{106,76}}), iconTransformation(
          extent={{60,38},{72,50}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a NorthTop annotation (
      Placement(transformation(extent={{-10,86},{10,106}}), iconTransformation(
          extent={{-6,60},{6,72}})));
equation
  connect(inlet, circularPipe.inlet)
    annotation (Line(points={{-46,-4},{-40,-4}},
                                               color={0,0,0}));
  connect(circularPipe.outlet, outlet)
    annotation (Line(points={{40,-4},{46,-4}},
                                             color={0,0,0}));
  connect(PCWest.InletConcave, circularPipe.thermalPort)
    annotation (Line(points={{-61.16,20},{0,20},{0,11.2}},  color={191,0,0}));
  connect(PCNorth.InletConcave, circularPipe.thermalPort)
    annotation (Line(points={{0,59.16},{0,11.2}},  color={191,0,0}));
  connect(PCEast.InletConcave, circularPipe.thermalPort)
    annotation (Line(points={{61.16,20},{0,20},{0,11.2}},  color={191,0,0}));
  connect(PCSouth.InletConcave, circularPipe.thermalPort)
    annotation (Line(points={{0,-37.16},{0,11.2}},  color={191,0,0}));
  connect(PCWest.inletPlanar, PlaneNW.outletS) annotation (Line(points={{-66,45.2},
          {-66,46},{-67.15,46},{-67.15,54.35}}, color={191,0,0}));
  connect(PlaneNW.inletE, PCNorth.outletPlanar) annotation (Line(points={{-53.5,
          65},{-34,65},{-34,64},{-25.2,64}}, color={191,0,0}));
  connect(PlaneSW.inletE, PCSouth.outletPlanar) annotation (Line(points={{-53.5,
          -41},{-53.5,-42},{-25.2,-42}}, color={191,0,0}));
  connect(PlaneSW.inletN, PCWest.outletPlanar) annotation (Line(points={{-66.85,
          -30.35},{-66,-28},{-66,-5.2}}, color={191,0,0}));
  connect(PCSouth.inletPlanar, PlaneSE.outletW) annotation (Line(points={{25.2,-42},
          {48,-42},{48,-43},{53.5,-43}}, color={191,0,0}));
  connect(PlaneSE.inletN, PCEast.outletPlanar) annotation (Line(points={{67.15,-32.35},
          {67.15,-14},{66,-14},{66,-5.2}}, color={191,0,0}));
  connect(PCEast.inletPlanar, PlaneNE.outletS) annotation (Line(points={{66,45.2},
          {66,46},{64.85,46},{64.85,54.35}}, color={191,0,0}));
  connect(PCNorth.inletPlanar, PlaneNE.outletW) annotation (Line(points={{25.2,64},
          {46,64},{46,65},{51.5,65}}, color={191,0,0}));
  connect(PlaneNW.outletW, NorthWestHor) annotation (Line(points={{-80.5,65},{-80.5,
          66},{-96,66}}, color={191,0,0}));
  connect(PlaneSW.outletW, SouthWestHor) annotation (Line(points={{-80.5,-41},{-80.5,
          -40},{-96,-40}}, color={191,0,0}));
  connect(PCSouth.OutletOppoCon, SouthBottom)
    annotation (Line(points={{0,-53},{0,-80}}, color={191,0,0}));
  connect(PlaneSE.inletE, SouthEastHor) annotation (Line(points={{80.5,-43},{80.5,
          -42},{96,-42}}, color={191,0,0}));
  connect(PCEast.OutletOppoCon, East)
    annotation (Line(points={{77,20},{96,20}}, color={191,0,0}));
  connect(West, PCWest.OutletOppoCon)
    annotation (Line(points={{-96,20},{-77,20}}, color={191,0,0}));
  connect(PlaneNE.inletE, NorthEastHor)
    annotation (Line(points={{78.5,65},{80,66},{96,66}}, color={191,0,0}));
  connect(PCNorth.OutletOppoCon, NorthTop)
    annotation (Line(points={{0,75},{0,96}}, color={191,0,0}));
  connect(PlaneNW.inletN, NorthTop) annotation (Line(points={{-66.85,75.65},{
          -68,75.65},{-68,84},{0,84},{0,96}}, color={191,0,0}));
  connect(PlaneNE.inletN, NorthTop) annotation (Line(points={{65.15,75.65},{
          65.15,84},{0,84},{0,96}}, color={191,0,0}));
  connect(PlaneSW.outletS, SouthBottom) annotation (Line(points={{-67.15,-51.65},
          {-67.15,-66},{0,-66},{0,-80}}, color={191,0,0}));
  connect(PlaneSE.outletS, SouthBottom) annotation (Line(points={{66.85,-53.65},
          {66.85,-66},{0,-66},{0,-80}}, color={191,0,0}));
    annotation (Line(points={{46,-4},{46,-4}}, color={0,0,0}),
                Placement(transformation(extent={{-28,-28},{28,28}})),
              Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{40,40},{-40,-40}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-28,-28},{-60,-28},{-60,30},{-26,30},{-34,22},{-38,12},{-40,2},
              {-40,-6},{-38,-14},{-34,-22},{-28,-28}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{28,-28},{60,-28},{60,30},{26,30},{34,22},{38,12},{40,2},{40,-6},
              {38,-14},{34,-22},{28,-28}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-15,-29},{17,-29},{17,29},{-17,29},{-9,21},{-5,11},{-3,1},{-3,
              -7},{-5,-15},{-9,-23},{-15,-29}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          origin={-1,43},
          rotation=90),
        Polygon(
          points={{29,-15},{29,17},{-29,17},{-29,-17},{-21,-9},{-11,-5},{-1,-3},
              {7,-3},{15,-5},{23,-9},{29,-15}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          origin={1,-43},
          rotation=180),
        Rectangle(
          extent={{-60,60},{-30,26}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-60,-28},{-28,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{30,-26},{60,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{28,60},{60,28}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end ColdPlateCircularChannelCV;
