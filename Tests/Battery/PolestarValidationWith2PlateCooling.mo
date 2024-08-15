within DynTherM.Tests.Battery;
model PolestarValidationWith2PlateCooling
  "Validation of Polestar battery module with its cooling system"

  package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;

  // Cell Properties
  parameter Length W_cell = 0.35 "Cell width" annotation (Dialog(tab="Geometry"));
  parameter Length H_cell = 0.1 "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Length t_cell = 0.01 "Cell thickness" annotation (Dialog(tab="Geometry"));
  parameter ElectricCharge C_nom = 66.4*3600 "Nominal cell capacity";

  // Module
  parameter Integer N_cv = 20 "Number of vertical control volumes in which each cell is discretized";
  parameter Integer Ns = 4 "Number of cells connected in series";
  parameter Integer Np = 3 "Number of cells connected in parallel";
  parameter Length t_fw = 0.002 "Thickness of firewall between cells in parallel";
  parameter Length t_resin = 0.0025 "Thickness of resin between cells and frame";
  parameter Length t_frame = 0.005 "Frame thickness";

  // Cold Plate
  parameter Length L = W_cell "Length of the channel" annotation (Dialog(tab="Geometry"));
  parameter Length t = 2*R_int + 0.002 "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
  parameter Length d = (Ns*Np*t_cell)/6 "Center to center distance between the Channels" annotation (Dialog(tab="Geometry"));
  parameter Length R_int = 0.0025  "Channel internal radius" annotation (Dialog(tab="Geometry"));

  parameter Temperature T_fluid=298.15;
  parameter MassFlowRate m_flow=0.04416;

  // Initialization

  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
      Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={0,62})));
  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
        600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
        210])
             annotation (Placement(transformation(extent={{-92,54},{-80,66}})));
  Systems.Battery.PouchModuleParallel pouchModuleParallel(
    redeclare model InPlaneCellMat = DynTherM.Materials.PolestarCellInPlane,
    redeclare model CrossPlaneCellMat =
        DynTherM.Materials.PolestarCellCrossPlane,
    redeclare model FirewallMat = DynTherM.Materials.CompressionPadFoam,
    redeclare model ResinMat = DynTherM.Materials.ThermalResin,
    redeclare model FrameMat = DynTherM.Materials.AluminiumColdPlate,
    W_cell=W_cell,
    H_cell=H_cell,
    t_cell=t_cell,
    t_fw=t_fw,
    t_resin=t_resin,
    t_frame=t_frame,                                      C_nom(displayUnit="Ah")
       = C_nom,
    SoC_start=0.1,
    N_cv=N_cv,
    Ns=Ns,
    Np=Np)
    annotation (Placement(transformation(extent={{-34,-12},{38,60}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{54,-18},{70,-2}})));
  Modelica.Blocks.Math.Gain gain(k=-1)
    annotation (Placement(transformation(extent={{-38,76},{-26,88}})));
  Systems.Battery.ColdPlatePolestar coldPlatePolestar(
    redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    T_start_solid(displayUnit="degC") = 298.15,
    T_start_fluid=298.15,
    m_flow_start=0.04416,
    rho_start(displayUnit="kg/m3") = 1e3,
    u_start=2,
    dP_start=200000000,
    Re_start=3e3,
    Pr_start=25,
    N_cv=1,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{18,-90},{110,-24}})));
    BoundaryConditions.flow_source          flow_source1(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
      annotation (Placement(transformation(extent={{6,-40},{20,-54}})));
    BoundaryConditions.pressure_sink          pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false)
      annotation (Placement(transformation(extent={{18,-70},{6,-58}})));
  Components.HeatTransfer.WallConduction wallConduction(
    redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
    t=0.092,
    A=W_cell*(t_cell + t_fw)*Ns*Np)
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=90,
        origin={16,-30})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier(Nx=12, Ny=1)
    annotation (Placement(transformation(extent={{-13,-4},{13,4}},
        rotation=90,
        origin={1,-30})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=1,  Ny=1)
    annotation (Placement(transformation(extent={{-13,4},{13,-4}},
        rotation=90,
        origin={27,-30})));
  Systems.Battery.ColdPlatePolestar coldPlatePolestar1(
    redeclare model Mat = Materials.AluminiumColdPlate,
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    T_start_solid(displayUnit="degC") = 298.15,
    T_start_fluid=298.15,
    m_flow_start=0.04416,
    rho_start(displayUnit="kg/m3") = 1e3,
    u_start=2,
    dP_start=200000000,
    Re_start=3e3,
    Pr_start=25,
    N_cv=1,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{-76,-90},{16,-24}})));
    BoundaryConditions.flow_source          flow_source2(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
      annotation (Placement(transformation(extent={{-90,-42},{-78,-54}})));
    BoundaryConditions.pressure_sink          pressure_sink2(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false)
      annotation (Placement(transformation(extent={{-76,-70},{-88,-58}})));
  Components.HeatTransfer.WallConduction wallConduction1(
    redeclare model Mat = Materials.AluminiumColdPlate,
    t=0.092,
    A=W_cell*(t_cell + t_fw)*Ns*Np)
    annotation (Placement(transformation(extent={{-6,6},{6,-6}},
        rotation=90,
        origin={-80,-6})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier2(Nx=12, Ny=1)
    annotation (Placement(transformation(extent={{-13,4},{13,-4}},
        rotation=90,
        origin={-59,-6})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier3(Nx=1, Ny=1)
    annotation (Placement(transformation(extent={{-13,4},{13,-4}},
        rotation=90,
        origin={-67,-26})));
equation
  connect(pouchModuleParallel.p, signalCurrent.p) annotation (Line(points={{-19.6,
          16.8},{-38,16.8},{-38,62},{-8,62}}, color={0,0,255}));
  connect(pouchModuleParallel.n, signalCurrent.n) annotation (Line(points={{9.2,
          16.8},{42,16.8},{42,62},{8,62}},         color={0,0,255}));
  connect(pouchModuleParallel.n, ground.p)
    annotation (Line(points={{9.2,16.8},{62,16.8},{62,-2}}, color={0,0,255}));
  connect(gain.u, I_charging.y)
    annotation (Line(points={{-39.2,82},{-68,82},{-68,60},{-79.4,60}},
                                                   color={0,0,127}));
  connect(gain.y, signalCurrent.i)
    annotation (Line(points={{-25.4,82},{0,82},{0,71.6}},color={0,0,127}));
  connect(flow_source1.outlet, coldPlatePolestar.inlet) annotation (Line(points={{20,-47},
          {20,-48},{30,-48},{30,-54.36},{34.4286,-54.36}},
        color={0,0,0}));
  connect(pressure_sink1.inlet, coldPlatePolestar.outlet) annotation (Line(
        points={{18,-64},{30,-64},{30,-58.98},{34.4286,-58.98}},
        color={0,0,0}));
  connect(heatFlowMultiplier.single, wallConduction.inlet)
    annotation (Line(points={{3.4,-30},{13.96,-30}},         color={191,0,0}));
  connect(heatFlowMultiplier.distributed, pouchModuleParallel.Bottom)
    annotation (Line(points={{-1.4,-30},{-6,-30},{-6,0.24},{-5.2,0.24}},
                                                                   color={191,0,
          0}));
  connect(wallConduction.outlet, heatFlowMultiplier1.single)
    annotation (Line(points={{18.04,-30},{24.6,-30}},   color={191,0,0}));
  connect(coldPlatePolestar.Top, heatFlowMultiplier1.distributed) annotation (
      Line(points={{63.3429,-45.78},{63.3429,-42},{64,-42},{64,-30},{29.4,-30}},
        color={191,0,0}));
  connect(flow_source2.outlet, coldPlatePolestar1.inlet) annotation (Line(
        points={{-78,-48},{-60,-48},{-60,-54.36},{-59.5714,-54.36}}, color={0,0,
          0}));
  connect(pressure_sink2.inlet, coldPlatePolestar1.outlet) annotation (Line(
        points={{-76,-64},{-70,-64},{-70,-58},{-64,-58},{-64,-58.98},{-59.5714,
          -58.98}},
        color={0,0,0}));
  connect(heatFlowMultiplier2.single, wallConduction1.inlet) annotation (Line(
        points={{-61.4,-6},{-77.96,-6}},                   color={191,0,0}));
  connect(wallConduction1.outlet, heatFlowMultiplier3.single) annotation (Line(
        points={{-82.04,-6},{-90,-6},{-90,-26},{-69.4,-26}}, color={191,0,0}));
  connect(coldPlatePolestar1.Top, heatFlowMultiplier3.distributed) annotation (
      Line(points={{-30.6571,-45.78},{-30.6571,-42},{-30,-42},{-30,-26},{-64.6,
          -26}},
        color={191,0,0}));
  connect(pouchModuleParallel.Top, heatFlowMultiplier2.distributed) annotation (
     Line(points={{-5.2,33.36},{-46,33.36},{-46,-6},{-56.6,-6}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1200,
      Interval=1,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>Reproducing the fast charging simulation results for a battery module for the work [1], using the same inputs and the parameters. </p>
<h4>Comments:</h4>
<ul>
<li>The battery temperature results (Average temperature, minnimum temperature and maximum temperature) are in quite good agreement with the work [1].</li>
<li>The thickness of the resin and frame are not known. So, proceeded with good guesses for their dimensions. Its worth noting that the thickness of the resin will have more effect on the temperature distribution of the cells becuase of its lower thermal conductivity.</li>
</ul>
<h4>Results:</h4>
<ol>
<li>Cell Temperatures</li>
<p><img src=\"modelica://DynTherM/Figures/Results_Cell_Temperatures.png\"/></p>
<li>Cell Internal heat generation rate</li>
<p><img src=\"modelica://DynTherM/Figures/Cell_Internal heatGeneration Rate.PNG\"/></p>
<li>Coolant Temperatures</li>
</ol>
<p><img src=\"modelica://DynTherM/Figures/Results CoolantTemperatures.png\"/></p>
<p><br><h4>References</h4></p>
<p>[1] I. Gul. &quot;Time efficient simulations for advanced battery cooling concepts&quot;, M.Sc. thesis, Polestar / Universitat Politecnica de Catalunya, 2023.</p>
</html>"));
end PolestarValidationWith2PlateCooling;
