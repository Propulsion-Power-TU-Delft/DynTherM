within DynTherM.Validation;
package Battery
  model PouchCellPolestar
    package Water = Modelica.Media.Water.StandardWater;

    Components.Electrical.PouchCell pouchCell1D(
      H=0.1,
      W=0.35,
      t=0.01,
      C_nom(displayUnit="Ah") = 230400,
      initOpt=environment.initOpt,
      SoC_start=0.1,
      Tstart=298.15)
      annotation (Placement(transformation(extent={{-38,6},{38,56}})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,200; 200,200; 200,200; 500,
          200; 501,120; 700,120; 701,100; 900,100; 901,120; 1000,120; 1001,70; 1200,
          70]) annotation (Placement(transformation(extent={{98,22},{82,38}})));
    Components.OneDimensional.CircularCV single_channel(
      redeclare model Mat = DynTherM.Materials.Aluminium,
      redeclare package Medium = Water,
      allowFlowReversal=environment.allowFlowReversal,
      initOpt=environment.initOpt,
      L=0.3,
      R_ext(displayUnit="mm") = 0.0051,
      R_int(displayUnit="mm") = 0.005,
      T_start_solid=298.15,
      T_start_fluid=298.15,
      m_flow_start=0.044,
      u_start=1)
      annotation (Placement(transformation(extent={{-24,-64},{24,-16}})));
    BoundaryConditions.ZeroDimensional.flow_source flow_source(
      redeclare package Medium = Water,
      T_nom=298.15,
      massFlow_nom=0.04,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=true,
      use_in_T=true)
      annotation (Placement(transformation(extent={{-70,-52},{-46,-28}})));
    BoundaryConditions.ZeroDimensional.pressure_sink pressure_sink(redeclare
        package Medium = Water, allowFlowReversal=environment.allowFlowReversal)
      annotation (Placement(transformation(extent={{50,-50},{70,-30}})));
    Modelica.Blocks.Sources.Ramp m_cool(
      height=0.00004,
      duration=12,
      offset=0.04416,
      startTime=20)
      annotation (Placement(transformation(extent={{-98,-28},{-82,-12}})));
    Modelica.Blocks.Sources.Constant T_cool(k=298.15)
      annotation (Placement(transformation(extent={{-98,12},{-82,28}})));
    inner Components.Environment environment(allowFlowReversal=false, initOpt=
          DynTherM.Choices.InitOpt.fixedState)
      annotation (Placement(transformation(extent={{-98,40},{-64,74}})));
    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
        Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=-90,
          origin={40,30})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{22,-16},{38,0}})));
    Modelica.Blocks.Math.Gain gain(k=-1)
      annotation (Placement(transformation(extent={{72,24},{60,36}})));
  equation
    connect(flow_source.outlet, single_channel.inlet)
      annotation (Line(points={{-46,-40},{-19.2,-40}},color={0,0,0}));
    connect(single_channel.outlet, pressure_sink.inlet)
      annotation (Line(points={{19.2,-40},{50,-40}}, color={0,0,0}));
    connect(single_channel.solid_surface, pouchCell1D.Bottom)
      annotation (Line(points={{0,-20.8},{0,11}},     color={191,0,0}));
    connect(m_cool.y, flow_source.in_massFlow) annotation (Line(points={{-81.2,
            -20},{-68,-20},{-68,-31.6},{-67.6,-31.6}},
                                        color={0,0,127}));
    connect(T_cool.y, flow_source.in_T) annotation (Line(points={{-81.2,20},{
            -60.4,20},{-60.4,-31.6}},
                                color={0,0,127}));
    connect(pouchCell1D.p, signalCurrent.p) annotation (Line(points={{29.9778,
            38.9167},{29.9778,38},{40,38}}, color={0,0,255}));
    connect(pouchCell1D.n, signalCurrent.n) annotation (Line(points={{29.9778,
            22.25},{29.9778,22},{40,22}}, color={0,0,255}));
    connect(pouchCell1D.n, ground.p) annotation (Line(points={{29.9778,22.25},{
            30,20},{30,0}}, color={0,0,255}));
    connect(signalCurrent.i, gain.y)
      annotation (Line(points={{49.6,30},{59.4,30}}, color={0,0,127}));
    connect(gain.u, I_charging.y)
      annotation (Line(points={{73.2,30},{81.2,30}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
              {100,80}})),                                         Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,80}})),
      experiment(
        StopTime=1200,
        Interval=1,
        __Dymola_Algorithm="Dassl"));
  end PouchCellPolestar;

  model PolestarModule "Validation of Polestar battery module"

  // Cell Properties
    parameter Length W_cell = 0.35 "Cell width" annotation (Dialog(tab="Geometry"));
    parameter Length H_cell = 0.1 "Cell height" annotation (Dialog(tab="Geometry"));
    parameter Length t_cell = 0.01 "Cell thickness" annotation (Dialog(tab="Geometry"));
    parameter ElectricCharge C_nom = 66.4*3600 "Nominal cell capacity";

  // Module
    parameter Integer N_cv = 5 "Number of vertical control volumes in which each cell is discretized";
    parameter Integer Ns = 4 "Number of cells connected in series";
    parameter Integer Np = 3 "Number of cells connected in parallel";
    parameter Length t_fw = 0.002 "Thickness of firewall between cells in parallel";
    parameter Length t_resin = 0.0025 "Thickness of resin between cells and frame";
    parameter Length t_frame = 0.005 "Frame thickness";

    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
        Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=0,
          origin={0,60})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
          600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
          210]) annotation (Placement(transformation(extent={{80,76},{64,92}})));
    Systems.Battery.PouchModuleParallel pouchModuleParallel(
      W_cell=W_cell,
      H_cell=H_cell,
      t_cell=t_cell,
      t_fw=t_fw,
      t_resin=t_resin,
      t_frame=t_frame,
      C_nom(displayUnit="Ah") = C_nom,
      SoC_start=0.1,
      N_cv=N_cv,
      Ns=Ns,
      Np=Np)
      annotation (Placement(transformation(extent={{-30,-30},{46,46}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{52,-32},{68,-16}})));
    Modelica.Blocks.Math.Gain gain(k=-1)
      annotation (Placement(transformation(extent={{42,78},{30,90}})));
    BoundaryConditions.OneDimensional.thermal1D bottomBC(
      Nx=Ns*Np,
      T=298.15*ones(Ns*Np),
      use_di_Q=false,
      use_di_T=true,
      use_in_T=false)
      annotation (Placement(transformation(extent={{-36,-26},{36,-54}})));

    BoundaryConditions.OneDimensional.thermal1D topBC(
      Nx=Ns*Np,
      T=298.15*ones(Ns*Np),
      use_di_Q=false,
      use_di_T=true,
      use_in_T=false)
      annotation (Placement(transformation(extent={{-116,34},{-44,6}})));
  equation
    connect(pouchModuleParallel.p, signalCurrent.p) annotation (Line(points={{-14.8,
            0.4},{-14,0.4},{-14,60},{-8,60}},   color={0,0,255}));
    connect(pouchModuleParallel.n, signalCurrent.n) annotation (Line(points={{15.6,
            0.4},{15.6,2},{16,2},{16,60},{8,60}},    color={0,0,255}));
    connect(pouchModuleParallel.n, ground.p)
      annotation (Line(points={{15.6,0.4},{60,0.4},{60,-16}}, color={0,0,255}));
    connect(gain.u, I_charging.y)
      annotation (Line(points={{43.2,84},{63.2,84}}, color={0,0,127}));
    connect(gain.y, signalCurrent.i)
      annotation (Line(points={{29.4,84},{0,84},{0,69.6}}, color={0,0,127}));
    connect(bottomBC.thermal, pouchModuleParallel.Bottom) annotation (Line(points
          ={{0,-40},{0,-18},{0.4,-18},{0.4,-17.08}}, color={191,0,0}));
    connect(topBC.thermal, pouchModuleParallel.Top) annotation (Line(points={{-80,
            20},{-80,40},{0,40},{0,17.88},{0.4,17.88}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Polygon(
            origin={24,16},
            lineColor={78,138,73},
            fillColor={78,138,73},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                   Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Validation of the electro-thermal model of the battery module, without cooling system.</p>
<p>Instead of modelling the cooling system, a constant surface temperature is applied to the top and bottom surfaces.</p>
</html>"));
  end PolestarModule;

  model PolestarModuleCooling1D
    "Validation of Polestar battery module with its cooling system"

    package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;

    // Cell Properties
    parameter Length W_cell = 0.35 "Cell width" annotation (Dialog(tab="Geometry"));
    parameter Length H_cell = 0.1 "Cell height" annotation (Dialog(tab="Geometry"));
    parameter Length t_cell = 0.01 "Cell thickness" annotation (Dialog(tab="Geometry"));
    parameter ElectricCharge C_nom = 66.4*3600 "Nominal cell capacity";

    // Module
    parameter Integer N_cv = 5 "Number of vertical control volumes in which each cell is discretized";
    parameter Integer Ns = 4 "Number of cells connected in series";
    parameter Integer Np = 3 "Number of cells connected in parallel";
    parameter Length t_fw = 2e-3 "Thickness of firewall between cells in parallel";
    parameter Length t_resin = 2.5e-3 "Thickness of resin between cells and frame";
    parameter Length t_frame = 5e-3 "Frame thickness";

    // Cold Plate
    parameter Integer N_cv_channels = 2 "Number of control Volumes for each channel in the cold plate";
    parameter Length L = W_cell "Length of the channel" annotation (Dialog(tab="Geometry"));
    parameter Length t = 2*R_int + 2e-3 "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
    parameter Length d = (Ns*Np*t_cell)/6 "Center to center distance between the Channels" annotation (Dialog(tab="Geometry"));
    parameter Length R_int = 2.5e-3  "Channel internal radius" annotation (Dialog(tab="Geometry"));

    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=0.04416;

    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
        Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=0,
          origin={0,68})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
          600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
          210])
               annotation (Placement(transformation(extent={{-96,84},{-84,96}})));
    Systems.Battery.PouchModuleParallel module(
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
      t_frame=t_frame,
      C_nom(displayUnit="Ah") = C_nom,
      SoC_start=0.1,
      N_cv=N_cv,
      Ns=Ns,
      Np=Np) annotation (Placement(transformation(extent={{-30,-6},{44,68}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{52,4},{68,20}})));
    Modelica.Blocks.Math.Gain gain(k=-1)
      annotation (Placement(transformation(extent={{-46,84},{-34,96}})));
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
      dP_start=8000,
      Re_start=3e3,
      Pr_start=25,
      N_cv=N_cv_channels)
      annotation (Placement(transformation(extent={{-68,-120},{70,-22}})));
    BoundaryConditions.ZeroDimensional.flow_source flow_source1(
      redeclare package Medium = Coolant,
      T_nom=T_fluid,
      massFlow_nom=m_flow,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=false,
      use_in_T=false)
      annotation (Placement(transformation(extent={{-78,-52},{-62,-68}})));
    BoundaryConditions.ZeroDimensional.pressure_sink pressure_sink1(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      use_ambient=false)
      annotation (Placement(transformation(extent={{-64,-86},{-76,-74}})));
    Components.HeatTransfer.WallConduction thermal_interface(
      redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
      t(displayUnit="mm") = 0.092,
      A=module.W_module*module.t_module,
      Tstart=298.15) annotation (Placement(transformation(
          extent={{-12,-10},{12,10}},
          rotation=0,
          origin={0,-28})));
    CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplier(Nx=Ns*Np)
      annotation (Placement(transformation(
          extent={{-14,-4},{14,4}},
          rotation=0,
          origin={0,-12})));
    CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplier1(Nx=
          N_cv_channels)
      annotation (Placement(transformation(
          extent={{-14,-5},{14,5}},
          rotation=180,
          origin={0,-43})));

  equation

    connect(module.p, signalCurrent.p) annotation (Line(points={{-15.2,23.6},{-16,
            23.6},{-16,68},{-8,68}}, color={0,0,255}));
    connect(module.n, signalCurrent.n) annotation (Line(points={{14.4,23.6},{14,23.6},
            {14,68},{8,68}},       color={0,0,255}));
    connect(module.n, ground.p) annotation (Line(points={{14.4,23.6},{60,23.6},{60,
            20}},     color={0,0,255}));
    connect(gain.u, I_charging.y)
      annotation (Line(points={{-47.2,90},{-83.4,90}},
                                                     color={0,0,127}));
    connect(gain.y, signalCurrent.i)
      annotation (Line(points={{-33.4,90},{0,90},{0,77.6}},color={0,0,127}));
    connect(flow_source1.outlet, coldPlatePolestar.inlet) annotation (Line(points={{-62,-60},
            {-54,-60},{-54,-67.08},{-43.3571,-67.08}},
          color={0,0,0}));
    connect(pressure_sink1.inlet, coldPlatePolestar.outlet) annotation (Line(
          points={{-64,-80},{-54,-80},{-54,-73.94},{-43.3571,-73.94}},
          color={0,0,0}));
    connect(heatFlowMultiplier1.single, thermal_interface.outlet)
      annotation (Line(points={{0,-40},{0,-31.4}}, color={191,0,0}));
    connect(thermal_interface.inlet, heatFlowMultiplier.single)
      annotation (Line(points={{0,-24.6},{0,-14.4}}, color={191,0,0}));
    connect(heatFlowMultiplier.distributed, module.Bottom) annotation (Line(
          points={{0,-9.6},{-0.4,-9.6},{-0.4,6.58}}, color={191,0,0}));
    connect(heatFlowMultiplier1.distributed, coldPlatePolestar.Top) annotation (
        Line(points={{0,-46},{0.0142857,-46},{0.0142857,-54.34}}, color={191,0,0}));
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
  end PolestarModuleCooling1D;

  model PolestarModuleCooling2D
    "Validation of Polestar battery module with its cooling system"

    package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;

    // Cell Properties
    parameter Length W_cell = 0.35 "Cell width" annotation (Dialog(tab="Geometry"));
    parameter Length H_cell = 0.1 "Cell height" annotation (Dialog(tab="Geometry"));
    parameter Length t_cell = 0.01 "Cell thickness" annotation (Dialog(tab="Geometry"));
    parameter ElectricCharge C_nom = 66.4*3600 "Nominal cell capacity";

    // Module
    parameter Integer N_cv = 5 "Number of vertical control volumes in which each cell is discretized";
    parameter Integer Ns = 4 "Number of cells connected in series";
    parameter Integer Np = 3 "Number of cells connected in parallel";
    parameter Length t_fw = 2e-3 "Thickness of firewall between cells in parallel";
    parameter Length t_resin = 2.5e-3 "Thickness of resin between cells and frame";
    parameter Length t_frame = 5e-3 "Frame thickness";

    // Cold Plate
    parameter Integer N_cv_channels = 2 "Number of control Volumes for each channel in the cold plate";
    parameter Length L = W_cell "Length of the channel" annotation (Dialog(tab="Geometry"));
    parameter Length t = 2*R_int + 0.002 "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
    parameter Length d = (Ns*Np*t_cell)/6  "Center to center distance between the Channels" annotation (Dialog(tab="Geometry"));
    parameter Length R_int = 0.0025  "Channel internal radius" annotation (Dialog(tab="Geometry"));

    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=0.04416;

    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
        Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=0,
          origin={0,68})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
          600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
          210])
               annotation (Placement(transformation(extent={{-96,84},{-84,96}})));
    Systems.Battery.PouchModuleParallel module(
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
      t_frame=t_frame,
      C_nom(displayUnit="Ah") = C_nom,
      SoC_start=0.1,
      N_cv=N_cv,
      Ns=Ns,
      Np=Np) annotation (Placement(transformation(extent={{-30,-10},{44,64}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{52,-2},{68,14}})));
    Modelica.Blocks.Math.Gain gain(k=-1)
      annotation (Placement(transformation(extent={{-46,84},{-34,96}})));
    Systems.Battery.ColdPlatePolestar2D coldPlatePolestar(
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
      dP_start=8000,
      Re_start=3e3,
      Pr_start=25,
      N_cv=N_cv_channels)
      annotation (Placement(transformation(extent={{-62,-114},{62,-26}})));
    BoundaryConditions.ZeroDimensional.flow_source flow_source1(
      redeclare package Medium = Coolant,
      T_nom=T_fluid,
      massFlow_nom=m_flow,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=false,
      use_in_T=false)
      annotation (Placement(transformation(extent={{-78,-52},{-62,-68}})));
    BoundaryConditions.ZeroDimensional.pressure_sink pressure_sink1(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      use_ambient=false)
      annotation (Placement(transformation(extent={{-64,-86},{-76,-74}})));
    Components.TwoDimensional.WallConductionHorizontal2D thermalInterface(
      x=module.t_module,
      y(displayUnit="mm") = 0.092,
      z=module.W_module,
      N=Ns*Np,
      Tstart=298.15)
      "Thermal Interface material between cooling plate and frame which adds thermal resistance"
      annotation (Placement(transformation(extent={{-16,-34},{16,-6}})));

  equation
    for i in 1:6 loop
      for j in 1:N_cv_channels loop
          connect(thermalInterface.South.ports[i], coldPlatePolestar.Top.ports[N_cv_channels,i]);
      end for;
    end for;

    connect(module.p, signalCurrent.p) annotation (Line(points={{-15.2,19.6},{-16,
            19.6},{-16,68},{-8,68}}, color={0,0,255}));
    connect(module.n, signalCurrent.n) annotation (Line(points={{14.4,19.6},{14,19.6},
            {14,68},{8,68}}, color={0,0,255}));
    connect(module.n, ground.p)
      annotation (Line(points={{14.4,19.6},{60,19.6},{60,14}}, color={0,0,255}));
    connect(gain.u, I_charging.y) annotation (Line(points={{-47.2,90},{-83.4,90}},
                                                     color={0,0,127}));
    connect(gain.y, signalCurrent.i) annotation (Line(points={{-33.4,90},{0,90},{0,77.6}},color={0,0,127}));
    connect(flow_source1.outlet, coldPlatePolestar.inlet) annotation (Line(points={{-62,-60},
            {-52,-60},{-52,-66.48},{-39.8571,-66.48}},
          color={0,0,0}));
    connect(pressure_sink1.inlet, coldPlatePolestar.outlet) annotation (Line(
          points={{-64,-80},{-52,-80},{-52,-72.64},{-39.8571,-72.64}},
          color={0,0,0}));
    connect(thermalInterface.North, module.Bottom) annotation (Line(points={{0,-15.8},
            {0,2.58},{-0.4,2.58}}, color={191,0,0}));
   annotation (Line(
          points={{-11,-16.3},{8,-16.3},{8,-38},{12.7429,-38},{12.7429,-47.58}},
          color={191,0,0}),
               Line(
          points={{-11,-16.3},{8,-16.3},{8,-38},{12.7429,-38},{12.7429,-47.58}},
          color={191,0,0}),
                Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
        __Dymola_Algorithm="Esdirk45a"),
      Documentation(info="<html>
<p>Reproducing the fast charging simulation results for a battery module for the work [1], using the same inputs and the parameters. This also takes into account the temperature variation in the moduel</p>
<h4>Comments:</h4>
<ul>
<li>The battery temperature results (Average temperature, minnimum temperature and maximum temperature) are in quite good agreement with the work [1].</li>
<li>The thickness of the resin and frame are not known. So, proceeded with good guesses for their dimensions. Its worth noting that the thickness of the resin will have more effect on the temperature distribution of the cells becuase of its lower thermal conductivity.</li>
</ul>
<h4>References</h4>
<p>[1] I. Gul. &quot;Time efficient simulations for advanced battery cooling concepts&quot;, M.Sc. thesis, Polestar / Universitat Politecnica de Catalunya, 2023.</p>
</html>"));
  end PolestarModuleCooling2D;

  model PolestarValidationWith2PlateCooling
    "Validation of Polestar battery module with its cooling system"

    package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;

    // Cell Properties
    parameter Length W_cell = 0.35 "Cell width" annotation (Dialog(tab="Geometry"));
    parameter Length H_cell = 0.1 "Cell height" annotation (Dialog(tab="Geometry"));
    parameter Length t_cell = 0.01 "Cell thickness" annotation (Dialog(tab="Geometry"));
    parameter ElectricCharge C_nom = 66.4*3600 "Nominal cell capacity";

    // Module
    parameter Integer N_cv = 10 "Number of vertical control volumes in which each cell is discretized";
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
          origin={0,70})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
          600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
          210])
               annotation (Placement(transformation(extent={{-96,84},{-84,96}})));
    Systems.Battery.PouchModuleParallel module(
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
      t_frame=t_frame,
      C_nom(displayUnit="Ah") = C_nom,
      SoC_start=0.1,
      N_cv=N_cv,
      Ns=Ns,
      Np=Np) annotation (Placement(transformation(extent={{-30,-12},{44,68}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{32,-16},{48,0}})));
    Modelica.Blocks.Math.Gain gain(k=-1)
      annotation (Placement(transformation(extent={{-46,84},{-34,96}})));
    Systems.Battery.ColdPlatePolestar coldPlateBottom(
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
      dP_start=8000,
      Re_start=3e3,
      Pr_start=25,
      N_cv=1) annotation (Placement(transformation(extent={{22,-102},{120,-38}})));
    BoundaryConditions.ZeroDimensional.flow_source source_bottom(
      redeclare package Medium = Coolant,
      T_nom=T_fluid,
      massFlow_nom=m_flow,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=false,
      use_in_T=false)
      annotation (Placement(transformation(extent={{14,-54},{26,-66}})));
    BoundaryConditions.ZeroDimensional.pressure_sink sink_bottom(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      use_ambient=false)
      annotation (Placement(transformation(extent={{26,-86},{14,-74}})));
    Components.HeatTransfer.WallConduction thermal_interface_bottom(
      redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
      t(displayUnit="mm") = 0.092,
      A=module.W_module*module.t_module) annotation (Placement(transformation(
          extent={{-10,-7},{10,7}},
          rotation=0,
          origin={70,-37})));
    CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplier(Nx=Ns*Np)
      annotation (Placement(transformation(
          extent={{-14,-5},{14,5}},
          rotation=0,
          origin={70,-27})));
    CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplier1(Nx=
          N_cv_channels)
      annotation (Placement(transformation(
          extent={{-14,5},{14,-5}},
          rotation=0,
          origin={70,-47})));
    Systems.Battery.ColdPlatePolestar coldPlateTop(
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
      dP_start=8000,
      Re_start=3e3,
      Pr_start=25,
      N_cv=1) annotation (Placement(transformation(extent={{-78,-104},{20,-36}})));
    BoundaryConditions.ZeroDimensional.flow_source source_top(
      redeclare package Medium = Coolant,
      T_nom=T_fluid,
      massFlow_nom=m_flow,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=false,
      use_in_T=false)
      annotation (Placement(transformation(extent={{-86,-54},{-74,-66}})));
    BoundaryConditions.ZeroDimensional.pressure_sink sink_top(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      use_ambient=false)
      annotation (Placement(transformation(extent={{-74,-86},{-86,-74}})));
    Components.HeatTransfer.WallConduction thermal_interface_top(
      redeclare model Mat = Materials.AluminiumColdPlate,
      t(displayUnit="mm") = 0.092,
      A=module.W_module*module.t_module) annotation (Placement(transformation(
          extent={{-10,-7},{10,7}},
          rotation=0,
          origin={-30,-37})));
    CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplier2(Nx=Ns*Np) annotation (Placement(transformation(
          extent={{-14,5},{14,-5}},
          rotation=180,
          origin={-30,-27})));
    CustomInterfaces.Adaptors.heatFlowMultiplier1D heatFlowMultiplier3(Nx=
          N_cv_channels)
      annotation (Placement(transformation(
          extent={{-14,-5},{14,5}},
          rotation=180,
          origin={-30,-47})));
  equation
    connect(module.p, signalCurrent.p) annotation (Line(points={{-15.2,20},{-16,20},
            {-16,70},{-8,70}}, color={0,0,255}));
    connect(module.n, signalCurrent.n) annotation (Line(points={{14.4,20},{14,20},
            {14,70},{8,70}}, color={0,0,255}));
    connect(module.n, ground.p)
      annotation (Line(points={{14.4,20},{40,20},{40,0}}, color={0,0,255}));
    connect(gain.u, I_charging.y)
      annotation (Line(points={{-47.2,90},{-83.4,90}},
                                                     color={0,0,127}));
    connect(gain.y, signalCurrent.i)
      annotation (Line(points={{-33.4,90},{0,90},{0,79.6}},color={0,0,127}));
    connect(source_bottom.outlet, coldPlateBottom.inlet) annotation (Line(points={
            {26,-60},{32,-60},{32,-68},{39.5,-68},{39.5,-67.44}}, color={0,0,0}));
    connect(sink_bottom.inlet, coldPlateBottom.outlet) annotation (Line(points={{26,
            -80},{32,-80},{32,-72},{40,-72},{40,-71.92},{39.5,-71.92}}, color={0,0,
            0}));
    connect(heatFlowMultiplier.single, thermal_interface_bottom.inlet)
      annotation (Line(points={{70,-30},{70,-34.62}}, color={191,0,0}));
    connect(heatFlowMultiplier.distributed, module.Bottom) annotation (Line(
          points={{70,-24},{70,-16},{0,-16},{0,1.6},{-0.4,1.6}}, color={191,0,0}));
    connect(thermal_interface_bottom.outlet, heatFlowMultiplier1.single)
      annotation (Line(points={{70,-39.38},{70,-44}}, color={191,0,0}));
    connect(coldPlateBottom.Top, heatFlowMultiplier1.distributed) annotation (
        Line(points={{70.3,-59.12},{70.3,-58},{70,-58},{70,-50}}, color={191,0,0}));
    connect(source_top.outlet, coldPlateTop.inlet) annotation (Line(points={{-74,-60},
            {-68,-60},{-68,-67.28},{-60.5,-67.28}}, color={0,0,0}));
    connect(sink_top.inlet, coldPlateTop.outlet) annotation (Line(points={{-74,-80},
            {-68,-80},{-68,-72.04},{-60.5,-72.04}}, color={0,0,0}));
    connect(heatFlowMultiplier2.single, thermal_interface_top.inlet)
      annotation (Line(points={{-30,-30},{-30,-34.62}}, color={191,0,0}));
    connect(thermal_interface_top.outlet, heatFlowMultiplier3.single)
      annotation (Line(points={{-30,-39.38},{-30,-44}}, color={191,0,0}));
    connect(coldPlateTop.Top, heatFlowMultiplier3.distributed) annotation (Line(
          points={{-29.7,-58.44},{-30,-58.44},{-30,-50}}, color={191,0,0}));
    connect(module.Top, heatFlowMultiplier2.distributed) annotation (Line(points={
            {-0.4,38.4},{-30,38.4},{-30,-24}}, color={191,0,0}));
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
end Battery;
