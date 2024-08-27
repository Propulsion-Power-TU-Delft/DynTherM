within DynTherM.Tests.Battery;
package ComparisonCoolingPlate
  "Test to compare the performance of parallel and series cooling plate configurations"
  model Parallel "Validation of Polestar battery module"


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
    parameter Length t_plate = 2*R_int + 0.002 "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
    parameter Length W_plate = (Ns*Np*t_cell) "Center to center distance between the Channels" annotation (Dialog(tab="Geometry"));
    parameter Length R_int = 0.0025  "Channel internal radius" annotation (Dialog(tab="Geometry"));
    parameter Integer N_cv_ch = 2 "Number of control volumes in which each channel is discretized";
    parameter Integer N_channels = 6 "Number of channels in the cold plate";

    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=5.4*0.04416;


    Systems.Battery.ColdPlateCircularParallel2D coldPlateCircularParallel2D(
      redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
      redeclare package Medium = Coolant,
      L=L,
      W_plate=W_plate,
      t_plate=t_plate,
      R_int=R_int,
      DP_opt=DynTherM.Choices.PDropOpt.correlation,
      initOpt=DynTherM.Choices.InitOpt.fixedState,
      T_start_plate=298.15,
      T_start_fluid=298.15,
      m_flow_start=m_flow/6,
      u_start=2.1,
      rho_start(displayUnit="kg/m3") = 1070,
      dP_start(displayUnit="bar") = 8000,
      Re_start=3000,
      Pr_start=30,
      N_cv=N_cv_ch,
      N_channels=N_channels)
      annotation (Placement(transformation(extent={{-50,-118},{58,-10}})));
      BoundaryConditions.flow_source          flow_source1(
      redeclare package Medium = Coolant,
      P_nom=300000,
      T_nom=T_fluid,
      massFlow_nom=m_flow,
      allowFlowReversal=environment.allowFlowReversal,
      use_in_massFlow=false,
      use_in_T=false)
        annotation (Placement(transformation(extent={{-94,-54},{-68,-80}})));
      BoundaryConditions.pressure_sink          pressure_sink1(
      redeclare package Medium = Coolant,
      allowFlowReversal=environment.allowFlowReversal,
      use_ambient=true)
        annotation (Placement(transformation(extent={{84,-74},{100,-58}})));
    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
        Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=0,
          origin={10,70})));
    Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
          600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
          210])
               annotation (Placement(transformation(extent={{-82,62},{-70,74}})));
    Systems.Battery.PouchModuleParallel pouchModuleParallel(
      redeclare model InPlaneCellMat = Materials.PolestarCellInPlane,
      redeclare model CrossPlaneCellMat = Materials.PolestarCellCrossPlane,
      redeclare model FirewallMat = Materials.CompressionPadFoam,
      redeclare model ResinMat = Materials.ThermalResin,
      redeclare model FrameMat = Materials.AluminiumColdPlate,
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
      annotation (Placement(transformation(extent={{-24,-4},{48,68}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{64,-10},{80,6}})));
    Modelica.Blocks.Math.Gain gain(k=-1)
      annotation (Placement(transformation(extent={{-28,84},{-16,96}})));
    Components.TwoDimensional.WallConductionDiscretized ThermalInterface(
      redeclare model Mat = Materials.AluminiumColdPlate,
      t=0.092,
      A=W_cell*(t_cell + t_fw/2)*Ns*Np,
      Tstart=298.15,
      Nx=Ns*Np,
      Ny=1)
      "Thermal Interface material between cooling plate and frame which adds thermal resistance"
      annotation (Placement(transformation(extent={{-14,-22},{16,0}})));
  equation
    connect(coldPlateCircularParallel2D.outlet, pressure_sink1.inlet)
      annotation (Line(points={{58,-64},{72,-64},{72,-66},{84,-66}},
                                                   color={0,0,0}));
    connect(flow_source1.outlet, coldPlateCircularParallel2D.inlet) annotation (
        Line(points={{-68,-67},{-68,-68},{-52,-68},{-52,-64},{-50,-64}},
                                                                     color={0,0,0}));
    connect(pouchModuleParallel.p,signalCurrent. p) annotation (Line(points={{-9.6,
            24.8},{-28,24.8},{-28,70},{2,70}},  color={0,0,255}));
    connect(pouchModuleParallel.n,signalCurrent. n) annotation (Line(points={{19.2,
            24.8},{52,24.8},{52,70},{18,70}},        color={0,0,255}));
    connect(pouchModuleParallel.n,ground. p)
      annotation (Line(points={{19.2,24.8},{72,24.8},{72,6}}, color={0,0,255}));
    connect(gain.u,I_charging. y)
      annotation (Line(points={{-29.2,90},{-58,90},{-58,68},{-69.4,68}},
                                                     color={0,0,127}));
    connect(gain.y,signalCurrent. i)
      annotation (Line(points={{-15.4,90},{10,90},{10,79.6}},
                                                           color={0,0,127}));
    connect(pouchModuleParallel.Bottom,ThermalInterface. inlet)  annotation (Line(
          points={{4.8,8.24},{2,8.24},{2,-7.7},{1,-7.7}},        color={191,0,0}));

    for i in 1:6 loop
      for j in 1:N_cv_ch loop
          connect(ThermalInterface.outlet.ports[2*i,1], coldPlateCircularParallel2D.Top.ports[N_cv_ch,i]);
          connect(ThermalInterface.outlet.ports[(2*i)-1,1], coldPlateCircularParallel2D.Top.ports[N_cv_ch,i]);
      end for;
    end for;
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
          coordinateSystem(preserveAspectRatio=false), graphics={Line(points={{2,-14},
                {2,-24},{24,-24},{24,-42}},        color={238,46,47})}),
      Documentation(info="<html>
</html>"),      Diagram(graphics={                               Line(points={{4,-16},
                {4,-26},{4,-26},{4,-42}},          color={238,46,47})}));
  end Parallel;
end ComparisonCoolingPlate;
