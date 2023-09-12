within DynTherM.Systems.Helicopter.Subsystems;
model FuselagePanel
  "Basic rectangular fuselage panel consisting of radiation and convection effects on a sandwich panel with window."
  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter Modelica.Units.SI.Temperature T_start=300 "Temperature start value"
    annotation (Dialog(group="Initialization"));
  parameter Modelica.Units.SI.Length W "Panel width"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length L "Panel length"
    annotation (Dialog(group="Dimensions"));
  parameter Modelica.Units.SI.Length t "Panel thickness"
    annotation (Dialog(group="Dimensions"));
  parameter Boolean include_window "=true is panel has windows";
  parameter Boolean use_r_eff=false "Use direct input for r_eff";
  parameter Real r_eff_di=0.02 "Reflection coefficient" annotation (Dialog(enable=use_r_eff));
  parameter Boolean use_fus_radiation=true "Use the component for fuselage radiation";
  parameter Modelica.Units.SI.Area A_window=0 "Window area"
    annotation (Dialog(group="Dimensions", enable=include_window));
  parameter Modelica.Units.SI.Length t_window=0 "Window thickness"
    annotation (Dialog(group="Dimensions", enable=include_window));
  parameter Integer Nw_side=0 "Number of windows per fuselage side" annotation (Dialog(group="Dimensions", enable = include_window));
  parameter Modelica.Units.SI.Angle csi
    "Tilt angle of the surface wrt horizontal (facing ground 90<)"
    annotation (Dialog(group="Dimensions"));
  parameter Real coeff=1 "Effective convection area";

  final parameter Modelica.Units.SI.Area A_tot_window=A_window*Nw_side
    "Total window area";
  final parameter Modelica.Units.SI.Area A_fus=coeff*W*L - A_tot_window
    "Fuselage area (minus windows)";
  final parameter Modelica.Units.SI.Area A_fus_rad(fixed=false)
    "Radiative fuselage area";
  // Materials
  replaceable model Fuselage_int =
      DynTherM.Materials.AirbusEES.Fuselage constrainedby
    DynTherM.Materials.Properties
    "Internal fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  replaceable model Fuselage_core =
      DynTherM.Materials.AirbusEES.Fuselage constrainedby
    DynTherM.Materials.Properties
    "Core fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  replaceable model Fuselage_ext =
      DynTherM.Materials.AirbusEES.Fuselage constrainedby
    DynTherM.Materials.Properties
    "External fuselage material" annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Materials"));
  // User Heat Correlations Choices
  replaceable model HTC_int =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    annotation (choicesAllMatching=true, Dialog(tab="Materials + Correlations", group="Heat Correlations"));
  // External Radiation
  Components.HeatTransfer.WallRadiation wallRadiation(csi=csi,
    redeclare model Material = Materials.Paints.ExteriorGreyMetal (greybody=DynTherM.Choices.GreyBodyOpt.Greybody),
    A=A_fus_rad)                                                                                             annotation (Placement(transformation(extent={{-10,10},{
            10,-10}},
        rotation=90,
        origin={-60,50})));
  Components.HeatTransfer.WindowRadiation windowRadiation(redeclare model Mat
      = Materials.AirbusEES.Window,
                               A=A_tot_window,
    use_r_eff=use_r_eff,
    r_eff_di=r_eff_di)                         if include_window annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-60,-50})));
  // External Convection
  Components.HeatTransfer.ExternalConvection extConvection(A=A_fus, redeclare
      model HTC = HTC_ext) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-60,70})));
  Components.HeatTransfer.ExternalConvection extConvectionWindow(A=A_tot_window,
      redeclare model HTC = HTC_ext) if include_window annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-60,-30})));
  // Conduction
  SandwichPanel sandwichPanel(
    L=L,
    W=W,
    A_window=A_window,
    Nw_side=Nw_side,
    t_total=t,
    T_start=T_start,
    redeclare model Fuselage_int = Fuselage_int,
    redeclare model Fuselage_core = Fuselage_core,
    redeclare model Fuselage_ext = Fuselage_ext)
                              annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-14,50})));
  Components.HeatTransfer.WallConduction windowConduction(
    Tstart=T_start,
    redeclare model Mat = Materials.AirbusEES.Window,
    t=t_window,
    A=A_tot_window,
    initOpt=environment.initOpt) if include_window annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-18,-30})));
  // Internal Convection
  Components.HeatTransfer.InternalConvection intConvection(A=A_fus, redeclare
      model HTC = HTC_int) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={44,50})));
  Components.HeatTransfer.InternalConvection intConvectionWindow(
      redeclare model HTC = HTC_int, A=A_tot_window)
                                     if include_window annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={44,-30})));
  // Ports
  DynTherM.CustomInterfaces.IrradiancePort irradiancePort annotation (Placement(
        transformation(extent={{-100,-10},{-80,10}}), iconTransformation(extent
          ={{-100,-10},{-80,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPortInner
    annotation (Placement(transformation(extent={{80,20},{100,40}}),
        iconTransformation(extent={{80,20},{100,40}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a thermalPortToSeats if include_window
    annotation (Placement(transformation(extent={{80,-56},{100,-36}}),
        iconTransformation(extent={{80,-40},{100,-20}})));
initial equation
  if not use_fus_radiation then
    A_fus_rad = 0;
  else
    A_fus_rad = A_fus;
  end if;
equation
  if Nw_side <> 0 then
     connect(windowRadiation.outlet, irradiancePort) annotation (Line(points={{-61.4,
          -50},{-76,-50},{-76,0},{-90,0}},   color={191,0,0}));
  end if;
  connect(wallRadiation.outlet, irradiancePort) annotation (Line(points={{-61.4,
          50},{-76,50},{-76,0},{-90,0}},                                                       color={191,0,0}));
  connect(thermalPortInner, intConvection.inlet)
    annotation (Line(points={{90,30},{72,30},{72,50},{47.4,50}},
                                                 color={191,0,0}));
   connect(extConvection.inlet, wallRadiation.inlet) annotation (Line(points={{-56.6,
          70},{-32,70},{-32,50},{-56.6,50}}, color={191,0,0}));
  connect(windowRadiation.inlet_transmitted, thermalPortToSeats) annotation (
      Line(points={{-56.6,-52.6},{90,-52.6},{90,-46}},color={191,0,0}));
  connect(thermalPortToSeats, thermalPortToSeats)
    annotation (Line(points={{90,-46},{90,-46}},
                                               color={191,0,0}));
  connect(extConvectionWindow.inlet, windowConduction.outlet)
    annotation (Line(points={{-56.6,-30},{-21.4,-30}},
                                                     color={191,0,0}));
  connect(windowRadiation.inlet_absorbed, windowConduction.outlet) annotation (
      Line(points={{-56.6,-47.4},{-21.4,-47.4},{-21.4,-30}},
                                                          color={191,0,0}));
  connect(windowConduction.inlet, intConvectionWindow.outlet)
    annotation (Line(points={{-14.6,-30},{40.6,-30}},
                                                    color={191,0,0}));
  connect(intConvectionWindow.inlet, intConvection.inlet) annotation (Line(
        points={{47.4,-30},{72,-30},{72,50},{47.4,50}},
                                                      color={191,0,0}));
  connect(sandwichPanel.int, intConvection.outlet)
    annotation (Line(points={{-7,50},{40.6,50}}, color={191,0,0}));
  connect(wallRadiation.inlet, sandwichPanel.ext)
    annotation (Line(points={{-56.6,50},{-21,50}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-80,60},{80,-60}},
          lineColor={28,108,200},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid), Text(
          extent={{-54,44},{58,-38}},
          lineColor={28,108,200},
          fillColor={135,135,135},
          fillPattern=FillPattern.None,
          textString="PANEL")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuselagePanel;
