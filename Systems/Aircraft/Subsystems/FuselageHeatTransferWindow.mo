within DynTherM.Systems.Aircraft.Subsystems;
model FuselageHeatTransferWindow
  "Model of heat transfer from external environment to inner section of upper fuselage (with transparencies)"
  outer DynTherM.Components.Environment environment "Environmental properties";

  replaceable model HTC_int =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    annotation (choicesAllMatching=true);

  replaceable model HTC_ext =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal
    annotation (choicesAllMatching=true);

  parameter Real coeff "Fraction of cylinder with active heat transfer";
  parameter Modelica.Units.SI.Length L_fuselage
    "Length of the fuselage cylindrical section";
  parameter Modelica.Units.SI.Length R_ext "External radius of the fuselage";
  parameter Modelica.Units.SI.Length t_fuselage
    "Overall fuselage thickness (half for lower section)";
  parameter Modelica.Units.SI.Angle csi
    "Tilt angle of the surface wrt horizontal";
  parameter Modelica.Units.SI.Irradiance E_tb "Beam component of the clear-sky solar irradiance";
  parameter Modelica.Units.SI.Irradiance E_td "Diffuse component of the clear-sky solar irradiance";
  parameter Modelica.Units.SI.Irradiance E_tr "Ground reflected component of the clear-sky solar irradiance";
  parameter Modelica.Units.SI.Angle theta "Incidence angle";
  parameter Modelica.Units.SI.Length L_window "Window length";
  parameter Modelica.Units.SI.Length H_window "Window height";

  parameter Integer Nw_side=0 "Number of windows per fuselage side";
  parameter Modelica.Units.SI.Temperature Tstart_fuselage
    "Fuselage temperature start value" annotation (Dialog(tab="Initialization"));

  final parameter Modelica.Units.SI.Length R_int=composite.interiorPanel_int.R_int
    "Internal radius of the fuselage";
  final parameter Modelica.Units.SI.Area A_tot_window=L_window*H_window*Nw_side
    "Total window area";
  final parameter Modelica.Units.SI.Area A_int=coeff*2*environment.pi*
    L_fuselage*R_int - A_tot_window "Internal fuselage area (minus windows)";
  final parameter Modelica.Units.SI.Area A_ext=coeff*2*environment.pi*
    L_fuselage*R_ext - A_tot_window "External fuselage area (minus windows)";

  input Modelica.Units.SI.Pressure P_air_gap "Average pressure inside the air cavity";
  input Modelica.Units.SI.MassFraction X_air_gap[2] "Composition of the air cavity";

  Components.HeatTransfer.ExternalConvection extConvection(A=A_ext,
    redeclare model HTC = HTC_ext)
    annotation (Placement(transformation(extent={{46,56},{74,28}})));
  Components.HeatTransfer.WallRadiation wallRadiation(
    A=A_ext,
    csi=csi)
    annotation (Placement(transformation(extent={{6,56},{34,28}})));
  Components.HeatTransfer.SolarRadiation solarRadiation(
    E_tb_fixed=E_tb,
    E_td_fixed=E_td,
    E_tr_fixed=E_tr,
    theta_fixed=theta,
    csi=csi)
    annotation (Placement(transformation(extent={{-40,92},{0,52}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatAbsorbed annotation (
      Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{20,-70},{40,-50}})));
  Components.HeatTransfer.InternalConvection intConvection(A=A_int,
   redeclare model HTC = HTC_int)
    annotation (Placement(transformation(extent={{4,-24},{36,-56}})));

  FuselageSandwichStructure composite(
    coeff=coeff,
    L_fuselage=L_fuselage,
    R_ext=R_ext,
    t_tot=t_fuselage,
    L_window=L_window,
    H_window=H_window,
    Nw_side=Nw_side,
    Tstart=Tstart_fuselage)
    annotation (Placement(transformation(extent={{14,-18},{66,20}})));
  Components.HeatTransfer.ExternalConvection extConvectionWindow(A=A_tot_window,
      redeclare model HTC = HTC_ext)
    annotation (Placement(transformation(extent={{-42,56},{-14,28}})));
  Components.HeatTransfer.InternalConvection intConvectionWindow(A=A_tot_window,
      redeclare model HTC = HTC_int)
    annotation (Placement(transformation(extent={{-36,-24},{-4,-56}})));
  CabinWindow cabinWindow(
    H_window=H_window,
    L_window=L_window*Nw_side,
    Tstart=Tstart_fuselage)
    annotation (Placement(transformation(extent={{-76,-28},{-22,18}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatTransmitted
    annotation (Placement(transformation(extent={{-64,-110},{-44,-90}}),
        iconTransformation(extent={{-40,-70},{-20,-50}})));
equation
  cabinWindow.P_air = P_air_gap;
  cabinWindow.X_air = X_air_gap;

  connect(solarRadiation.inlet, wallRadiation.outlet)
    annotation (Line(points={{-20,59.6},{-20,60},{20,60},{20,43.96}},
                                                       color={191,0,0}));
  connect(heatAbsorbed, intConvection.inlet) annotation (Line(points={{0,-100},
          {20,-100},{20,-45.44}}, color={191,0,0}));
  connect(wallRadiation.inlet, composite.ext) annotation (Line(points={{20,37.24},
          {20,14.3},{40,14.3}}, color={191,0,0}));
  connect(extConvection.inlet, composite.ext)
    annotation (Line(points={{60,37.24},{60,14.3},{40,14.3}},color={191,0,0}));
  connect(composite.int, intConvection.outlet)
    annotation (Line(points={{40,-12.3},{40,-34.56},{20,-34.56}},
                                                    color={191,0,0}));
  connect(intConvectionWindow.inlet, heatAbsorbed) annotation (Line(points={{-20,
          -45.44},{-20,-100},{0,-100}}, color={191,0,0}));
  connect(cabinWindow.heatAbsorbed, intConvectionWindow.outlet) annotation (
      Line(points={{-43.6,-10.1111},{-43.6,-34},{-20,-34},{-20,-34.56}}, color=
          {191,0,0}));
  connect(extConvectionWindow.inlet, cabinWindow.heatExt) annotation (Line(
        points={{-28,37.24},{-28,20},{-43.6,20},{-43.6,10.3333}}, color={191,0,0}));
  connect(solarRadiation.inlet, cabinWindow.irradianceExt) annotation (Line(
        points={{-20,59.6},{-54,59.6},{-54,10.3333},{-54.4,10.3333}}, color={191,
          0,0}));
  connect(heatTransmitted, cabinWindow.heatTransmitted) annotation (Line(points=
         {{-54,-100},{-54.4,-100},{-54.4,-10.1111}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-100,-180},{100,20}},
          lineColor={0,0,0},
          startAngle=0,
          endAngle=180,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Ellipse(
          extent={{-80,-160},{80,0}},
          lineColor={0,0,0},
          startAngle=0,
          endAngle=180,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Line(
          points={{-100,-80},{-80,-80}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{80,-80},{100,-80}},
          color={0,0,0},
          thickness=0.5),
        Ellipse(
          extent={{-70,-150},{68,-12}},
          lineColor={28,108,200},
          startAngle=0,
          endAngle=80,
          closure=EllipseClosure.None,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,-150},{-68,-12}},
          lineColor={28,108,200},
          startAngle=0,
          endAngle=80,
          closure=EllipseClosure.None,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{-10,-12},{-16,-18}}, color={28,108,200}),
        Line(points={{-10,-12},{-18,-10}}, color={28,108,200}),
        Line(points={{10,-12},{16,-18}}, color={28,108,200}),
        Line(points={{10,-12},{18,-10}}, color={28,108,200}),
        Line(points={{-20,24},{-20,80}},  color={238,46,47}),
        Line(points={{-14,34},{-20,24}},  color={238,46,47}),
        Line(points={{-26,34},{-20,24}},  color={238,46,47}),
        Line(points={{-40,18},{-40,74}},  color={238,46,47}),
        Line(points={{-34,28},{-40,18}},  color={238,46,47}),
        Line(points={{-46,28},{-40,18}},  color={238,46,47}),
        Line(points={{20,24},{20,80}},    color={238,46,47}),
        Line(points={{26,34},{20,24}},    color={238,46,47}),
        Line(points={{14,34},{20,24}},    color={238,46,47}),
        Line(points={{40,18},{40,74}},    color={238,46,47}),
        Line(points={{46,28},{40,18}},    color={238,46,47}),
        Line(points={{34,28},{40,18}},    color={238,46,47}),
        Ellipse(
          extent={{-100,-180},{100,20}},
          lineColor={0,255,255},
          startAngle=30,
          endAngle=60,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Ellipse(
          extent={{100,-180},{-100,20}},
          lineColor={0,255,255},
          startAngle=30,
          endAngle=60,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Ellipse(
          extent={{-68,-148},{78,-2}},
          lineColor={0,255,255},
          startAngle=30,
          endAngle=60,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Ellipse(
          extent={{68,-148},{-78,-2}},
          lineColor={0,255,255},
          startAngle=30,
          endAngle=60,
          closure=EllipseClosure.None,
          lineThickness=0.5),
        Line(points={{50,-30},{84,4}},    color={238,46,47}),
        Line(points={{-50,-30},{-84,4}},  color={238,46,47}),
        Line(points={{-50,-20},{-50,-30}},color={238,46,47}),
        Line(points={{-50,-30},{-60,-30}},color={238,46,47}),
        Line(points={{60,-30},{50,-30}},  color={238,46,47}),
        Line(points={{50,-20},{50,-30}},  color={238,46,47}),
        Text(
          extent={{-30,-16},{28,-46}},
          lineColor={0,0,0},
          lineThickness=0.5,
          textString="FUSELAGE")}),                              Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuselageHeatTransferWindow;
