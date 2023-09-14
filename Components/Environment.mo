within DynTherM.Components;
model Environment "Environmental properties (moist air)"
  // References:
  // [1] ISA model to compute variation of properties with altitude
  // [2] X. Fang. Study on saturated water vapor pressure equations for calculation of aircraft air-conditioning systems, 1995.
  // [3] Q. Dai, X. Fang. A new model for atmospheric radiation under clear sky condition at various altitudes, 2014.
  // [4] Earth System Research Laboratory (NOAA) https://psl.noaa.gov/cgi-bin/data/timeseries/timeseries1.pl

  package Medium = Modelica.Media.Air.MoistAir;

  parameter Real Mach_inf=0 "Aircraft/rotorcraft Mach number";
  parameter Modelica.Units.SI.Length Altitude=0
    "Aircraft altitude with respect to sea level";
  parameter Real ISA_plus=0 "Temperature difference with respect to ISA standard value at given altitude";
  parameter Real phi_amb=0.5 "Ambient relative humidity";
  parameter Real phi_amb_ground=0.5 "Ambient relative humidity on ground";
  parameter Modelica.Units.SI.Temperature T_ground=303.15
    "Temperature of the ground";
  parameter Modelica.Units.SI.Angle lat=0.58712876 "Latitude" annotation (
      Dialog(tab="Geographical location (default: Atlanta, GA, USA)"));
  parameter Modelica.Units.SI.Angle long=-1.47358149 "Longitude" annotation (
      Dialog(tab="Geographical location (default: Atlanta, GA, USA)"));
  parameter Modelica.Units.SI.Angle long_ref=-1.30899694
    "Longitude of the time-zone reference meridian" annotation (Dialog(tab=
          "Geographical location (default: Atlanta, GA, USA)"));
  parameter Integer Month=7 "Month of the year (1-12)";
  parameter Integer Day=199 "Day of the year (1-365)";
  parameter Integer Hour=12 "Hour of the day (0-23)";
  parameter Integer Minute=0 "Minutes (0-59)";
  parameter Modelica.Units.SI.Pressure P_amb_di=101325 "Fixed value of ambient pressure" annotation (Dialog(enable=use_P_amb));
  parameter Modelica.Units.SI.Temperature T_amb_di=288.15 "Fixed value of ambient temperature" annotation (Dialog(enable=use_T_amb));
  parameter Real tau_b[12]=
    {0.334, 0.324, 0.355, 0.383, 0.379, 0.406, 0.440, 0.427, 0.388, 0.358, 0.354, 0.335} "Beam optical depth" annotation (Dialog(tab="Geographical location (default: Atlanta, GA, USA)"));
  parameter Real tau_d[12]=
    {2.614, 2.580, 2.474, 2.328, 2.324, 2.270, 2.202, 2.269, 2.428, 2.514, 2.523, 2.618} "Beam diffuse depth" annotation (Dialog(tab="Geographical location (default: Atlanta, GA, USA)"));
  parameter Boolean use_P_amb = false "Use fixed value for the ambient pressure" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_T_amb = false "Use fixed value for the ambient temperature" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_Long = false "Use connector input for the longitude" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean use_in_Lat = false "Use connector input for the latitude" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Modelica.Units.SI.Angle psi=1.0471975511965976
    "Azimuth angle wrt south - fuselage main axis";
  parameter Boolean use_ext_sw=false
    "Bypass solar radiation calculations and get E_dir, E_diff, E_refl, theta from an external sw" annotation (Dialog(group="External inputs"), choices(checkBox=true));
  parameter Boolean allowFlowReversal=true
    "= false to restrict to design flow direction (inlet -> outlet)" annotation (Dialog(tab="Simulation options"));
  parameter DynTherM.Choices.InitOpt initOpt=DynTherM.Choices.InitOpt.steadyState
    annotation (Dialog(tab="Simulation options"));
  constant Real sigma( final quantity="Stefan-Boltzmann constant", final unit="W/(m2.K4)")=5.67e-8;
  constant Modelica.Units.SI.Acceleration g=9.80665
    "Gravitational acceleration";
  constant Real n_air=1.000293 "Air refractive index";
  constant Real pi=Modelica.Constants.pi;

  final parameter Real R=287.058 "Specific gas constant";
  final parameter Modelica.Units.SI.Pressure P0=101325
    "Ambient pressure at sea-level [Pa]";
  final parameter Modelica.Units.SI.Temperature T0=288.15
    "Ambient temperature at sea-level [K]";
  final parameter Modelica.Units.SI.Area S_hb=1.75
    "Average human body surface area";
  final parameter Modelica.Units.SI.DensityOfHeatFlowRate M_hb[3]={60,115,70}
    "Rate of metabolic heat production of: passengers, cabin crew, pilots";

  Modelica.Units.SI.Pressure Pv "Water vapour pressure";
  Modelica.Units.SI.Emissivity eps_sky "Clear sky emmisivity";
  Modelica.Units.SI.Temperature T_sky "Sky temperature";
  Modelica.Units.SI.Temperature T_ISA "Ambient temperature - ISA conditions";
  Modelica.Units.SI.Temperature T_amb "Ambient temperature";
  Modelica.Units.SI.Temperature T_ground_corr
    "Temperature of the ground corrected for altitude";
  Modelica.Units.SI.Pressure P_amb "Ambient pressure";
  Modelica.Units.SI.Pressure P_cab_target
    "Target cabin pressure - fixed by standard CFR 25.841";
  Modelica.Units.SI.MassFraction X_water "Water content in ambient air";
  Modelica.Units.SI.MassFraction X_air "Dry air content in ambient air";
  Modelica.Units.SI.MassFraction X_amb[2] "Ambient air mass fractions";
  Medium.ThermodynamicState state_amb "Ambient thermodynamic state";

  Modelica.Blocks.Interfaces.RealInput in_Long if use_in_Long annotation (Placement(
        transformation(
        origin={-60,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-17,-17},{17,17}},
        rotation=0,
        origin={-99,-21})));
  Modelica.Blocks.Interfaces.RealInput in_Lat if use_in_Lat annotation (Placement(
        transformation(
        origin={0,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-100,-80})));

protected
  Modelica.Blocks.Interfaces.RealInput in_Long0 annotation (Placement(
        transformation(
        origin={-40,64},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,60})));
  Modelica.Blocks.Interfaces.RealInput in_Lat0 annotation (Placement(
        transformation(
        origin={-20,90},
        extent={{-10,-10},{10,10}},
        rotation=270), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));

protected
  Modelica.Units.SI.Pressure Pv_ground "Water vapour pressure on ground";
  Modelica.Units.SI.Emissivity eps_sky_ground "Clear sky emmisivity on ground";
  Modelica.Units.SI.Temperature T_sky_ground "Sky temperature on ground";

equation
  state_amb = Medium.setState_pTX(P_amb, T_amb, X_amb);
  X_water = Medium.massFraction_pTphi(P_amb, T_amb, phi_amb);
  X_air = 1 - X_water;
  X_amb = {X_water, X_air};

  // Compute ambient temperature and pressure [1]
  if (Altitude <= 11000) then
    T_ISA = T0 - Altitude/1000*6.5;

    if use_P_amb then
      P_amb = P_amb_di;
    else
      P_amb = P0*(T_ISA/T0)^(g*1000/(R*6.5));
    end if;

  elseif (Altitude > 11000) and (Altitude <= 20000) then
    T_ISA = T0 - 11*6.5;

    if use_P_amb then
      P_amb = P_amb_di;
    else
      P_amb = P0*(T_ISA/T0)^(g*1000/(R*6.5))*
        Modelica.Math.exp(-g/(R*T_ISA)*(Altitude - 11000));
    end if;

  else
    assert(false, "Altitude exceeding 20km is unsupported");
  end if;

  if use_T_amb then
    T_amb = T_amb_di;
  else
    T_amb = T_ISA + ISA_plus;
  end if;

  // Compute vapor pressure [2]
  // At altitude
  if T_amb < 273.15 then
    Pv = phi_amb * exp(29.06 - 6211.88/(T_amb - 273.15 + 274.35));
  else
    Pv = phi_amb * exp(23.3 - 3890.94/(T_amb - 273.15 + 230.4));
  end if;

  // On ground (T_ground correction)
  if (T0 + ISA_plus) < 273.15 then
    Pv_ground = phi_amb_ground * exp(29.06 - 6211.88/((T0 + ISA_plus) - 273.15 + 274.35));
  else
    Pv_ground = phi_amb_ground * exp(23.3 - 3890.94/((T0 + ISA_plus) - 273.15 + 230.4));
  end if;

  // Compute clear sky temperature and emissivity [3]
  // At altitude
  eps_sky = (0.48 + 0.17 * (Pv/100)^0.22) * (P_amb/P0)^0.45;
  eps_sky = (T_sky/T_amb)^4;

  // On ground (T_ground correction)
  eps_sky_ground = (0.48 + 0.17 * (Pv_ground/100)^0.22);
  eps_sky_ground = (T_sky_ground/(T0 + ISA_plus))^4;

  // T_ground correction: column of air between aircraft and ground increases with altitude
  if (Altitude > 0) then
    T_ground_corr = T_ground - (T_sky_ground - T_sky);
  else
    T_ground_corr = T_ground;
  end if;

  // Compute target cabin pressure
  if (P_amb + 50000) < 76200 then
    P_cab_target = 76200;          // Minimum allowed cabin pressure
  elseif (P_amb + 50000) > P0 then
    P_cab_target = P0 + 1000;      // Maximum allowed cabin pressure (on ground)
  else
    P_cab_target = P_amb + 50000;  // 50 kPa of pressurization during normal operation
  end if;

  // Connect protected connectors to public conditional connectors
  in_Long0 = long;
  in_Lat0 = lat;
  connect(in_Long, in_Long0);
  connect(in_Lat, in_Lat0);

  annotation (
    defaultComponentName="environment",
    defaultComponentPrefixes="inner",
    missingInnerMessage="The Environment object is missing, please drag it on the top layer of your model",
    Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-68,70},{-20,22}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-46,18},{-42,-2}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-46,94},{-42,74}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,10},{2,-10}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-82,46},
          rotation=-90),
        Rectangle(
          extent={{-2,10},{2,-10}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-6,46},
          rotation=-90),
        Polygon(
          points={{-82,78},{-68,64},{-64,68},{-82,78}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-9,7},{5,-7},{9,-3},{-9,7}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-17,71},
          rotation=-90),
        Polygon(
          points={{-9,-7},{5,7},{9,3},{-9,-7}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-71,21},
          rotation=360),
        Polygon(
          points={{9,7},{-5,-7},{-9,-3},{9,7}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-19,21},
          rotation=-90),
        Ellipse(
          extent={{-52,-34},{46,-64}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-26,-42},{16,-74}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-4,-38},{38,-70}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-46,-36},{-4,-68}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-28,-26},{36,-56}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-20,-16},{22,-48}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-36,-26},{6,-58}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end Environment;
