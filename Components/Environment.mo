within DynTherM.Components;
model Environment "Environmental properties (moist air)"

  package Medium = Modelica.Media.Air.MoistAir;

  // Climatic data
  parameter Real ISA_plus=0 "Temperature difference with respect to ISA standard value at given altitude";
  parameter Real phi_amb=0.5 "Ambient relative humidity";
  parameter Real phi_amb_ground=0.5 "Ambient relative humidity on ground";
  parameter Temperature T_ground=303.15 "Temperature of the ground";
  parameter Length altitude_di=0 "Fixed value of altitude with respect to sea level" annotation (Dialog(enable=use_di_altitude));
  parameter Pressure P_amb_di=101325 "Fixed value of ambient pressure" annotation (Dialog(enable=use_P_amb));
  parameter Temperature T_amb_di=288.15 "Fixed value of ambient temperature" annotation (Dialog(enable=use_T_amb));
  parameter Real Mach_inf_di=0 "Fixed value of free-stream Mach number" annotation (Dialog(enable=use_di_Mach_inf));
  parameter Velocity V_inf_di=0 "Fixed value of free-stream velocity" annotation (Dialog(enable=use_di_V_inf));


  // Geographical data
  input Angle lat=0.58712876 "Latitude" annotation (Dialog(tab="Geographical location (default: Atlanta, GA, USA)"), enable=true);
  input Angle long=-1.47358149 "Longitude" annotation (Dialog(tab="Geographical location (default: Atlanta, GA, USA)"), enable=true);
  parameter Angle long_ref=-1.30899694
    "Longitude of the time-zone reference meridian" annotation (Dialog(tab=
          "Geographical location (default: Atlanta, GA, USA)"));
  parameter Integer Month=7 "Month of the year (1-12)";
  parameter Integer Day=199 "Day of the year (1-365)";
  parameter Integer Hour=12 "Hour of the day (0-23)";
  parameter Integer Minute=0 "Minutes (0-59)";
  parameter Real tau_b[12]=
    {0.334, 0.324, 0.355, 0.383, 0.379, 0.406, 0.440, 0.427, 0.388, 0.358, 0.354, 0.335} "Beam optical depth" annotation (Dialog(tab="Geographical location (default: Atlanta, GA, USA)"));
  parameter Real tau_d[12]=
    {2.614, 2.580, 2.474, 2.328, 2.324, 2.270, 2.202, 2.269, 2.428, 2.514, 2.523, 2.618} "Beam diffuse depth" annotation (Dialog(tab="Geographical location (default: Atlanta, GA, USA)"));

  // Options
  parameter Boolean use_di_altitude = true "True if altitude is given as parameter" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Boolean use_di_Mach_inf = true "True if free-stream Mach number is given as parameter" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Boolean use_di_V_inf = false "True if free-stream velocity is given as parameter" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Boolean use_in_altitude = false "True if altitude is given as input" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Boolean use_in_Mach_inf = false "True if free-stream Mach number is given as input" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Boolean use_in_V_inf = false "True if free-stream velocity is given as input" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));

  parameter Boolean use_P_amb = false "Use fixed value for the ambient pressure" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Boolean use_T_amb = false "Use fixed value for the ambient temperature" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Angle psi=1.0471975511965976
    "Azimuth angle wrt south - fuselage main axis";
  parameter Boolean use_ext_sw=false
    "Bypass solar radiation calculations and get E_dir, E_diff, E_refl, theta from an external software" annotation (Dialog(tab="Simulation options"), choices(checkBox=true));
  parameter Boolean allowFlowReversal=true
    "= false to restrict to design flow direction (inlet -> outlet)" annotation (Dialog(tab="Simulation options"));
  parameter DynTherM.Choices.InitOpt initOpt=DynTherM.Choices.InitOpt.steadyState "Initialization type" annotation (Dialog(tab="Simulation options"));

  // Constants
  constant Real sigma( final quantity="Stefan-Boltzmann constant", final unit="W/(m2.K4)")=5.67e-8;
  constant Acceleration g=9.80665 "Gravitational acceleration";
  constant Real n_air=1.000293 "Air refractive index";
  constant Real pi=Modelica.Constants.pi;
  constant Real R=287.058 "Specific gas constant";
  constant Pressure P0=101325 "Ambient pressure at sea-level [Pa]";
  constant Temperature T0=288.15 "Ambient temperature at sea-level [K]";
  constant Area S_hb=1.75 "Average human body surface area";
  constant DensityOfHeatFlowRate M_hb[3]={60,115,70}
    "Rate of metabolic heat production of: passengers, cabin crew, pilots";

  Length altitude "Altitude with respect to sea level";
  Velocity V_inf "Free-stream velocity";
  Real Mach_inf "Free-stream Mach number";
  Pressure Pv "Water vapour pressure";
  Emissivity eps_sky "Clear sky emmisivity";
  Temperature T_sky "Sky temperature";
  Temperature T_ISA "Ambient temperature - ISA conditions";
  Temperature T_amb "Ambient temperature";
  Temperature T_ground_corr "Temperature of the ground corrected for altitude";
  Pressure P_amb "Ambient pressure";
  Pressure P_cab_target "Target cabin pressure - fixed by standard CFR 25.841";
  MassFraction X_water "Water content in ambient air";
  MassFraction X_air "Dry air content in ambient air";
  MassFraction X_amb[2] "Ambient air mass fractions";
  Medium.ThermodynamicState state_amb "Ambient thermodynamic state";

  Modelica.Blocks.Interfaces.RealInput in_V_inf if use_in_V_inf
    "Free-stream velocity - input connector" annotation (Placement(transformation(
        origin={-100,-60},
        extent={{20,-20},{-20,20}},
        rotation=180), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-100,-80})));
  Modelica.Blocks.Interfaces.RealInput in_altitude if use_in_altitude
    "Altitude with respect to sea level - input connector" annotation (Placement(transformation(
        origin={-100,-40},
        extent={{20,-20},{-20,20}},
        rotation=180), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-100,-40})));
  Modelica.Blocks.Interfaces.RealInput in_Mach_inf if use_in_Mach_inf
    "Free-stream Mach number - input connector" annotation (Placement(transformation(
        origin={-100,-20},
        extent={{20,-20},{-20,20}},
        rotation=180), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-100,0})));

protected
  Pressure Pv_ground "Water vapour pressure on ground";
  Emissivity eps_sky_ground "Clear sky emmisivity on ground";
  Temperature T_sky_ground "Sky temperature on ground";
  Modelica.Blocks.Interfaces.RealInput V_inf_internal;
  Modelica.Blocks.Interfaces.RealInput Mach_inf_internal;
  Modelica.Blocks.Interfaces.RealInput altitude_internal;

equation
  state_amb = Medium.setState_pTX(P_amb, T_amb, X_amb);
  X_water = Medium.massFraction_pTphi(P_amb, T_amb, phi_amb);
  X_air = 1 - X_water;
  X_amb = {X_water, X_air};
  Mach_inf = V_inf/Medium.velocityOfSound(state_amb);

  //Boundary equations
  if use_di_altitude then
    altitude_di = altitude;
  elseif use_in_altitude then
    altitude_internal = altitude;
  end if;

  if use_di_Mach_inf then
    Mach_inf_di = Mach_inf;
  elseif use_in_Mach_inf then
    Mach_inf_internal = Mach_inf;
  end if;

  if use_di_V_inf then
    V_inf_di = V_inf;
  elseif use_in_V_inf then
    V_inf_internal = V_inf;
  end if;

  // Compute ambient temperature and pressure [1]
  if (altitude <= 11000) then
    T_ISA = T0 - altitude/1000*6.5;

    if use_P_amb then
      P_amb = P_amb_di;
    else
      P_amb = P0*(T_ISA/T0)^(g*1000/(R*6.5));
    end if;

  elseif (altitude > 11000) and (altitude <= 20000) then
    T_ISA = T0 - 11*6.5;

    if use_P_amb then
      P_amb = P_amb_di;
    else
      P_amb = P0*(T_ISA/T0)^(g*1000/(R*6.5))*
        Modelica.Math.exp(-g/(R*T_ISA)*(altitude - 11000));
    end if;

  else

    T_amb = T0;
    P_amb = P0;
    assert(false, "altitude exceeding 20km is unsupported");
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
  if (altitude > 0) then
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
  connect(in_altitude, altitude_internal);
  connect(in_Mach_inf, Mach_inf_internal);
  connect(in_V_inf, V_inf_internal);

  annotation (
    defaultComponentName="environment",
    defaultComponentPrefixes="inner",
    missingInnerMessage="The Environment object is missing, please drag it on the top layer of your model",
    Icon(graphics={
        Ellipse(
          extent={{-10,42},{38,-6}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Sphere),
        Rectangle(
          extent={{12,-10},{16,-30}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{12,66},{16,46}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,10},{2,-10}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-24,18},
          rotation=-90),
        Rectangle(
          extent={{-2,10},{2,-10}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          rotation=-90,
          origin={52,18}),
        Polygon(
          points={{-24,50},{-10,36},{-6,40},{-24,50}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-9,7},{5,-7},{9,-3},{-9,7}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={41,43},
          rotation=-90),
        Polygon(
          points={{-9,-7},{5,7},{9,3},{-9,-7}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={-13,-7},
          rotation=360),
        Polygon(
          points={{9,7},{-5,-7},{-9,-3},{9,7}},
          lineColor={238,46,47},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={39,-7},
          rotation=-90),
        Ellipse(
          extent={{-90,14},{8,-16}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-64,6},{-22,-26}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-42,10},{0,-22}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-84,12},{-42,-20}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-66,22},{-2,-8}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-58,32},{-16,0}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-74,22},{-32,-10}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0},
          lineThickness=1),
        Ellipse(
          extent={{-18,-14},{80,-44}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{8,-22},{50,-54}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{30,-18},{72,-50}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-12,-16},{30,-48}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{6,-6},{70,-36}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{14,4},{56,-28}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={95,95,95}),
        Ellipse(
          extent={{-2,-6},{40,-38}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0},
          lineThickness=1)}),
    Documentation(info="<html>
<p>References:</p>
<p>[1]&nbsp;ISA&nbsp;model&nbsp;to&nbsp;compute&nbsp;variation&nbsp;of&nbsp;properties&nbsp;with&nbsp;altitude</p>
<p>[2]&nbsp;X.&nbsp;Fang.&nbsp;Study&nbsp;on&nbsp;saturated&nbsp;water&nbsp;vapor&nbsp;pressure&nbsp;equations&nbsp;for&nbsp;calculation&nbsp;of&nbsp;aircraft&nbsp;air-conditioning&nbsp;systems,&nbsp;1995.</p>
<p>[3]&nbsp;Q.&nbsp;Dai,&nbsp;X.&nbsp;Fang.&nbsp;A&nbsp;new&nbsp;model&nbsp;for&nbsp;atmospheric&nbsp;radiation&nbsp;under&nbsp;clear&nbsp;sky&nbsp;condition&nbsp;at&nbsp;various&nbsp;altitudes,&nbsp;2014.</p>
<p>[4]&nbsp;Earth&nbsp;System&nbsp;Research&nbsp;Laboratory&nbsp;(NOAA)&nbsp;https://psl.noaa.gov/cgi-bin/data/timeseries/timeseries1.pl</p>
</html>"));
end Environment;
