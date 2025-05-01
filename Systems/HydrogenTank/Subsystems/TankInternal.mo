within DynTherM.Systems.HydrogenTank.Subsystems;
model TankInternal "Internal part of the tank (no heat transfer through walls)"

  replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
    ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);

  // Options and constants
  parameter Boolean allowFlowReversal
    "= true to allow flow reversal, false restricts to design direction" annotation(Evaluate=true);
  parameter Real Ff "Tank Fill fraction";
  parameter Time tauev = 1e1 "Time constant of bulk evaporation";
  parameter Time tauc = 1e1 "Time constant of bulk condensation";

  //parameter Real Kcs = 0 "Surface condensation coefficient [kg/(s.m2.K)]";
  //parameter Real Kes = 0 "Surface evaporation coefficient [kg/(s.m2.K)]";
  //parameter Real Kvs = 0 "Surface heat transfer coefficient (vapor-surface) [W/(m2.K)]";
  //parameter Real Kls = 0 "Surface heat transfer coefficient (liquid-surface) [W/(m2.K)]";

  constant Acceleration g=Modelica.Constants.g_n;
  constant Real pi=Modelica.Constants.pi;

  // Initialization
  parameter Choices.InitOpt initOpt "Initialisation option" annotation (Dialog(tab="Initialization"));
  parameter Boolean noInitialPressure=false
    "Remove initial equation on pressure" annotation (Dialog(tab="Initialization"),choices(checkBox=true));
  parameter Pressure P_start "Pressure - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.SpecificEnthalpy hl_start=Medium.bubbleEnthalpy(Medium.setSat_p(P_start))
    "Enthalpy of liquid hydrogen - start value" annotation (Dialog(tab="Initialization"));
  parameter Medium.SpecificEnthalpy hv_start=Medium.dewEnthalpy(Medium.setSat_p(P_start))
    "Enthalpy of gaseous hydrogen - start value" annotation (Dialog(tab="Initialization"));
  parameter Length y_start = 0.808*Ff*R_int
    "Liquid level referred to the centreline - start value" annotation (Dialog(tab="Initialization"));
  parameter Volume Vl_start = Vt*Ff "Liquid fluid volume";

  // Geometry
  parameter Length L "Length" annotation (Dialog(tab="Geometry"));
  parameter Length R_int "Internal radius" annotation (Dialog(tab="Geometry"));
  parameter Real AR = 1 "Tank end-dome aspect ratio (h/R)" annotation (Dialog(tab="Geometry"));

  final parameter Length R_eq = R_int*(1 + AR^2)/(2*AR)
    "Radius of equivalent sphere when dome AR < 1";
  final parameter Length L_star = R_eq - AR*R_int
    "Length of aproximated cylinder which is cut of from a half sphere when dome AR < 1";
  final parameter Length L_star_loc = L_star/2
    "Horizontal location of R_star (middle of the cylinder)";
  final parameter Length R_star = R_eq*cos(asin(L_star_loc/R_eq))
    "Radius of aproximated cylinder which is cut of from a half sphere when dome AR < 1";
  final parameter Integer n_star = 1 "Number of equivalent sections of volume cylinders";
  final parameter Length dR = R_eq - R_int
    "Difference between radius of dome and equivalents sphere";
  final parameter Volume Vt = pi*R_int^2*L +
    2*pi*(R_int*AR)^2*(R_eq - (R_int*AR)/3) "Internal volume";
  final parameter Area Awt_tot = 2*pi*R_int*L + 2*pi*R_eq^2*AR "Internal area";

  Real A_frac "Liquid area fraction";
  Length y(start = y_start) "Liquid level referred to the centreline";
  Length y0 "Liquid level referred to the bottom";
  Length dh_outer "Height of wall segments 0 and 4 (top and bottom)";
  Length dh_between "Height of wall segments 1, 3, 5 and 7";
  Length dh_inner "Height of wall segments 2 and 6 (central sections)";

  // Thermodynamics States
  Medium.SaturationProperties sat "Saturated state";
  Medium.ThermodynamicState liq "Thermodynamic state of the liquid";
  Medium.ThermodynamicState vap "Thermodynamic state of the vapor";

  // Mass flow rates
  MassFlowRate ql "Liquid mass flow rate";
  MassFlowRate qv "Gaseous mass flow rate";
  MassFlowRate wc "Mass flow rate of bulk condensation";
  MassFlowRate wev "Mass flow rate of bulk evaporation";
  MassFlowRate ws "Mass flow rate of surface evaporation/condensation
    (positive for evaporation, negative for condensation)";
  //MassFlowRate wcs "Mass flow rate of surface condensation";
  //MassFlowRate wes "Mass flow rate of surface evaporation";

  // Fluid Properties
  Mass Mv "Mass of vapor";
  Mass Ml "Mass of liquid";
  Volume Vv(start = Vt - Vl_start) "Volume of vapor";
  Volume Vl(start = Vl_start, stateSelect=StateSelect.default) "Volume of liquid";
  Energy El "Liquid internal energy";
  Energy Ev "Vapour internal energy";
  PerUnit xl "Mass fraction of vapor in the liquid volume";
  PerUnit xv "Steam quality in the vapor volume";
  Medium.AbsolutePressure P(start=P_start, stateSelect=StateSelect.prefer)
    "Surface pressure";
  SpecificEnthalpy hls "Specific enthalpy of saturated liquid";
  SpecificEnthalpy hvs "Specific enthalpy of saturated vapor";
  SpecificEnthalpy Dh "Latent Heat of Evaporation at boiling point";
  Medium.SpecificEnthalpy hl(start=hl_start, stateSelect=StateSelect.prefer)
    "Specific enthalpy of liquid";
  Medium.SpecificEnthalpy hv(start=hv_start, stateSelect=StateSelect.prefer)
    "Specific enthalpy of vapor";
  Medium.Temperature Ts "Saturation temperature";
  Medium.Temperature Tl "Liquid temperature";
  Medium.Temperature Tv "Vapour temperature";
  Density rhol "Liquid density";
  Density rhov "Vapour density";
  Medium.SpecificInternalEnergy ul "Liquid specific internal energy";
  Medium.SpecificInternalEnergy uv "Vapour specific internal energy";

  // Wall temperatures
  Temperature Tw[8] "Wall temperature";
  Temperature Tw_dome[2] "Wall temperature of the dome";

  // Heat flow rates
  Power Q_tot "Total heat flow from wall to inner";
  Power Qwv_tot "Heat flow from the wall to the vapor";
  Power Qwl_tot "Heat flow from the wall to the liquid";
  Power Ql_tot "Total heat flow to the liquid";
  Power Qv_tot "Total heat flow to the vapor";
  Power Qwv[8] "Heat flow from the wall to the vapor";
  Power Qwl[8] "Heat flow from the wall to the liquid";
  Power Qwv_dome[2] "Heat flow from the wall to the dome";
  Power Qwl_dome[2] "Heat flow from the wall to the dome";
  Power Qsv "Heat flow from the vapor to the liquid";
  Power Qsl "Heat flow from the liquid to the vapor";
  Power Ql_int "Heat flow introduced into the liquid directly";
  Power Qv_int "Heat flow introduced into the vapor directly";
  Power Q_int "Heat flow introduced into the fluid (liq. and vap.) directly";

  // Wall Interface Surface Areas
  Area Awl_tot "Surface of the wall-liquid interface";
  Area Awv_tot "Surface of the wall-vapor interface";
  Area Awl[8] "Surface of the wall-liquid interface";
  Area Awv[8] "Surface of the wall-vapor interface";
  Area Awl_dome[2] "Surface of the wall-liquid interface at the dome";
  Area Awv_dome[2] "Surface of the wall-vapor interface at the dome";
  Area Asup "Surface of the liquid-vapor interface";
  Area Awt[8] "Surface of the wall";
  Area Awt_dome[2] "Surface of the dome";

  // Heat Transfer Coefficients and Non-Dimensional Numbers
  PrandtlNumber Prl "Prandtl number of the liquid";
  PrandtlNumber Prv "Prandtl number of the vapor";

  NusseltNumber Nul[8] "Nusselt number of the liquid";
  NusseltNumber Nuv[8] "Nusselt number of the vapor";
  NusseltNumber Nul_dome[2] "Nusselt number of the liquid at the dome";
  NusseltNumber Nuv_dome[2] "Nusselt number of the vapor at the dome";

  RayleighNumber Ral[8] "Rayleigh number of the liquid";
  RayleighNumber Rav[8] "Rayleigh number of the vapor";
  RayleighNumber Ral_dome[2] "Rayleigh number of the liquid at the dome";
  RayleighNumber Rav_dome[2] "Rayleigh number of the vapor at wall at the dome";

  CoefficientOfHeatTransfer h_int_l[8] "Internal convective heat transfer of the liquid";
  CoefficientOfHeatTransfer h_int_v[8] "Internal convective heat transfer of the vapor";
  CoefficientOfHeatTransfer h_int_l_dome[2] "Internal convective heat transfer of the liquid at the dome";
  CoefficientOfHeatTransfer h_int_v_dome[2] "Internal convective heat transfer of the vapor at the dome";

  CoefficientOfHeatTransfer h_rad_l[8] "Internal radiation equivalent convective coefficient for the liquid";
  CoefficientOfHeatTransfer h_rad_v[8] "Internal radiation equivalent convective coefficient for the vapor";
  CoefficientOfHeatTransfer h_rad_l_dome[2] "Internal radiation equivalent convective coefficient for the liquid at the dome";
  CoefficientOfHeatTransfer h_rad_v_dome[2] "Internal radiation equivalent convective coefficient for the vapor at the dome";

  CoefficientOfHeatTransfer h_tot_l[8] "Total internal heat transfer coefficient for the liquid";
  CoefficientOfHeatTransfer h_tot_v[8] "Total internal heat transfer of the vapor";
  CoefficientOfHeatTransfer h_tot_l_dome[2] "Total internal heat transfer coefficient for the liquid at the dome";
  CoefficientOfHeatTransfer h_tot_v_dome[2] "Total internal heat transfer of the vapor at the dome";

  CustomInterfaces.ZeroDimensional.ExtFluidPort_B LH2_outlet(redeclare package
      Medium = Medium, m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
           else 0)) annotation (Placement(transformation(extent={{-154,-82},{-132,
            -60}}, rotation=0), iconTransformation(extent={{-154,-82},{-132,-60}})));
  CustomInterfaces.ZeroDimensional.ExtFluidPort_B VH2_outlet(redeclare package
      Medium = Medium, m_flow(max=if allowFlowReversal then +Modelica.Constants.inf
           else 0)) annotation (Placement(transformation(extent={{22,80},{40,98}},
          rotation=0), iconTransformation(extent={{22,80},{40,98}})));
  CustomInterfaces.ThreeDimensional.HeatPort3D_A wall(Nx=8, Ny=1) annotation (
      Placement(transformation(extent={{-48,42},{-8,82}}, rotation=0),
        iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-28,62})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heat_port annotation (
      Placement(transformation(extent={{10,-72},{48,-34}}, rotation=0),
        iconTransformation(extent={{48,-44},{66,-26}})));
  CustomInterfaces.ThreeDimensional.HeatPort3D_A dome(Nx=2, Ny=1) annotation (
      Placement(transformation(extent={{-116,66},{-90,92}}, rotation=0),
        iconTransformation(
        extent={{-22,-22},{22,22}},
        rotation=90,
        origin={104,-40})));

equation
  // Conservation equations
  der(Mv) = qv + wev - wc + ws "Vapour volume mass balance";
  der(Ml) = ql - wev + wc - ws "Liquid volume mass balance";
  der(Ev) = qv*hv + wev*hvs - wc*hls + ws*hvs + Qv_tot - Qsv - P*der(Vv) "Vapour volume energy balance";
  der(El) = ql*hl - wev*hvs + wc*hls - ws*hls + Ql_tot + Qsl - P*der(Vl) "Liquid volume energy balance";
  //(Ev) = qv*hv + (wev - wcs)*hvs - wc*hls + Qv_tot - Qvl - p*der(Vv) "Vapour volume energy balance";
  //der(El) = ql*hl + (wcs - wev)*hvs + wc*hls + Ql_tot + Qvl - p*der(Vl) "Liquid volume energy balance";

  // Fluid properties
  sat = Medium.setSat_p(P);
  liq = Medium.setState_ph(P, hl, 2);
  Tl = Medium.temperature_ph(P, hl, 1);
  rhol = Medium.density_ph(P, hl, 1);
  ul = Medium.specificInternalEnergy(liq);
  vap = Medium.setState_ph(P, hv, 2);
  Tv = Medium.temperature_ph(P, hv, 1);
  rhov = Medium.density_ph(P, hv, 1);
  uv = Medium.specificInternalEnergy(vap);

  hls = sat.hl;
  hvs = sat.hv;
  Ts = sat.Tsat;
  Dh = hvs - hls;

  Mv = Vv*rhov;
  Ml = Vl*rhol;
  Ev = Mv*(hv - P/rhov); //Medium.specificInternalEnergy(vap);
  El = Ml*(hl - P/rhol); //Medium.specificInternalEnergy(liq);

  wev = xl*rhol*Vl/tauev;
  wc = (1 - xv)*rhov*Vv/tauc;
  //wcs = Kcs*Asup*(Ts - Tl);
  //wes = Kes*Asup*(Ts - Tv);
  ws = (Qsv - Qsl)/Dh;
  //wes = homotopy(if Qsv >= Qsl then ws else 0, 0);
  //wcs = homotopy(if Qsl >= Qsv then -ws else 0, 0);

  //Qvl = Ks*Asup*(Tv - Ts);
  Qsv = 2*Asup*vap.lambda*(Tv - Ts)/((R_int - y)/2);
  Qsl = 2*Asup*liq.lambda*(Ts - Tl)/((R_int + y)/2);
  Q_tot = sum(Qwl) + sum(Qwv) + Q_int;

  xv = homotopy(if hv >= hvs then 1 else (hv - hls)/(hvs - hls),
    (hv - hls)/(hvs - hls));
  xl = homotopy(if hl <= hls then 0 else (hl - hls)/(hvs - hls), 0);

  Prl = (liq.eta*liq.cp)/(liq.lambda);
  Prv = (vap.eta*vap.cp)/(vap.lambda);

  y0 = y + R_int "Level (referred to the bottom)";
  Vl = L*(R_int^2*acos(-y/R_int) + y*sqrt(R_int^2 - y^2)) +
    pi*(y0 + dR)^2*(R_eq - 1/3*(y0 + dR)) -
    2*(L_star*(R_star^2*acos(-y/R_star) + y*sqrt(R_star^2 - y^2)));
  Awl_tot = 2*R_int*acos(-y/R_int)*L + 2*pi*R_eq*(y0 + dR) -
    2*(2*R_star*acos(-y/R_star)*L_star);
  //Awl = 2*R_int*acos(-y/R_int)*L + 2*pi*R_eq*(y0 + dR) - 2* (2*pi*R_eq*(R_eq - AR*R_int)*((y0 + dR)/R_eq));
  Asup = 2*sqrt(R_int^2 - y^2)*L + pi*(R_eq*cos(asin((y)/R_eq)))^2 - 2* (2*sqrt(R_star^2 - y^2)*L_star);

  Vt = Vl + Vv;
  Awt_tot = Awv_tot + Awl_tot;
  //Awv_tot = 2*pi*R_int*L - Awl "Metal-vapor interface area";

  // Define difference in height between the separation lines that divide the different wall segments
  dh_outer = R_int*(1-cos(Modelica.Units.Conversions.from_deg(22.5)))
    "Height of wall segments 0 and 4 (top and bottom)";
  dh_between = R_int*(1-cos(Modelica.Units.Conversions.from_deg(22.5+45))) - dh_outer
    "Height of wall segments 1,3,5, and 7";
  dh_inner = (R_int - dh_outer - dh_between)*2
    "Height of wall segments 2 and 6 (central sections)";

  // Calculate the surface area available for heat transfer on the inner tank, for each wall segment
  Awt[1] = 2*R_int*acos(-(-(dh_inner/2+dh_between))/R_int)*L;
  Awt[2] = (2*R_int*acos(-(-(dh_inner/2))/R_int)*L - Awt[1])/2;
  Awt[3] = (2*R_int*acos(-(dh_inner/2)/R_int)*L - Awt[1] - 2*Awt[2])/2;
  Awt[4] = Awt[2];
  Awt[5] = Awt[1];
  Awt[6] = Awt[2];
  Awt[7] = Awt[3];
  Awt[8] = Awt[2];
  //Awt_dome[1] = 0.5 * 4*pi*R_int^2*((1+2*AR^1.6075)/3)^(1/1.6075) "Inner wall surface area of dome 1";
  Awt_dome[1] = 2*pi*R_eq*(R_int*AR);
  Awt_dome[2] = Awt_dome[1];

  // Calculate surface area in contact with the liquid for each wall segment
  if y >= (dh_inner/2 + dh_between) then
    Awl[1] = Awt[1];
    Awl[2] = Awt[2];
    Awl[3] = Awt[3];
    Awl[4] = Awt[4];
    Awl[5] = 2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[7] - Awt[6] - Awt[4] - Awt[3] - Awt[2] - Awt[1];
    Awl[6] = Awt[6];
    Awl[7] = Awt[7];
    Awl[8] = Awt[8];

  elseif y >= (dh_inner/2) then
    Awl[1] = Awt[1];
    Awl[2] = Awt[2];
    Awl[3] = Awt[3];
    Awl[4] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[7] - Awt[3] - Awt[2] - Awt[1])/2;
    Awl[5] = 0;
    Awl[6] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[7] - Awt[3] - Awt[2] - Awt[1])/2;
    Awl[7] = Awt[7];
    Awl[8] = Awt[8];

  elseif y >= -(dh_inner/2) then
    Awl[1] = Awt[1];
    Awl[2] = Awt[2];
    Awl[3] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[2] - Awt[1])/2;
    Awl[4] = 0;
    Awl[5] = 0;
    Awl[6] = 0;
    Awl[7] = (2*R_int*acos(-y/R_int)*L - Awt[8] - Awt[2] - Awt[1])/2;
    Awl[8] = Awt[8];

  elseif y >= -(dh_inner/2 + dh_between) then
    Awl[1] = Awt[1];
    Awl[2] = (2*R_int*acos(-y/R_int)*L - Awt[1])/2;
    Awl[3] = 0;
    Awl[4] = 0;
    Awl[5] = 0;
    Awl[6] = 0;
    Awl[7] = 0;
    Awl[8] = (2*R_int*acos(-y/R_int)*L - Awt[1])/2;

  else
    Awl[1] = 2*R_int*acos(-y/R_int)*L;
    Awl[2] = 0;
    Awl[3] = 0;
    Awl[4] = 0;
    Awl[5] = 0;
    Awl[6] = 0;
    Awl[7] = 0;
    Awl[8] = 0;
  end if;

  for i in 1:8 loop
    // Calculate surface area in contact with the vapor for each wall segment
    Awv[i] = Awt[i] - Awl[i];

    // Calculate Rayleigh number
    Ral[i] = (g*0.01658*abs(Tw[i] - Tl)*((R_int + y)^3)*Prl*rhol^2)/(liq.eta^2);
    Rav[i] = (g*0.01658*abs(Tw[i] - Tv)*((R_int - y)^3)*Prv*rhov^2)/(vap.eta^2);

    // Calculate Nusselt number
    Nul[i] = 0.0605*(Ral[i]^(1/3));
    Nuv[i] = 0.364*2*R_int/(R_int - y)*(Rav[i]^(1/4));

    // Calculate internal convective heat transfer coefficient
    h_int_l[i] = (Nul[i]*liq.lambda)/(R_int + y);
    h_int_v[i] = (Nuv[i]*vap.lambda)/(R_int - y);

    // Calculate equivalent radiative heat transfer coefficient
    h_rad_l[i] = Modelica.Constants.sigma*0.09*(Tw[i]^2 + Tl^2)*(Tw[i] + Tl);
    h_rad_v[i] = 0;  // Assumption

    // Calculate total heat transfer coefficient
    h_tot_l[i] = h_int_l[i] + h_rad_l[i];
    h_tot_v[i] = h_int_v[i] + h_rad_v[i];

    // Heat flows of the wall segments
    Qwl[i] = h_tot_l[i]*Awl[i]*(Tw[i] - Tl);
    Qwv[i] = h_tot_v[i]*Awv[i]*(Tw[i] - Tv);

    // Boundary conditions
    wall.ports[i,1].T = Tw[i];
    Qwv[i] + Qwl[i] = wall.ports[i,1].Q_flow;
  end for;

  for i in 1:2 loop
    // Calculate surface area in contact with the vapor at the domes
    Awv_dome[i] = Awt_dome[i] - Awl_dome[i];

    // Calculate surface area in contact with the liquid at the domes
    Awl_dome[i] = pi*R_eq*(y0 + dR) - (2*R_star*acos(-y/R_star)*L_star);

    // Calculate Rayleigh number
    Ral_dome[i] = (g*0.01658*abs(Tw_dome[i] - Tl)*((R_int + y)^3)*Prl*rhol^2)/(liq.eta^2);
    Rav_dome[i] = (g*0.01658*abs(Tw_dome[i] - Tv)*((R_int - y)^3)*Prv*rhov^2)/(vap.eta^2);

    // Calculate Nusselt number
    Nul_dome[i] = 0.0605*(Ral_dome[i]^(1/3));
    Nuv_dome[i] = 0.364*2*R_int/(R_int - y)*(Rav_dome[i]^(1/4));

    // Calculate internal convective heat transfer coefficient
    h_int_l_dome[i] = (Nul_dome[i]*liq.lambda)/(R_int+y);
    h_int_v_dome[i] = (Nuv_dome[i]*vap.lambda)/(R_int-y);

    // Calculate equivalent radiative heat transfer coefficient
    h_rad_l_dome[i] = Modelica.Constants.sigma*0.09*(Tw_dome[i]^2 + Tl^2)*(Tw_dome[i] + Tl);
    h_rad_v_dome[i] = 0;  // Assumption

    // Calculate total heat transfer coefficient
    h_tot_l_dome[i] = h_int_l_dome[i] + h_rad_l_dome[i];
    h_tot_v_dome[i] = h_int_v_dome[i] + h_rad_v_dome[i];

    // Heat flows of the wall segments
    Qwl_dome[i] = h_tot_l_dome[i]*Awl_dome[i]*(Tw_dome[i] - Tl);
    Qwv_dome[i] = h_tot_v_dome[i]*Awv_dome[i]*(Tw_dome[i] - Tv);

    // Boundary conditions
    dome.ports[i,1].T = Tw_dome[i];
    Qwv_dome[i] + Qwl_dome[i] = dome.ports[i,1].Q_flow;
  end for;

  // Split the internal heat flow to the liquid and vapor, according to the volume
  A_frac = 1;//(Awl0+Awl1+Awl2+Awl3+Awl4+Awl5+Awl6+Awl7)/(Awt[1]+Awt[2]+Awt[3]+Awt[4]+Awt[5]+Awt[6]+Awt[7]+Awt[8]) "Liquid area fraction, neglecting the dome volumes";
  Qv_int = Q_int*(1 - A_frac);
  Ql_int = Q_int*A_frac;

  Qwl_tot = sum(Qwl) + sum(Qwl_dome);
  Qwv_tot = sum(Qwv) + sum(Qwv_dome);
  Ql_tot = Qwl_tot + Ql_int;
  Qv_tot = Qwv_tot + Qv_int;

  // Boundary conditions
  LH2_outlet.P = P;
  LH2_outlet.m_flow = ql;
  LH2_outlet.h_outflow = hl;
  VH2_outlet.P = P;
  VH2_outlet.m_flow = qv;
  VH2_outlet.h_outflow = hv;
  heat_port.T = Tv;
  Q_int = heat_port.Q_flow;

  // Sanity check
  assert(Ml > 1e-6, "Liquid fuel depleted");
  assert(Mv > 1e-6, "Gaseous fuel depleted");
  assert(Vl > 1e-6, "Liquid fuel depleted");
  assert(Vv > 1e-6, "Gaseous fuel depleted");
  assert(y < R_int - 1e-6, "Liquid level above max tank radius");
  assert(y > -R_int + 1e-6, "Liquid level below min tank radius");

initial equation
  if initOpt == DynTherM.Choices.InitOpt.noInit then
    // do nothing

  elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
    if not noInitialPressure then
      P = P_start;
    end if;

    hl = hl_start;
    hv = hv_start;
    Vl = Vl_start;

  elseif initOpt == DynTherM.Choices.InitOpt.steadyState then
    if not noInitialPressure then
      der(P) = 0;
    end if;

    der(hl) = 0;
    der(hv) = 0;
    der(Vl) = 0;

  else
    assert(false, "Unsupported initialization option");
  end if;
  annotation (Dialog(tab="Initialisation"),
                    Dialog(tab="Initialisation"),
        Documentation(info="<HTML>
<p>Simplified model of a drum for drum boilers and fire-tube boilers. This model assumes
<ul>
<li>Thermodynamic equiibrium between the liquid, vapor, and metal wall
<li>Perfect separation of the liquid and vapor phase
</ul></p>
<p>The model has two state variables the pressure <code>p</code> and the liquid volume <code>Vl</code>. It is possible to extend it,
adding a specific geometry and the computation of the level from the liquid volume. In that case, one may want to use the level as a state.
</p>
</HTML>",   revisions="<html>
<ul>
<li><i>19 Feb 2019</i>
    by <a href=\"mailto:francesco.casella@polimi.it\">Francesco Casella</a>:<br>
       Adapted from old <code>Drum2States</code> model.</li>
</ul>
</html>"),
      Icon(graphics={Rectangle(
            extent={{-152,100},{122,-100}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Bitmap(extent={{-220,-136},{224,152}}, fileName="modelica://DynTherM/Figures/LH2Tank.png")}));
end TankInternal;
