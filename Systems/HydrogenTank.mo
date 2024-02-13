within DynTherM.Systems;
package HydrogenTank

  model tmp
    outer DynTherM.Components.Environment environment "Environmental properties";
    replaceable package Medium = Media.ExtMedia.CoolProp.Hydrogen constrainedby
      ExternalMedia.Media.BaseClasses.ExternalTwoPhaseMedium "Medium model" annotation(choicesAllMatching = true);

        // Parameters, system properties, and constants
        parameter Boolean allowFlowReversal=environment.allowFlowReversal
          "= true to allow flow reversal, false restricts to design direction" annotation(Evaluate=true);
        parameter Choices.InitOpt initOpt=environment.initOpt
          "Initialisation option" annotation (Dialog(tab="Initialisation"));
        parameter Boolean noInitialPressure=false
          "Remove initial equation on pressure" annotation (Dialog(tab="Initialisation"),choices(checkBox=true));
        parameter Pressure P_start "Pressure start value" annotation (Dialog(tab="Initialisation"));
        parameter Real Ff "Tank Fill fraction";
        parameter Volume Vt = pi*R_tank_int^2*L + 2* pi*(R_tank_int*AR)^2*(R_eq - (R_tank_int*AR)/3)
                                                                "Tank internal volume";
        parameter Area Awt = 2*pi*R_tank_int*L + 2*pi*R_eq^2*AR "Tank internal area";
        parameter Volume Vlstart = Vt*Ff "Liquid fluid volume";
        parameter Length R_tank_int "Internal radius of the fuel tank";
        parameter Length L "Length";
        parameter Real AR=1 "Tank end-dome aspect ratio (h/R)";
        parameter Time tauev = 1E1 "Time constant of bulk evaporation";
        parameter Time tauc = 1E1 "Time constant of bulk condensation";
        //parameter Real Kcs = 0 "Surface condensation coefficient [kg/(s.m2.K)]";
        //parameter Real Kes = 0 "Surface evaporation coefficient [kg/(s.m2.K)]";
        parameter Real Kvs = 0 "Surface heat transfer coefficient (vapour-surface) [W/(m2.K)]";
        parameter Real Kls = 0 "Surface heat transfer coefficient (liquid-surface) [W/(m2.K)]";
        parameter Medium.SpecificEnthalpy hlstart=Medium.bubbleEnthalpy(Medium.setSat_p(P_start)) "Liquid enthalpy start value"
          annotation (Dialog(tab="Initialisation"));
        parameter Medium.SpecificEnthalpy hvstart=Medium.dewEnthalpy(Medium.setSat_p(P_start)) "Vapour enthalpy start value"
          annotation (Dialog(tab="Initialisation"));
        parameter Length ystart = 0.808*Ff*R_tank_int "Startlevel (referred to the centreline), needed since y is an iteration variable of the initialization problem";
        constant Acceleration g=Modelica.Constants.g_n;
        constant Real pi=Modelica.Constants.pi;
        constant Real eps = 1E-6 "Numerical error on assertions";

        // Geometrical Tank Variables
        Real A_frac "Liquid area fraction";
        Length y(start = ystart) "Level (referred to the centreline)";
        Length y0 "Level (referred to the bottom)";
        Length dh_outer "Height of wall segments 0 and 4 (top and bottom)";
        Length dh_between "Height of wall segments 1,3,5, and 7";
        Length dh_inner "Height of wall segments 2 and 6 (central sections)";
        final parameter Length R_eq = R_tank_int*(1+AR^2)/(2*AR) "Radius of equivalent sphere when dome AR < 1";
        final parameter Length L_star = R_eq - AR*R_tank_int "Lenght of aproximated cylinder which is cut of from a half sphere due to aspect ratio (h/R) being smaller than one";
        final parameter Length L_star_loc = L_star/2 "Horizontal location of the R_star (middle of the cylinder)";
        final parameter Length R_star = R_eq*cos(asin(L_star_loc/R_eq)) "Radius of aproximated cylinder which is cut of from a half sphere due to aspect ratio (h/R) being smaller than one";
        final parameter Integer n_star = 1 "Number of equivalent sections of volume cylinders";
        final parameter Length dR = R_eq - R_tank_int "Difference between radius of tank/domes and equivalents sphere";

        // Thermodynamics States
        Medium.SaturationProperties sat "Saturation conditions";
        Medium.ThermodynamicState liquidState "Thermodynamic state of the liquid";
        Medium.ThermodynamicState vapourState "Thermodynamic state of the vapour";

        // Mass flow rates
        MassFlowRate ql "Liquid fluid mass flowrate";
        MassFlowRate qv "Gaseous fluid mass flowrate";
        //MassFlowRate wcs "Mass flowrate of surface condensation";
        //MassFlowRate wes "Mass flowrate of surface evaporation";
        MassFlowRate wc "Mass flowrate of bulk condensation";
        MassFlowRate wev "Mass flowrate of bulk evaporation";
        MassFlowRate ws "Mass flowrate of surface evaporation/condensation (positive for evaporation, negative for condensation)";

        // Fluid Properties
        Mass Mv "Gaseous fluid mass";
        Mass Ml "Liquid fluid mass";
        Volume Vv(start= Vt - Vlstart) "Gaseous fluid volume";
        Volume Vl(start= Vlstart, stateSelect=StateSelect.default) "Liquid fluid volume";
        Energy El "Liquid internal energy";
        Energy Ev "Vapour internal energy";
        PerUnit xl "Mass fraction of vapour in the liquid volume";
        PerUnit xv "Steam quality in the vapour volume";
        Medium.AbsolutePressure P(start=P_start, stateSelect=StateSelect.prefer)
          "Surface pressure";
        SpecificEnthalpy hls "Specific enthalpy of saturated liquid";
        SpecificEnthalpy hvs "Specific enthalpy of saturated vapour";
        SpecificEnthalpy Dh "Latent Heat of Evaporation at boiling point";
        Medium.SpecificEnthalpy hl(start=hlstart, stateSelect=StateSelect.prefer) "Specific enthalpy of liquid";
        Medium.SpecificEnthalpy hv(start=hvstart, stateSelect=StateSelect.prefer) "Specific enthalpy of vapour";
        Medium.Temperature Ts "Saturation temperature";
        Medium.Temperature Tl "Liquid temperature";
        Medium.Temperature Tv "Vapour temperature";
        Density rhol "Liquid density";
        Density rhov "Vapour density";
        Medium.SpecificInternalEnergy ul "Liquid specific internal energy";
        Medium.SpecificInternalEnergy uv "Vapour specific internal energy";

        // Wall temperatures
        Temperature Tw0 "Wall temperature segment 0";
        Temperature Tw1 "Wall temperature segment 1";
        Temperature Tw2 "Wall temperature segment 2";
        Temperature Tw3 "Wall temperature segment 3";
        Temperature Tw4 "Wall temperature segment 4";
        Temperature Tw5 "Wall temperature segment 5";
        Temperature Tw6 "Wall temperature segment 6";
        Temperature Tw7 "Wall temperature segment 7";
        Temperature Twdome1 "Wall temperature dome 1";
        Temperature Twdome2 "Wall temperature dome 2";

        // Heat Flows
        Power Qtot "Total heat flow from wall to inner";
        Power Qwv "Heat flow from the wall to the vapour";
        Power Qwl "Heat flow from the wall to the liquid";
        Power Qltot "Total heat flow to the liquid";
        Power Qvtot "Total heat flow to the vapour";
        Power Qwv0 "Heat flow from the wall to the vapour segment 0";
        Power Qwl0 "Heat flow from the wall to the liquid segment 0";
        Power Qwv1 "Heat flow from the wall to the vapour segment 1";
        Power Qwl1 "Heat flow from the wall to the liquid segment 1";
        Power Qwv2 "Heat flow from the wall to the vapour segment 2";
        Power Qwl2 "Heat flow from the wall to the liquid segment 2";
        Power Qwv3 "Heat flow from the wall to the vapour segment 3";
        Power Qwl3 "Heat flow from the wall to the liquid segment 3";
        Power Qwv4 "Heat flow from the wall to the vapour segment 4";
        Power Qwl4 "Heat flow from the wall to the liquid segment 4";
        Power Qwv5 "Heat flow from the wall to the vapour segment 5";
        Power Qwl5 "Heat flow from the wall to the liquid segment 5";
        Power Qwv6 "Heat flow from the wall to the vapour segment 6";
        Power Qwl6 "Heat flow from the wall to the liquid segment 6";
        Power Qwv7 "Heat flow from the wall to the vapour segment 7";
        Power Qwl7 "Heat flow from the wall to the liquid segment 7";
        Power Qwv_dome1 "Heat flow from the wall to dome 1";
        Power Qwl_dome1 "Heat flow from the wall to dome 1";
        Power Qwv_dome2 "Heat flow from the wall to dome 2";
        Power Qwl_dome2 "Heat flow from the wall to dome 2";
        Power Qvs "Heat flow from the vapour to the liquid";
        Power Qsl "Heat flow from the liquid to the vapour";
        Power Qlint "Heat flow introduced into the liquid directly";
        Power Qvint "Heat flow introduced into the vapour directly";
        Power Qint "Heat flow introduced into the fluid (liq. and vap.) directly";

        // Wall Interface Surface Area's
        Area Awl "Surface of the wall-liquid interface";
        Area Awv "Surface of the wall-vapour interface";
        Area Awl0 "Surface of the wall-liquid interface segment 0";
        Area Awv0 "Surface of the wall-vapour interface segment 0";
        Area Awl1 "Surface of the wall-liquid interface segment 1";
        Area Awv1 "Surface of the wall-vapour interface segment 1";
        Area Awl2 "Surface of the wall-liquid interface segment 2";
        Area Awv2 "Surface of the wall-vapour interface segment 2";
        Area Awl3 "Surface of the wall-liquid interface segment 3";
        Area Awv3 "Surface of the wall-vapour interface segment 3";
        Area Awl4 "Surface of the wall-liquid interface segment 4";
        Area Awv4 "Surface of the wall-vapour interface segment 4";
        Area Awl5 "Surface of the wall-liquid interface segment 5";
        Area Awv5 "Surface of the wall-vapour interface segment 5";
        Area Awl6 "Surface of the wall-liquid interface segment 6";
        Area Awv6 "Surface of the wall-vapour interface segment 6";
        Area Awl7 "Surface of the wall-liquid interface segment 7";
        Area Awv7 "Surface of the wall-vapour interface segment 7";
        Area Awl_dome1 "Surface of the wall-liquid interface at dome 1";
        Area Awv_dome1 "Surface of the wall-vapour interface at dome 1";
        Area Awl_dome2 "Surface of the wall-liquid interface at dome 2";
        Area Awv_dome2 "Surface of the wall-vapour interface at dome 2";
        Area Asup "Surface of the liquid-vapour interface";
        Area Awt0 "Surface of the wall segment 0";
        Area Awt1 "Surface of the wall segment 1";
        Area Awt2 "Surface of the wall segment 2";
        Area Awt3 "Surface of the wall segment 3";
        Area Awt4 "Surface of the wall segment 4";
        Area Awt5 "Surface of the wall segment 5";
        Area Awt6 "Surface of the wall segment 6";
        Area Awt7 "Surface of the wall segment 7";
        Area Awt_dome1 "Surface of dome 1";
        Area Awt_dome2 "Surface of dome 2";

        // Heat Transfer Coefficients and Constants
        PrandtlNumber Prl "Prandtl number of the liquid";
        PrandtlNumber Prv "Prandtl number of the vapour";

        NusseltNumber Nuhl0 "Nusselt number of the liquid segment 0";
        NusseltNumber Nuhl1 "Nusselt number of the liquid segment 1";
        NusseltNumber Nuhl2 "Nusselt number of the liquid segment 2";
        NusseltNumber Nuhl3 "Nusselt number of the liquid segment 3";
        NusseltNumber Nuhl4 "Nusselt number of the liquid segment 4";
        NusseltNumber Nuhl5 "Nusselt number of the liquid segment 5";
        NusseltNumber Nuhl6 "Nusselt number of the liquid segment 6";
        NusseltNumber Nuhl7 "Nusselt number of the liquid segment 7";
        NusseltNumber Nuhl_dome1 "Nusselt number of the liquid at dome 1";
        NusseltNumber Nuhl_dome2 "Nusselt number of the liquid at dome 2";
        NusseltNumber Nuhv0 "Nusselt number of the vapour at wall segment 0";
        NusseltNumber Nuhv1 "Nusselt number of the vapour at wall segment 1";
        NusseltNumber Nuhv2 "Nusselt number of the vapour at wall segment 2";
        NusseltNumber Nuhv3 "Nusselt number of the vapour at wall segment 3";
        NusseltNumber Nuhv4 "Nusselt number of the vapour at wall segment 4";
        NusseltNumber Nuhv5 "Nusselt number of the vapour at wall segment 5";
        NusseltNumber Nuhv6 "Nusselt number of the vapour at wall segment 6";
        NusseltNumber Nuhv7 "Nusselt number of the vapour at wall segment 7";
        NusseltNumber Nuhv_dome1 "Nusselt number of the vapour at dome 1";
        NusseltNumber Nuhv_dome2 "Nusselt number of the vapour at dome 2";

        RayleighNumber Rahl0 "Rayleigh number of the liquid segment 0";
        RayleighNumber Rahl1 "Rayleigh number of the liquid segment 1";
        RayleighNumber Rahl2 "Rayleigh number of the liquid segment 2";
        RayleighNumber Rahl3 "Rayleigh number of the liquid segment 3";
        RayleighNumber Rahl4 "Rayleigh number of the liquid segment 4";
        RayleighNumber Rahl5 "Rayleigh number of the liquid segment 5";
        RayleighNumber Rahl6 "Rayleigh number of the liquid segment 6";
        RayleighNumber Rahl7 "Rayleigh number of the liquid segment 7";
        RayleighNumber Rahl_dome1 "Rayleigh number of the liquid at dome 1";
        RayleighNumber Rahl_dome2 "Rayleigh number of the liquid at dome 2";
        RayleighNumber Rahv0 "Rayleigh number of the vapour at wall segment 0";
        RayleighNumber Rahv1 "Rayleigh number of the vapour at wall segment 1";
        RayleighNumber Rahv2 "Rayleigh number of the vapour at wall segment 2";
        RayleighNumber Rahv3 "Rayleigh number of the vapour at wall segment 3";
        RayleighNumber Rahv4 "Rayleigh number of the vapour at wall segment 4";
        RayleighNumber Rahv5 "Rayleigh number of the vapour at wall segment 5";
        RayleighNumber Rahv6 "Rayleigh number of the vapour at wall segment 6";
        RayleighNumber Rahv7 "Rayleigh number of the vapour at wall segment 7";
        RayleighNumber Rahv_dome1 "Rayleigh number of the vapour at wall at dome 1";
        RayleighNumber Rahv_dome2 "Rayleigh number of the vapour at wall at dome 2";

        CoefficientOfHeatTransfer hintl0 "Internal convective heat transfer of the liquid segment 0";
        CoefficientOfHeatTransfer hintl1 "Internal convective heat transfer of the liquid segment 1";
        CoefficientOfHeatTransfer hintl2 "Internal convective heat transfer of the liquid segment 2";
        CoefficientOfHeatTransfer hintl3 "Internal convective heat transfer of the liquid segment 3";
        CoefficientOfHeatTransfer hintl4 "Internal convective heat transfer of the liquid segment 4";
        CoefficientOfHeatTransfer hintl5 "Internal convective heat transfer of the liquid segment 5";
        CoefficientOfHeatTransfer hintl6 "Internal convective heat transfer of the liquid segment 6";
        CoefficientOfHeatTransfer hintl7 "Internal convective heat transfer of the liquid segment 7";
        CoefficientOfHeatTransfer hintl_dome1 "Internal convective heat transfer of the liquid at dome 1";
        CoefficientOfHeatTransfer hintl_dome2 "Internal convective heat transfer of the liquid at dome 2";
        CoefficientOfHeatTransfer hintv0 "Internal convective heat transfer of the vapour segment 0";
        CoefficientOfHeatTransfer hintv1 "Internal convective heat transfer of the vapour segment 1";
        CoefficientOfHeatTransfer hintv2 "Internal convective heat transfer of the vapour segment 2";
        CoefficientOfHeatTransfer hintv3 "Internal convective heat transfer of the vapour segment 3";
        CoefficientOfHeatTransfer hintv4 "Internal convective heat transfer of the vapour segment 4";
        CoefficientOfHeatTransfer hintv5 "Internal convective heat transfer of the vapour segment 5";
        CoefficientOfHeatTransfer hintv6 "Internal convective heat transfer of the vapour segment 6";
        CoefficientOfHeatTransfer hintv7 "Internal convective heat transfer of the vapour segment 7";
        CoefficientOfHeatTransfer hintv_dome1 "Internal convective heat transfer of the vapour at dome 1";
        CoefficientOfHeatTransfer hintv_dome2 "Internal convective heat transfer of the vapour at dome 2";

        CoefficientOfHeatTransfer hradl0 "Internal radiation equivalent convective coefficient for the liquid segment 0";
        CoefficientOfHeatTransfer hradl1 "Internal radiation equivalent convective coefficient for the liquid segment 1";
        CoefficientOfHeatTransfer hradl2 "Internal radiation equivalent convective coefficient for the liquid segment 2";
        CoefficientOfHeatTransfer hradl3 "Internal radiation equivalent convective coefficient for the liquid segment 3";
        CoefficientOfHeatTransfer hradl4 "Internal radiation equivalent convective coefficient for the liquid segment 4";
        CoefficientOfHeatTransfer hradl5 "Internal radiation equivalent convective coefficient for the liquid segment 5";
        CoefficientOfHeatTransfer hradl6 "Internal radiation equivalent convective coefficient for the liquid segment 6";
        CoefficientOfHeatTransfer hradl7 "Internal radiation equivalent convective coefficient for the liquid segment 7";
        CoefficientOfHeatTransfer hradl_dome1 "Internal radiation equivalent convective coefficient for the liquid at dome 1";
        CoefficientOfHeatTransfer hradl_dome2 "Internal radiation equivalent convective coefficient for the liquid at dome 2";
        CoefficientOfHeatTransfer hradv "Internal radiation equivalent convective coefficient for the vapour";

        CoefficientOfHeatTransfer htotl0 "Total internal heat transfer coefficient for the liquid segment 0";
        CoefficientOfHeatTransfer htotl1 "Total internal heat transfer coefficient for the liquid segment 1";
        CoefficientOfHeatTransfer htotl2 "Total internal heat transfer coefficient for the liquid segment 2";
        CoefficientOfHeatTransfer htotl3 "Total internal heat transfer coefficient for the liquid segment 3";
        CoefficientOfHeatTransfer htotl4 "Total internal heat transfer coefficient for the liquid segment 4";
        CoefficientOfHeatTransfer htotl5 "Total internal heat transfer coefficient for the liquid segment 5";
        CoefficientOfHeatTransfer htotl6 "Total internal heat transfer coefficient for the liquid segment 6";
        CoefficientOfHeatTransfer htotl7 "Total internal heat transfer coefficient for the liquid segment 7";
        CoefficientOfHeatTransfer htotl_dome1 "Total internal heat transfer coefficient for the liquid dome 1";
        CoefficientOfHeatTransfer htotl_dome2 "Total internal heat transfer coefficient for the liquid dome 2";
        CoefficientOfHeatTransfer htotv0 "Total internal heat transfer of the vapour segment 0";
        CoefficientOfHeatTransfer htotv1 "Total internal heat transfer of the vapour segment 1";
        CoefficientOfHeatTransfer htotv2 "Total internal heat transfer of the vapour segment 2";
        CoefficientOfHeatTransfer htotv3 "Total internal heat transfer of the vapour segment 3";
        CoefficientOfHeatTransfer htotv4 "Total internal heat transfer of the vapour segment 4";
        CoefficientOfHeatTransfer htotv5 "Total internal heat transfer of the vapour segment 5";
        CoefficientOfHeatTransfer htotv6 "Total internal heat transfer of the vapour segment 6";
        CoefficientOfHeatTransfer htotv7 "Total internal heat transfer of the vapour segment 7";
        CoefficientOfHeatTransfer htotv_dome1 "Total internal heat transfer of the vapour at dome 1";
        CoefficientOfHeatTransfer htotv_dome2 "Total internal heat transfer of the vapour at dome 2";

      CustomInterfaces.ExtFluidPort_B LH2_outlet(redeclare package Medium = Medium,
        m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(extent={{-150,-82},{-128,-60}},
                rotation=0), iconTransformation(extent={{-150,-82},{-128,-60}})));
      CustomInterfaces.ExtFluidPort_B VH2_outlet(redeclare package Medium = Medium,
        m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0)) annotation (Placement(transformation(extent={{20,80},{38,98}}, rotation=0),
              iconTransformation(extent={{20,80},{38,98}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall0 "Metal wall thermal port" annotation (Placement(transformation(extent={{-32,-20},
                {-20,-8}},         rotation=0), iconTransformation(extent={{-32,-20},
                {-20,-8}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall1 "Metal wall thermal port" annotation (Placement(transformation(extent={{-60,-14},
                {-48,-2}},         rotation=0), iconTransformation(extent={{-60,-14},
                {-48,-2}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall2 "Metal wall thermal port" annotation (Placement(transformation(extent={{-74,14},
                {-62,26}},         rotation=0), iconTransformation(extent={{-74,14},
                {-62,26}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall3 "Metal wall thermal port" annotation (Placement(transformation(extent={{-62,46},
                {-50,58}},         rotation=0), iconTransformation(extent={{-62,46},
                {-50,58}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall4 "Metal wall thermal port" annotation (Placement(transformation(extent={{-34,62},
                {-22,74}},         rotation=0), iconTransformation(extent={{-34,62},
                {-22,74}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall5 "Metal wall thermal port" annotation (Placement(transformation(extent={{-6,50},
                {6,62}},           rotation=0), iconTransformation(extent={{-6,50},
                {6,62}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall6 "Metal wall thermal port" annotation (Placement(transformation(extent={{2,24},{
                14,36}},           rotation=0), iconTransformation(extent={{2,24},{
                14,36}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a wall7 "Metal wall thermal port" annotation (Placement(transformation(extent={{-6,-4},
                {6,8}},            rotation=0), iconTransformation(extent={{-6,-4},
                {6,8}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Fluid_Heat_Port "Metal wall thermal port" annotation (Placement(transformation(extent={{10,-72},{48,-34}}, rotation=0), iconTransformation(extent={{38,-44},
                {56,-26}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a dome2 "Metal wall thermal port" annotation (Placement(transformation(extent={{-116,66},{-90,92}},  rotation=0), iconTransformation(extent={{-112,70},
                {-100,82}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a dome1 "Metal wall thermal port" annotation (Placement(transformation(extent={{84,-98},{116,-66}},rotation=0), iconTransformation(extent={{102,-40},
                {120,-22}})));

  equation
        der(Mv) = qv + wev - wc + ws "Vapour volume mass balance";
        der(Ml) = ql - wev + wc - ws "Liquid volume mass balance";
        //(Ev) = qv*hv + (wev - wcs)*hvs - wc*hls + Qvtot - Qvl - p*der(Vv) "Vapour volume energy balance";
        //der(El) = ql*hl + (wcs - wev)*hvs + wc*hls + Qltot + Qvl - p*der(Vl) "Liquid volume energy balance";
        der(Ev) = qv*hv + wev*hvs - wc*hls + ws*hvs + Qvtot - Qvs - P*der(Vv) "Vapour volume energy balance";
        der(El) = ql*hl - wev*hvs + wc*hls - ws*hls + Qltot + Qsl - P*der(Vl) "Liquid volume energy balance";

        Mv = Vv*rhov "Vapour volume mass";
        Ml = Vl*rhol "Liquid volume mass";

        Ev = Mv*(hv - P/rhov);//Medium.specificInternalEnergy(vapourState) "Vapour volume energy";
        El = Ml*(hl - P/rhol);//Medium.specificInternalEnergy(liquidState) "Liquid volume energy";

        wev = xl*rhol*Vl/tauev "Bulk evaporation flow rate";
        wc = (1 - xv)*rhov*Vv/tauc "Bulk condensation flow rate";
        //wcs = Kcs*Asup*(Ts - Tl) "Surface condensation flow rate";
        //wes = Kes*Asup*(Ts - Tv) "Surface evaporation flow rate";
        ws = (Qvs-Qsl)/Dh "Surface flow rate";
        //wes = homotopy(if Qvs >= Qsl then ws else 0, 0) "Surface evaporation flow rate";
        //wcs = homotopy(if Qsl >= Qvs then -ws else 0, 0) "Surface condensation flow rate";

        //Qvl = Ks*Asup*(Tv - Ts) "Heat flow from vapour to liquid volume";
        Qvs = 2*Asup*vapourState.lambda*(Tv - Ts)/((R_tank_int-y)/2) "Heat flow from vapour to surface";
        Qsl = 2*Asup*liquidState.lambda*(Ts - Tl)/((R_tank_int+y)/2) "Heat flow from surface to liquid volume";
        Qtot = Qwl + Qwv + Qint "Total heat flow from outer to inner";

        xv = homotopy(if hv >= hvs then 1 else (hv - hls)/(hvs - hls), (hv - hls)/(
          hvs - hls)) "Vapour quality in the vapour volume";
        xl = homotopy(if hl <= hls then 0 else (hl - hls)/(hvs - hls), 0)
          "Vapour quality in the liquid volume";

        Prl = (liquidState.eta*liquidState.cp)/(liquidState.lambda) "Prandtl number of the liquid";
        Prv = (vapourState.eta*vapourState.cp)/(vapourState.lambda) "Prandtl number of the vapour";

        Rahl0 = (g*0.01658*abs(Tw0 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl1 = (g*0.01658*abs(Tw1 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl2 = (g*0.01658*abs(Tw2 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl3 = (g*0.01658*abs(Tw3 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl4 = (g*0.01658*abs(Tw4 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl5 = (g*0.01658*abs(Tw5 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl6 = (g*0.01658*abs(Tw6 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl7 = (g*0.01658*abs(Tw7 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid";
        Rahl_dome1 = (g*0.01658*abs(Twdome1 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid at dome 1";
        Rahl_dome2 = (g*0.01658*abs(Twdome2 - Tl)*((R_tank_int+y)^3)*Prl*rhol^2)/(liquidState.eta^2) "Rayleigh number of the liquid at dome 2";
        Rahv0 = (g*0.01658*abs(Tw0 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 0";
        Rahv1 = (g*0.01658*abs(Tw1 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 1";
        Rahv2 = (g*0.01658*abs(Tw2 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 2";
        Rahv3 = (g*0.01658*abs(Tw3 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 3";
        Rahv4 = (g*0.01658*abs(Tw4 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 4";
        Rahv5 = (g*0.01658*abs(Tw5 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 5";
        Rahv6 = (g*0.01658*abs(Tw6 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 6";
        Rahv7 = (g*0.01658*abs(Tw7 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at wall segment 7";
        Rahv_dome1 = (g*0.01658*abs(Twdome1 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at dome 1";
        Rahv_dome2 = (g*0.01658*abs(Twdome2 - Tv)*((R_tank_int-y)^3)*Prv*rhov^2)/(vapourState.eta^2) "Rayleigh number of the vapour at dome 2";

        Nuhl0 = 0.0605*(Rahl0^(1/3)) "Nusselt number of the liquid";
        Nuhl1 = 0.0605*(Rahl1^(1/3)) "Nusselt number of the liquid";
        Nuhl2 = 0.0605*(Rahl2^(1/3)) "Nusselt number of the liquid";
        Nuhl3 = 0.0605*(Rahl3^(1/3)) "Nusselt number of the liquid";
        Nuhl4 = 0.0605*(Rahl4^(1/3)) "Nusselt number of the liquid";
        Nuhl5 = 0.0605*(Rahl5^(1/3)) "Nusselt number of the liquid";
        Nuhl6 = 0.0605*(Rahl6^(1/3)) "Nusselt number of the liquid";
        Nuhl7 = 0.0605*(Rahl7^(1/3)) "Nusselt number of the liquid";
        Nuhl_dome1 = 0.0605*(Rahl_dome1^(1/3)) "Nusselt number of the liquid at dome 1";
        Nuhl_dome2 = 0.0605*(Rahl_dome2^(1/3)) "Nusselt number of the liquid at dome 2";
        Nuhv0 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv0^(1/4)) "Nusselt number of the vapour at wall segment 0";//17;//
        Nuhv1 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv1^(1/4)) "Nusselt number of the vapour at wall segment 1";
        Nuhv2 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv2^(1/4)) "Nusselt number of the vapour at wall segment 2";
        Nuhv3 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv3^(1/4)) "Nusselt number of the vapour at wall segment 3";
        Nuhv4 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv4^(1/4)) "Nusselt number of the vapour at wall segment 4";
        Nuhv5 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv5^(1/4)) "Nusselt number of the vapour at wall segment 5";
        Nuhv6 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv6^(1/4)) "Nusselt number of the vapour at wall segment 6";
        Nuhv7 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv7^(1/4)) "Nusselt number of the vapour at wall segment 7";
        Nuhv_dome1 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv_dome1^(1/4)) "Nusselt number of the vapour at dome 1";
        Nuhv_dome2 = 0.364*2*R_tank_int/(R_tank_int-y)*(Rahv_dome2^(1/4)) "Nusselt number of the vapour at dome 2";

        hintl0 = (Nuhl0*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 0";
        hintl1 = (Nuhl1*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 1";
        hintl2 = (Nuhl2*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 2";
        hintl3 = (Nuhl3*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 3";
        hintl4 = (Nuhl4*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 4";
        hintl5 = (Nuhl5*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 5";
        hintl6 = (Nuhl6*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 6";
        hintl7 = (Nuhl7*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer of the liquid segment 7";
        hintl_dome1 = (Nuhl_dome1*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer at dome 1";
        hintl_dome2 = (Nuhl_dome2*liquidState.lambda)/(R_tank_int+y) "Internal convective heat transfer at dome 2";
        hintv0 = (Nuhv0*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 0";
        hintv1 = (Nuhv1*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 1";
        hintv2 = (Nuhv2*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 2";
        hintv3 = (Nuhv3*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 3";
        hintv4 = (Nuhv4*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 4";
        hintv5 = (Nuhv5*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 5";
        hintv6 = (Nuhv6*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 6";
        hintv7 = (Nuhv7*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at wall segment 7";
        hintv_dome1 = (Nuhv_dome1*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at dome 1";
        hintv_dome2 = (Nuhv_dome2*vapourState.lambda)/(R_tank_int-y) "Internal convective heat transfer of the vapour at dome 2";

        hradl0 = Modelica.Constants.sigma*0.09*(Tw0^2 + Tl^2)*(Tw0 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 0";
        hradl1 = Modelica.Constants.sigma*0.09*(Tw1^2 + Tl^2)*(Tw1 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 1";
        hradl2 = Modelica.Constants.sigma*0.09*(Tw2^2 + Tl^2)*(Tw2 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 2";
        hradl3 = Modelica.Constants.sigma*0.09*(Tw3^2 + Tl^2)*(Tw3 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 3";
        hradl4 = Modelica.Constants.sigma*0.09*(Tw4^2 + Tl^2)*(Tw4 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 4";
        hradl5 = Modelica.Constants.sigma*0.09*(Tw5^2 + Tl^2)*(Tw5 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 5";
        hradl6 = Modelica.Constants.sigma*0.09*(Tw6^2 + Tl^2)*(Tw6 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 6";
        hradl7 = Modelica.Constants.sigma*0.09*(Tw7^2 + Tl^2)*(Tw7 + Tl) "Internal radiation equivalent convective coefficient for the liquid segment 7";
        hradl_dome1 = Modelica.Constants.sigma*0.09*(Twdome1^2 + Tl^2)*(Twdome1 + Tl) "Internal radiation equivalent convective coefficient at dome 1";
        hradl_dome2 = Modelica.Constants.sigma*0.09*(Twdome2^2 + Tl^2)*(Twdome2 + Tl) "Internal radiation equivalent convective coefficient at dome 2";
        hradv = 0 "Internal radiation equivalent convective coefficient for the vapour";

        htotl0 = hintl0 + hradl0 "Total internal heat transfer coefficient for the liquid segment 0";
        htotl1 = hintl1 + hradl1 "Total internal heat transfer coefficient for the liquid segment 1";
        htotl2 = hintl2 + hradl2 "Total internal heat transfer coefficient for the liquid segment 2";
        htotl3 = hintl3 + hradl3 "Total internal heat transfer coefficient for the liquid segment 3";
        htotl4 = hintl4 + hradl4 "Total internal heat transfer coefficient for the liquid segment 4";
        htotl5 = hintl5 + hradl5 "Total internal heat transfer coefficient for the liquid segment 5";
        htotl6 = hintl6 + hradl6 "Total internal heat transfer coefficient for the liquid segment 6";
        htotl7 = hintl7 + hradl7 "Total internal heat transfer coefficient for the liquid segment 7";
        htotl_dome1 = hintl_dome1 + hradl_dome1 "Total internal heat transfer coefficient at dome 1";
        htotl_dome2 = hintl_dome2 + hradl_dome2 "Total internal heat transfer coefficient at dome 2";
        htotv0 = hintv0 + hradv "Total internal heat transfer coefficient for the vapour segment 0";
        htotv1 = hintv1 + hradv "Total internal heat transfer coefficient for the vapour segment 1";
        htotv2 = hintv2 + hradv "Total internal heat transfer coefficient for the vapour segment 2";
        htotv3 = hintv3 + hradv "Total internal heat transfer coefficient for the vapour segment 3";
        htotv4 = hintv4 + hradv "Total internal heat transfer coefficient for the vapour segment 4";
        htotv5 = hintv5 + hradv "Total internal heat transfer coefficient for the vapour segment 5";
        htotv6 = hintv6 + hradv "Total internal heat transfer coefficient for the vapour segment 6";
        htotv7 = hintv7 + hradv "Total internal heat transfer coefficient for the vapour segment 7";
        htotv_dome1 = hintv_dome1 + hradv "Total internal heat transfer coefficient for the vapour dome 1";
        htotv_dome2 = hintv_dome2 + hradv "Total internal heat transfer coefficient for the vapour dome 2";

        y0 = y + R_tank_int "Level (referred to the bottom)";
        Vl = L*(R_tank_int^2*acos(-y/R_tank_int) + y*sqrt(R_tank_int^2 - y^2)) + pi*(y0 + dR)^2*(R_eq - 1/3*(y0 + dR)) - 2* (L_star*(R_star^2*acos(-y/R_star) + y*sqrt(R_star^2 - y^2)))
        "Liquid volume";
        Awl = 2*R_tank_int*acos(-y/R_tank_int)*L + 2*pi*R_eq*(y0 + dR) - 2* (2*R_star*acos(-y/R_star)*L_star)
                                                                                 "Metal-liquid interface area";
        //Awl = 2*R_tank_int*acos(-y/R_tank_int)*L + 2*pi*R_eq*(y0 + dR) - 2* (2*pi*R_eq*(R_eq - AR*R_tank_int)*((y0 + dR)/R_eq))
        //                                                                         "Metal-liquid interface area";
        Asup = 2*sqrt(R_tank_int^2 - y^2)*L + pi*(R_eq*cos(asin((y)/R_eq)))^2 - 2* (2*sqrt(R_star^2 - y^2)*L_star) "Liquid-vapour interface area";

        Vt = Vl + Vv "Total tank volume";
        Awt = Awv + Awl "Total tank area";
        //Awv = 2*pi*R_tank_int*L - Awl "Metal-vapour interface area";

        // Define difference in height between the separation lines that divide the different wall segments
        dh_outer = R_tank_int*(1-cos(Modelica.Units.Conversions.from_deg(22.5))) "Height of wall segments 0 and 4 (top and bottom)";
        dh_between = R_tank_int*(1-cos(Modelica.Units.Conversions.from_deg(22.5+45))) - dh_outer "Height of wall segments 1,3,5, and 7";
        dh_inner = (R_tank_int - dh_outer - dh_between)*2 "Height of wall segments 2 and 6 (central sections)";

        // Calculate the surface area available for heat transfer on the inner tank, for each wall segment
        Awt0 = 2*R_tank_int*acos(-(-(dh_inner/2+dh_between))/R_tank_int)*L  "Inner wall surface area of wall segment 0";
        Awt1 = (2*R_tank_int*acos(-(-(dh_inner/2))/R_tank_int)*L - Awt0)/2  "Inner wall surface area of wall segment 1";
        Awt2 = (2*R_tank_int*acos(-(dh_inner/2)/R_tank_int)*L - Awt0 - 2*Awt1)/2  "Inner wall surface area of wall segment 2";
        Awt3 = Awt1              "Inner wall surface area of wall segment 3";
        Awt4 = Awt0              "Inner wall surface area of wall segment 4";
        Awt5 = Awt1              "Inner wall surface area of wall segment 5";
        Awt6 = Awt2              "Inner wall surface area of wall segment 6";
        Awt7 = Awt1              "Inner wall surface area of wall segment 7";
        //Awt_dome1 = 0.5 * 4*pi*R_tank_int^2*((1+2*AR^1.6075)/3)^(1/1.6075) "Inner wall surface area of dome 1";
        Awt_dome1 = 2*pi*R_eq*(R_tank_int*AR) "Inner wall surface area of dome 1";
        Awt_dome2 = Awt_dome1    "Inner wall surface area of dome 2";
        // Calculate surface area in contact with the liquid for each wall segment
        if y >= (dh_inner/2 + dh_between) then
          Awl0 = Awt0           "Wall surface area touching the liquid at wall segment 0";
          Awl1 = Awt1           "Wall surface area touching the liquid at wall segment 1";
          Awl2 = Awt2           "Wall surface area touching the liquid at wall segment 2";
          Awl3 = Awt3           "Wall surface area touching the liquid at wall segment 3";
          Awl4 = 2*R_tank_int*acos(-y/R_tank_int)*L - Awt7 - Awt6 - Awt5 - Awt3 - Awt2 - Awt1 - Awt0 "Wall surface area touching the liquid at wall segment 4";
          Awl5 = Awt5           "Wall surface area touching the liquid at wall segment 5";
          Awl6 = Awt6           "Wall surface area touching the liquid at wall segment 6";
          Awl7 = Awt7           "Wall surface area touching the liquid at wall segment 7";
        elseif y >= (dh_inner/2) then
          Awl0 = Awt0           "Wall surface area touching the liquid at wall segment 0";
          Awl1 = Awt1           "Wall surface area touching the liquid at wall segment 1";
          Awl2 = Awt2           "Wall surface area touching the liquid at wall segment 2";
          Awl3 = (2*R_tank_int*acos(-y/R_tank_int)*L - Awt7 - Awt6 - Awt2 - Awt1 - Awt0)/2 "Wall surface area touching the liquid at wall segment 3";
          Awl4 = 0              "Wall surface area touching the liquid at wall segment 4";
          Awl5 = (2*R_tank_int*acos(-y/R_tank_int)*L - Awt7 - Awt6 - Awt2 - Awt1 - Awt0)/2 "Wall surface area touching the liquid at wall segment 5";
          Awl6 = Awt6           "Wall surface area touching the liquid at wall segment 6";
          Awl7 = Awt7           "Wall surface area touching the liquid at wall segment 7";
        elseif y >= -(dh_inner/2) then
          Awl0 = Awt0           "Wall surface area touching the liquid at wall segment 0";
          Awl1 = Awt1           "Wall surface area touching the liquid at wall segment 1";
          Awl2 = (2*R_tank_int*acos(-y/R_tank_int)*L - Awt7 - Awt1 - Awt0)/2 "Wall surface area touching the liquid at wall segment 2";
          Awl3 = 0              "Wall surface area touching the liquid at wall segment 3";
          Awl4 = 0              "Wall surface area touching the liquid at wall segment 4";
          Awl5 = 0              "Wall surface area touching the liquid at wall segment 5";
          Awl6 = (2*R_tank_int*acos(-y/R_tank_int)*L - Awt7 - Awt1 - Awt0)/2 "Wall surface area touching the liquid at wall segment 6";
          Awl7 = Awt7           "Wall surface area touching the liquid at wall segment 7";
        elseif y >= -(dh_inner/2 + dh_between) then
          Awl0 = Awt0           "Wall surface area touching the liquid at wall segment 0";
          Awl1 = (2*R_tank_int*acos(-y/R_tank_int)*L - Awt0)/2 "Wall surface area touching the liquid at wall segment 1";
          Awl2 = 0              "Wall surface area touching the liquid at wall segment 2";
          Awl3 = 0              "Wall surface area touching the liquid at wall segment 3";
          Awl4 = 0              "Wall surface area touching the liquid at wall segment 4";
          Awl5 = 0           "Wall surface area touching the liquid at wall segment 5";
          Awl6 = 0           "Wall surface area touching the liquid at wall segment 6";
          Awl7 = (2*R_tank_int*acos(-y/R_tank_int)*L - Awt0)/2 "Wall surface area touching the liquid at wall segment 7";
        else
          Awl0 = 2*R_tank_int*acos(-y/R_tank_int)*L "Wall surface area touching the liquid at wall segment 0";
          Awl1 = 0              "Wall surface area touching the liquid at wall segment 1";
          Awl2 = 0              "Wall surface area touching the liquid at wall segment 2";
          Awl3 = 0              "Wall surface area touching the liquid at wall segment 3";
          Awl4 = 0              "Wall surface area touching the liquid at wall segment 4";
          Awl5 = 0           "Wall surface area touching the liquid at wall segment 5";
          Awl6 = 0           "Wall surface area touching the liquid at wall segment 6";
          Awl7 = 0           "Wall surface area touching the liquid at wall segment 7";
        end if;
        // Calculate surface area in contact with the liquid at the domes
        //Awl_dome1 = Awt_dome1 * (R_tank_int+y)/(2*R_tank_int) "Wall surface area touching the liquid at dome 1 - assuming the same area fraction as for a sphere for the same liquid level ";
        Awl_dome1 = pi*R_eq*(y0 + dR) - (2*R_star*acos(-y/R_star)*L_star) "Wall surface area touching the liquid at dome 1";
        Awl_dome2 = Awl_dome1 "Wall surface area touching the liquid at dome 2";
        // Calculate surface area in contact with the vapour for each wall segment
        Awv0 = Awt0 - Awl0     "Wall surface area touching the vapour at wall segment 0";
        Awv1 = Awt1 - Awl1     "Wall surface area touching the vapour at wall segment 1";
        Awv2 = Awt2 - Awl2     "Wall surface area touching the vapour at wall segment 2";
        Awv3 = Awt3 - Awl3     "Wall surface area touching the vapour at wall segment 3";
        Awv4 = Awt4 - Awl4     "Wall surface area touching the vapour at wall segment 4";
        Awv5 = Awt5 - Awl5     "Wall surface area touching the vapour at wall segment 5";
        Awv6 = Awt6 - Awl6     "Wall surface area touching the vapour at wall segment 6";
        Awv7 = Awt7 - Awl7     "Wall surface area touching the vapour at wall segment 7";
        Awv_dome1 = Awt_dome1 - Awl_dome1 "Wall surface area touching the vapour at dome 1";
        Awv_dome2 = Awt_dome2 - Awl_dome2 "Wall surface area touching the vapour at dome 2";

        // Heat flows of the wall segments
        Qwl0 = htotl0*Awl0*(Tw0 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv0 = htotv0*Awv0*(Tw0 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl1 = htotl1*Awl1*(Tw1 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv1 = htotv1*Awv1*(Tw1 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl2 = htotl2*Awl2*(Tw2 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv2 = htotv2*Awv2*(Tw2 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl3 = htotl3*Awl3*(Tw3 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv3 = htotv3*Awv3*(Tw3 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl4 = htotl4*Awl4*(Tw4 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv4 = htotv4*Awv4*(Tw4 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl5 = htotl5*Awl5*(Tw5 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv5 = htotv5*Awv5*(Tw5 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl6 = htotl6*Awl6*(Tw6 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv6 = htotv6*Awv6*(Tw6 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl7 = htotl7*Awl7*(Tw7 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv7 = htotv7*Awv7*(Tw7 - Tv) "Heat flow from metal wall to vapour volume";
        Qwl_dome1 = htotl_dome1*Awl_dome1*(Twdome1 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv_dome1 = htotv_dome1*Awv_dome1*(Twdome1 - Tv) "Heat flow from metal wall to liquid volume";
        Qwl_dome2 = htotl_dome2*Awl_dome2*(Twdome2 - Tl) "Heat flow from metal wall to liquid volume";
        Qwv_dome2 = htotv_dome2*Awv_dome2*(Twdome2 - Tv) "Heat flow from metal wall to liquid volume";

        // Split the internal heat flow to the liquid and vapour, according to the volume
        A_frac = 1;//(Awl0+Awl1+Awl2+Awl3+Awl4+Awl5+Awl6+Awl7)/(Awt0+Awt1+Awt2+Awt3+Awt4+Awt5+Awt6+Awt7) "Liquid area fraction, neglecting the dome volumes";
        Qvint = Qint*(1-A_frac) "Fraction of the introduced heat that goes towards the vapour";
        Qlint = Qint*A_frac "Fraction of the introduced heat that goes towards the liquid";

        Qwl = Qwl0 + Qwl1 + Qwl2 + Qwl3 + Qwl4 + Qwl5 + Qwl6 + Qwl7 + Qwl_dome1 + Qwl_dome2;
        Qwv = Qwv0 + Qwv1 + Qwv2 + Qwv3 + Qwv4 + Qwv5 + Qwv6 + Qwv7 + Qwv_dome1 + Qwv_dome2;

        Qltot = Qwl + Qlint;
        Qvtot = Qwv + Qvint;

        // Fluid properties
        sat = Medium.setSat_p(P);
        liquidState = Medium.setState_ph(P, hl, 2);
        Tl = Medium.temperature_ph(P, hl, 1);
        rhol = Medium.density_ph(P, hl, 1);
        ul = Medium.specificInternalEnergy(liquidState);
        vapourState = Medium.setState_ph(P, hv, 2);
        Tv = Medium.temperature_ph(P, hv, 1);
        rhov = Medium.density_ph(P, hv, 1);
        uv = Medium.specificInternalEnergy(vapourState);

        hls = sat.hl;
        hvs = sat.hv;
        Ts = sat.Tsat;
        Dh = hvs - hls;

        // Boundary conditions
        LH2_outlet.P = P;
        LH2_outlet.m_flow = ql;
        LH2_outlet.h_outflow = hl;
        VH2_outlet.P = P;
        VH2_outlet.m_flow = qv;
        VH2_outlet.h_outflow = hv;
        wall0.T = Tw0;
        wall1.T = Tw1;
        wall2.T = Tw2;
        wall3.T = Tw3;
        wall4.T = Tw4;
        wall5.T = Tw5;
        wall6.T = Tw6;
        wall7.T = Tw7;
        dome1.T = Twdome1;
        dome2.T = Twdome2;
        Fluid_Heat_Port.T = Tv;
        Qwv0 + Qwl0 = wall0.Q_flow;
        Qwv1 + Qwl1 = wall1.Q_flow;
        Qwv2 + Qwl2 = wall2.Q_flow;
        Qwv3 + Qwl3 = wall3.Q_flow;
        Qwv4 + Qwl4 = wall4.Q_flow;
        Qwv5 + Qwl5 = wall5.Q_flow;
        Qwv6 + Qwl6 = wall6.Q_flow;
        Qwv7 + Qwl7 = wall7.Q_flow;
        Qwv_dome1 + Qwl_dome1 = dome1.Q_flow;
        Qwv_dome2 + Qwl_dome2 = dome2.Q_flow;
        Qint = Fluid_Heat_Port.Q_flow "Heat introduced directly into the fluid (e.g. by an internal heat exchanger)";

        // Assertions
        assert(Ml > 0 + eps, "Liquid fuel depleted");
        assert(Mv > 0 + eps, "Gaseous fuel depleted");
        assert(Vl > 0 + eps, "Liquid fuel depleted");
        assert(Vv > 0 + eps, "Gaseous fuel depleted");
        assert(y < R_tank_int - eps, "Liquid level above max tank radius");
        assert(y > -R_tank_int + eps, "Liquid level below min tank radius");

  initial equation
        if initOpt == DynTherM.Choices.InitOpt.noInit then
          // do nothing

        elseif initOpt == DynTherM.Choices.InitOpt.fixedState then
          if not noInitialPressure then
            P = P_start;
          end if;
          hl = hlstart;
          hv = hvstart;
          Vl = Vlstart;

        elseif initOpt == DynTherM.Choices.InitOpt.steadyState then
          if not noInitialPressure then
            der(P) = 0;
          end if;
          der(hl) = 0;
          der(hv) = 0;
          der(Vl) = 0;
        else
          assert(false, "Unsupported initialisation option");
        end if;
          annotation (Dialog(tab="Initialisation"),
                      Dialog(tab="Initialisation"),
          Documentation(info="<HTML>
<p>Simplified model of a drum for drum boilers and fire-tube boilers. This model assumes
<ul>
<li>Thermodynamic equiibrium between the liquid, vapour, and metal wall
<li>Perfect separation of the liquid and vapour phase
</ul></p>
<p>The model has two state variables the pressure <code>p</code> and the liquid volume <code>Vl</code>. It is possible to extend it,
adding a specific geometry and the computation of the level from the liquid volume. In that case, one may want to use the level as a state.
</p>
</HTML>",     revisions="<html>
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
  end tmp;
end HydrogenTank;
