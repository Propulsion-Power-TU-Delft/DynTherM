within ThermalManagement.Materials;
partial model CellProperties

  constant Real poly_cm_SOC0[2] "Polynomial coefficient of specific heat capacity fitting at SOC=0%";
  constant Real poly_cm_SOC50[2] "Polynomial coefficient of specific heat capacity fitting at SOC=50%";
  constant Real poly_cm_SOC100[2] "Polynomial coefficient of specific heat capacity fitting at SOC=100%";

  constant Real poly_lambda_SOC0[2] "Polynomial coefficient of thermal conductivity fitting at SOC=0%";
  constant Real poly_lambda_SOC50[2] "Polynomial coefficient of thermal conductivity fitting at SOC=50%";
  constant Real poly_lambda_SOC100[2] "Polynomial coefficient of thermal conductivity fitting at SOC=100%";

  constant Real SOC_vec[3]={0, 50, 100} "State-of-charge values at which the properties are evaluated [%]";

end CellProperties;
