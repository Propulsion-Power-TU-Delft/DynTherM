within ThermalManagement.Materials;
partial model NCA18650

  // References:
  // [1] K. A. Murashko. Determination of the through-plane thermal conductivity and specific heat capacity of a Li-ion cylindrical cell, 2020.

  extends Modelica.Icons.MaterialProperty;
  extends Materials.CellProperties(
    poly_cm_SOC0={4.36109572e-01, 6.81897453e+02},
    poly_cm_SOC50={4.54894239e-01, 6.84867474e+02},
    poly_cm_SOC100={4.68770564e-01, 6.92490560e+02},
    poly_lambda_SOC0={-9.03701112e-04, 9.78691428e-01},
    poly_lambda_SOC50={-9.73382060e-04, 1.10435452},
    poly_lambda_SOC100={-3.95172100e-04, 1.01188181})
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));

end NCA18650;
