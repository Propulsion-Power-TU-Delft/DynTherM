within DynTherM.Materials;
model PolestarCellInPlane
  "Material used for battery cells of Polestar - in-plane thermal properties"
  extends Modelica.Icons.MaterialProperty;
  extends Materials.Properties(
    rho=2740,
    lambda=20,
    cm=1204);
  annotation (Documentation(info="<html>
</html>"));
end PolestarCellInPlane;
